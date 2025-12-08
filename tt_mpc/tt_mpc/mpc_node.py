#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from tt_interfaces.msg import PolytopeObstacleArray

import casadi as ca
import numpy as np
from numpy import sin, cos, tan
from scipy.optimize import linprog 

class DCBFNode(Node):
    def __init__(self):
        super().__init__('dcbf_mpc_node')
        
        # =====================================================
        #             MODEL PARAMETERS
        # =====================================================
        self.L0 = 0.42   
        self.M0 = -0.02  
        self.L1 = 0.537  

        self.TRACTOR_W = 0.35
        self.TRAILER_W = 0.35
        self.TRACTOR_FRONT = self.L0 + 0.07
        self.TRACTOR_BACK = 0.06
        self.TRAILER_FRONT = self.L1 + 0.015
        self.TRAILER_BACK = 0.064

        self.V0_MAX = 0.5
        self.DELTA0_MAX = np.radians(50)
        self.BETA_MAX = np.radians(60)

        self.N = 20          
        self.dt = 0.1        
        self.N_cbf = 15 
        self.MAX_OBS = 4  # Fixed number of obstacles     

        self.Q = np.diag([10.0, 10.0, 0.0, 0.0])  
        self.R = np.diag([0.1, 1.0])               
        self.R_smooth = np.diag([0.1, 0.5])        
        self.Q_terminal = self.Q * 10
        self.gamma = 0.9
        self.margin_dist = 0.2
        self.p_omega = 10.0

        # =====================================================
        #             ROS SETUP
        # =====================================================
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Path, '/mpc_trajectory', 10)
        
        # Subscribers
        self.create_subscription(Float64MultiArray, '/truck_trailer/state', self.state_cb, 10)
        self.create_subscription(PolytopeObstacleArray, '/obstacles', self.obs_cb, 10)
        
        qos_goal = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
            depth=10
        )
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, qos_goal)

        # Internal State
        self.current_state = np.zeros(4) 
        self.state_received = False
        self.goal_received = False
        self.u_prev = np.zeros(2)
        self.x_goal = np.array([2.0, 2.0, 0.0, 0.0]) # Default Goal

        # --- Obstacle Storage ---
        # We store obstacles as a list of (A, b) tuples in World Frame
        # Initialize with 4 dummy obstacles far away
        self.active_obstacles = []
        dummy_A, dummy_b = self.get_polytope_from_box(0.1, 0.1, 0.1)
        # Move them far away (x=100, x=101, etc)
        for i in range(self.MAX_OBS):
            A_w, b_w = self.transform_polytope(dummy_A, dummy_b, 100.0 + i*2.0, 100.0, 0.0)
            self.active_obstacles.append((A_w, b_w))

        # Add one real test obstacle at (1.0, 1.0) for immediate testing
        obs_size = (0.6, 0.4)
        A_local, b_local = self.get_polytope_from_box(obs_size[1]/2, obs_size[0]/2, obs_size[0]/2)
        A_real, b_real = self.transform_polytope(A_local, b_local, 1.0, 1.0, 0.0)
        self.active_obstacles[0] = (A_real, b_real)

        self.get_logger().info("Building MPC (Fatrop) with 4 Obstacles...")
        self.setup_mpc()
        self.get_logger().info("MPC Ready.")

        self.timer = self.create_timer(0.1, self.control_loop)

    # =====================================================
    #             HELPER FUNCTIONS
    # =====================================================
    def get_polytope_from_box(self, w, lf, lb):
        A = np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])
        b = np.array([lf, lb, w/2, w/2])
        return A, b

    def transform_polytope(self, A, b, x, y, theta):
        R = np.array([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        A_world = A @ R.T
        translation = np.array([x, y])
        b_world = b + A @ R.T @ translation
        return A_world, b_world

    def get_tractor_position(self, x1, y1, theta1, theta0):
        hx = x1 + self.L1 * cos(theta1)
        hy = y1 + self.L1 * sin(theta1)
        x0 = hx + self.M0 * cos(theta0)
        y0 = hy + self.M0 * sin(theta0)
        return x0, y0

    def compute_separation_dual(self, A_obs, b_obs, A_robot, b_robot):
        n_obs = len(b_obs)
        n_robot = len(b_robot)
        c = np.concatenate([-b_robot, b_obs])
        A_eq = np.hstack([A_robot.T, -A_obs.T])
        b_eq = np.zeros(2)
        bounds = [(0, None)] * (n_robot + n_obs)
        res = linprog(c, A_eq=A_eq, b_eq=b_eq, bounds=bounds, method='highs')
        if res.success:
            return -res.fun, res.x[n_robot:], res.x[:n_robot] 
        else:
            return 0.1, np.ones(n_obs)/n_obs, np.ones(n_robot)/n_robot

    # =====================================================
    #             MPC SETUP
    # =====================================================
    def setup_mpc(self):
        opti = ca.Opti()
        
        # 1. Constant Polytopes
        A_tractor_np, b_tractor_np = self.get_polytope_from_box(self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        A_trailer_np, b_trailer_np = self.get_polytope_from_box(self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        
        b_tractor_dm = ca.DM(b_tractor_np)
        b_trailer_dm = ca.DM(b_trailer_np)
        A_tractor_dm = ca.DM(A_tractor_np)
        A_trailer_dm = ca.DM(A_trailer_np)
        
        # 2. VARIABLES (PHASE 1)
        X, U = [], []
        # These will be lists of lists: Lambda_tractor[k][obs_i]
        Lambda_tractor, Mu_tractor, Omega_tractor = [], [], []
        Lambda_trailer, Mu_trailer, Omega_trailer = [], [], []
        
        for k in range(self.N):
            X.append(opti.variable(4))
            U.append(opti.variable(2))
            
            if k < self.N_cbf:
                # Per-stage lists
                l_tr_k, m_tr_k, o_tr_k = [], [], []
                l_tl_k, m_tl_k, o_tl_k = [], [], []
                
                for _ in range(self.MAX_OBS):
                    l_tr_k.append(opti.variable(4))
                    m_tr_k.append(opti.variable(4))
                    o_tr_k.append(opti.variable(1))
                    
                    l_tl_k.append(opti.variable(4))
                    m_tl_k.append(opti.variable(4))
                    o_tl_k.append(opti.variable(1))
                    
                Lambda_tractor.append(l_tr_k); Mu_tractor.append(m_tr_k); Omega_tractor.append(o_tr_k)
                Lambda_trailer.append(l_tl_k); Mu_trailer.append(m_tl_k); Omega_trailer.append(o_tl_k)
        
        X.append(opti.variable(4)) 
        
        # 3. PARAMETERS
        self.par_X0 = opti.parameter(4)
        self.par_X_ref = opti.parameter(4)
        self.par_U_prev = opti.parameter(2)
        
        # Obstacle Params: Stacked (4*MAX_OBS, 2) and (4*MAX_OBS)
        self.par_A_obs = opti.parameter(4 * self.MAX_OBS, 2)
        self.par_b_obs = opti.parameter(4 * self.MAX_OBS)
        
        # Initial separation parameters (one per obstacle)
        self.par_h0_tractor = opti.parameter(self.MAX_OBS)
        self.par_h0_trailer = opti.parameter(self.MAX_OBS)

        # 4. CONSTRAINTS (PHASE 2)
        cost = 0
        for k in range(self.N):
            xk = X[k]; uk = U[k]; x_next = X[k+1]
            
            # Cost
            err = xk - self.par_X_ref
            cost += ca.mtimes([err.T, self.Q, err]) + ca.mtimes([uk.T, self.R, uk])
            if k == 0: du = uk - self.par_U_prev
            else: du = uk - U[k-1]
            cost += ca.mtimes([du.T, self.R_smooth, du])
            
            # Dynamics
            x1, y1, th1, th0 = xk[0], xk[1], xk[2], xk[3]
            V0, delta0 = uk[0], uk[1]
            beta = th0 - th1
            V1 = V0 * ca.cos(beta) + self.M0 * (V0 * ca.tan(delta0) / self.L0) * ca.sin(beta)
            
            x_next_model = ca.vertcat(
                x1 + V1 * ca.cos(th1) * self.dt,
                y1 + V1 * ca.sin(th1) * self.dt,
                th1 + (V0 * ca.sin(beta) / self.L1 - self.M0 * (V0 * ca.tan(delta0) / self.L0) * ca.cos(beta) / self.L1) * self.dt,
                th0 + V0 * ca.tan(delta0) / self.L0 * self.dt
            )
            opti.subject_to(x_next == x_next_model)
            
            if k == 0: opti.subject_to(X[0] == self.par_X0)
            
            opti.subject_to(opti.bounded(-self.V0_MAX, uk[0], self.V0_MAX))
            opti.subject_to(opti.bounded(-self.DELTA0_MAX, uk[1], self.DELTA0_MAX))
            opti.subject_to(opti.bounded(-self.BETA_MAX, xk[3] - xk[2], self.BETA_MAX))

            # CBF Constraints
            if k < self.N_cbf:
                x1_n, y1_n, th1_n, th0_n = x_next_model[0], x_next_model[1], x_next_model[2], x_next_model[3]
                hx_n = x1_n + self.L1 * ca.cos(th1_n); hy_n = y1_n + self.L1 * ca.sin(th1_n)
                x0_n = hx_n + self.M0 * ca.cos(th0_n); y0_n = hy_n + self.M0 * ca.sin(th0_n)
                
                R_trac = ca.horzcat(ca.vertcat(ca.cos(th0_n), ca.sin(th0_n)), ca.vertcat(-ca.sin(th0_n), ca.cos(th0_n)))
                T_trac = ca.vertcat(x0_n, y0_n)
                R_trail = ca.horzcat(ca.vertcat(ca.cos(th1_n), ca.sin(th1_n)), ca.vertcat(-ca.sin(th1_n), ca.cos(th1_n)))
                T_trail = ca.vertcat(x1_n, y1_n)
                
                # Iterate over Obstacles
                for obs_i in range(self.MAX_OBS):
                    # Slice parameters
                    A_obs_i = self.par_A_obs[obs_i*4 : obs_i*4+4, :]
                    b_obs_i = self.par_b_obs[obs_i*4 : obs_i*4+4]
                    h0_tr_i = self.par_h0_tractor[obs_i]
                    h0_tl_i = self.par_h0_trailer[obs_i]
                    
                    # Variables
                    lam_tr = Lambda_tractor[k][obs_i]; mu_tr = Mu_tractor[k][obs_i]; om_tr = Omega_tractor[k][obs_i]
                    lam_tl = Lambda_trailer[k][obs_i]; mu_tl = Mu_trailer[k][obs_i]; om_tl = Omega_trailer[k][obs_i]
                    
                    # --- TRACTOR ---
                    opti.subject_to(lam_tr >= 0); opti.subject_to(mu_tr >= 0); opti.subject_to(om_tr >= 0)
                    
                    term1 = -ca.mtimes(b_tractor_dm.T, mu_tr) 
                    term2 = ca.mtimes((ca.mtimes([A_obs_i, T_trac]) - b_obs_i).T, lam_tr)
                    opti.subject_to(term1 + term2 >= om_tr * self.gamma**(k+1) * (h0_tr_i - self.margin_dist) + self.margin_dist)
                    
                    opti.subject_to(ca.mtimes(A_tractor_dm.T, mu_tr) + ca.mtimes([R_trac.T, ca.mtimes([A_obs_i.T, lam_tr])]) == 0)
                    tmp_tr = ca.mtimes(A_obs_i.T, lam_tr)
                    opti.subject_to(ca.mtimes(tmp_tr.T, tmp_tr) <= 1)
                    cost += self.p_omega * (om_tr - 1)**2

                    # --- TRAILER ---
                    opti.subject_to(lam_tl >= 0); opti.subject_to(mu_tl >= 0); opti.subject_to(om_tl >= 0)
                    
                    term1_tl = -ca.mtimes(b_trailer_dm.T, mu_tl)
                    term2_tl = ca.mtimes((ca.mtimes([A_obs_i, T_trail]) - b_obs_i).T, lam_tl)
                    opti.subject_to(term1_tl + term2_tl >= om_tl * self.gamma**(k+1) * (h0_tl_i - self.margin_dist) + self.margin_dist)
                    
                    opti.subject_to(ca.mtimes(A_trailer_dm.T, mu_tl) + ca.mtimes([R_trail.T, ca.mtimes([A_obs_i.T, lam_tl])]) == 0)
                    tmp_tl = ca.mtimes(A_obs_i.T, lam_tl)
                    opti.subject_to(ca.mtimes(tmp_tl.T, tmp_tl) <= 1)
                    cost += self.p_omega * (om_tl - 1)**2

        cost += ca.mtimes([(X[self.N] - self.par_X_ref).T, self.Q_terminal, (X[self.N] - self.par_X_ref)])
        opti.minimize(cost)
        
        opts = {"fatrop.print_level": 0, "print_time": 0, "fatrop.max_iter": 100, "fatrop.tol": 1e-4, "structure_detection": "auto", "expand": True}
        opti.solver('fatrop', opts)
        
        self.opti = opti; self.var_X = X; self.var_U = U
        self.var_Lambda_tr = Lambda_tractor; self.var_Mu_tr = Mu_tractor
        self.var_Lambda_tl = Lambda_trailer; self.var_Mu_tl = Mu_trailer

    # =====================================================
    #             CALLBACKS
    # =====================================================
    def state_cb(self, msg):
        d = msg.data
        self.current_state = np.array([d[0], d[1], d[3], d[2]])
        self.state_received = True

    def goal_cb(self, msg):
        q = msg.pose.orientation
        yaw = np.arctan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.x_goal = np.array([msg.pose.position.x, msg.pose.position.y, yaw, yaw])
        self.goal_received = True
        self.get_logger().info(f"New Goal Received: {self.x_goal}")

    def obs_cb(self, msg):
        # Update active obstacles list
        new_obs_list = []
        idx = 0
        for obs in msg.obstacles:
            if idx >= self.MAX_OBS: break
            if obs.num_constraints != 4: continue
            
            a_local = np.array(obs.a_vec).reshape((4, 2))
            b_local = np.array(obs.b_vec)
            new_obs_list.append((a_local, b_local))
            idx += 1
            
        # Fill remainder with dummy far-away obstacles
        dummy_A, dummy_b = self.get_polytope_from_box(0.1, 0.1, 0.1)
        while len(new_obs_list) < self.MAX_OBS:
            offset = 100.0 + len(new_obs_list)*2.0
            A_w, b_w = self.transform_polytope(dummy_A, dummy_b, offset, 100.0, 0.0)
            new_obs_list.append((A_w, b_w))
            
        self.active_obstacles = new_obs_list

    def control_loop(self):
        if not self.state_received:
            return

        x1, y1, th1, th0 = self.current_state
        x0, y0 = self.get_tractor_position(x1, y1, th1, th0)
        
        A_tr, b_tr = self.get_polytope_from_box(self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        A_tl, b_tl = self.get_polytope_from_box(self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        
        A_tr_w, b_tr_w = self.transform_polytope(A_tr, b_tr, x0, y0, th0)
        A_tl_w, b_tl_w = self.transform_polytope(A_tl, b_tl, x1, y1, th1)
        
        # Prepare Data Structures for 4 Obstacles
        A_obs_flat = np.zeros((4 * self.MAX_OBS, 2))
        b_obs_flat = np.zeros(4 * self.MAX_OBS)
        h0_tr_list = []
        h0_tl_list = []
        
        dual_init_data = [] # Store (l_tr, m_tr, l_tl, m_tl) for each obstacle

        for i in range(self.MAX_OBS):
            A_obs_i, b_obs_i = self.active_obstacles[i]
            
            # 1. Fill Parameter Arrays
            A_obs_flat[i*4 : i*4+4, :] = A_obs_i
            b_obs_flat[i*4 : i*4+4] = b_obs_i
            
            # 2. Compute Init Duals
            h_tr, l_tr, m_tr = self.compute_separation_dual(A_obs_i, b_obs_i, A_tr_w, b_tr_w)
            h_tl, l_tl, m_tl = self.compute_separation_dual(A_obs_i, b_obs_i, A_tl_w, b_tl_w)
            
            h0_tr_list.append(max(h_tr, 0.01))
            h0_tl_list.append(max(h_tl, 0.01))
            dual_init_data.append((l_tr, m_tr, l_tl, m_tl))

        # 3. Set Parameters
        self.opti.set_value(self.par_X0, self.current_state)
        self.opti.set_value(self.par_X_ref, self.x_goal)
        self.opti.set_value(self.par_U_prev, self.u_prev)
        self.opti.set_value(self.par_A_obs, A_obs_flat)
        self.opti.set_value(self.par_b_obs, b_obs_flat)
        self.opti.set_value(self.par_h0_tractor, h0_tr_list)
        self.opti.set_value(self.par_h0_trailer, h0_tl_list)
        
        # 4. Warm Start
        for k in range(self.N + 1):
            alpha = k / self.N
            interp = (1 - alpha) * self.current_state + alpha * self.x_goal
            self.opti.set_initial(self.var_X[k], interp)
        
        for k in range(self.N_cbf):
            for i in range(self.MAX_OBS):
                l_tr, m_tr, l_tl, m_tl = dual_init_data[i]
                
                self.opti.set_initial(self.var_Lambda_tr[k][i], l_tr)
                self.opti.set_initial(self.var_Mu_tr[k][i], m_tr)
                self.opti.set_initial(self.var_Lambda_tl[k][i], l_tl)
                self.opti.set_initial(self.var_Mu_tl[k][i], m_tl)

        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.var_U[0])
            self.u_prev = u_opt
            
            msg = Twist()
            msg.linear.x = float(u_opt[0])
            msg.angular.z = float(u_opt[1])
            self.cmd_pub.publish(msg)
            self.publish_path(sol)
            
        except RuntimeError:
            self.get_logger().error("Solver failed!")
            self.cmd_pub.publish(Twist())

    def publish_path(self, sol):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for k in range(self.N + 1):
            val = sol.value(self.var_X[k])
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(val[0])
            pose.pose.position.y = float(val[1])
            path_msg.poses.append(pose)
        self.traj_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DCBFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()