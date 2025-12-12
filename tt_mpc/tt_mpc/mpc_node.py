#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from tt_interfaces.msg import PolytopeObstacleArray

import casadi as ca
import numpy as np
from numpy import sin, cos, tan
from scipy.optimize import linprog 

class DCBFNode(Node):
    def __init__(self):
        super().__init__('dcbf_mpc_node')
        
        # --- Model & Tuning ---
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
        self.N = 10
        self.dt = 0.1
        self.N_cbf = 9
        self.MAX_OBS = 4

        self.Q = np.diag([10.0, 10.0, 0.0, 0.0])  
        self.R = np.diag([0.1, 1.0])               
        self.R_smooth = np.diag([0.1, 0.5])        
        self.Q_terminal = self.Q * 10
        self.gamma = 0.9
        self.margin_dist = 0.2
        self.p_omega = 10.0

        # --- ROS Setup ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.traj_pub = self.create_publisher(Path, '/planned_path', 10)
        self.poly_pub = self.create_publisher(MarkerArray, '/mpc_polytopes', 10)
        
        self.create_subscription(Float64MultiArray, '/truck_trailer/state', self.state_cb, 10)
        self.create_subscription(PolytopeObstacleArray, '/obstacles', self.obs_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # --- State ---
        self.current_state = np.zeros(4) 
        self.state_received = False
        self.u_prev = np.zeros(2)
        self.x_goal = np.array([2.0, 2.0, 0.0, 0.0])
        
        self.warm_start_val = None 
        self.vars_shape = None

        # --- Obstacles ---
        self.active_obstacles = []
        dummy_A, dummy_b = self.get_polytope_from_box(0.1, 0.1, 0.1)
        for i in range(self.MAX_OBS):
            A_w, b_w = self.transform_polytope(dummy_A, dummy_b, 100.0 + i*2.0, 100.0, 0.0)
            self.active_obstacles.append((A_w, b_w))

        # --- Build CasADi Function ---
        self.get_logger().info("Compiling MPC Function...")
        self.setup_mpc_function()
        self.get_logger().info("MPC Compiled & Ready.")

        self.timer = self.create_timer(0.1, self.control_loop)

    # ... [Visualization Helpers omitted for brevity, identical to previous] ...
    def get_vehicle_corners(self, x, y, theta, w, lf, lb):
        hw = w / 2.0; corners_local = np.array([[lf, hw],[lf, -hw],[-lb, -hw],[-lb, hw],[lf, hw]])
        R = np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]])
        return (R @ corners_local.T).T + np.array([x, y])

    def publish_polytopes(self, x1, y1, th1, th0, x0, y0):
        msg = MarkerArray()
        # Create markers logic (same as before)
        tr = self.get_vehicle_corners(x0, y0, th0, self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        tl = self.get_vehicle_corners(x1, y1, th1, self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        
        m1 = Marker(); m1.header.frame_id="map"; m1.type=Marker.LINE_STRIP; m1.action=Marker.ADD; m1.id=0; m1.ns="tr"; m1.scale.x=0.05; m1.color.a=1.0; m1.color.b=1.0
        m2 = Marker(); m2.header.frame_id="map"; m2.type=Marker.LINE_STRIP; m2.action=Marker.ADD; m2.id=1; m2.ns="tl"; m2.scale.x=0.05; m2.color.a=1.0; m2.color.r=1.0
        
        for p in tr: pt = Point(); pt.x=p[0]; pt.y=p[1]; m1.points.append(pt)
        for p in tl: pt = Point(); pt.x=p[0]; pt.y=p[1]; m2.points.append(pt)
        msg.markers = [m1, m2]
        self.poly_pub.publish(msg)

    # =====================================================
    #             CORE LOGIC
    # =====================================================
    def get_polytope_from_box(self, w, lf, lb):
        return np.array([[1, 0], [-1, 0], [0, 1], [0, -1]]), np.array([lf, lb, w/2, w/2])

    def transform_polytope(self, A, b, x, y, theta):
        R = np.array([[cos(theta), -sin(theta)], [sin(theta),  cos(theta)]])
        return A @ R.T, b + A @ R.T @ np.array([x, y])

    def get_tractor_position(self, x1, y1, theta1, theta0):
        return x1 + self.L1*cos(theta1) + self.M0*cos(theta0), y1 + self.L1*sin(theta1) + self.M0*sin(theta0)

    def compute_separation_dual(self, A_obs, b_obs, A_robot, b_robot):
        n_obs = len(b_obs); n_robot = len(b_robot)
        c = np.concatenate([-b_robot, b_obs])
        A_eq = np.hstack([A_robot.T, -A_obs.T])
        res = linprog(c, A_eq=A_eq, b_eq=np.zeros(2), bounds=[(0, None)]*(n_robot+n_obs), method='highs')
        return (-res.fun, res.x[n_robot:], res.x[:n_robot]) if res.success else (0.01, np.zeros(n_obs), np.zeros(n_robot))

    def setup_mpc_function(self):
        opti = ca.Opti()
        
        # --- 1. Define Parameters (Symbolic Placeholders) ---
        p_X0 = opti.parameter(4)
        p_Ref = opti.parameter(4)
        p_U_prev = opti.parameter(2)
        p_A_obs = opti.parameter(4 * self.MAX_OBS, 2)
        p_b_obs = opti.parameter(4 * self.MAX_OBS)
        p_h0_tr = opti.parameter(self.MAX_OBS)
        p_h0_tl = opti.parameter(self.MAX_OBS)
        
        # --- 2. Define Variables & Collect them for Warm Start ---
        X, U = [], []
        vars_flat = [] # Flattened list of ALL decision variables
        
        # Constants
        b_tr_dm = ca.DM(np.array([self.TRACTOR_FRONT, self.TRACTOR_BACK, self.TRACTOR_W/2, self.TRACTOR_W/2]))
        b_tl_dm = ca.DM(np.array([self.TRAILER_FRONT, self.TRAILER_BACK, self.TRAILER_W/2, self.TRAILER_W/2]))
        A_tr_dm = ca.DM(np.array([[1,0],[-1,0],[0,1],[0,-1]]))
        A_tl_dm = A_tr_dm
        
        for k in range(self.N):
            xk = opti.variable(4); uk = opti.variable(2)
            X.append(xk); U.append(uk)
            vars_flat.extend([xk, uk])
            
            if k < self.N_cbf:
                # Add Duals to variables list
                for _ in range(self.MAX_OBS):
                    l_tr = opti.variable(4); m_tr = opti.variable(4); o_tr = opti.variable(1)
                    l_tl = opti.variable(4); m_tl = opti.variable(4); o_tl = opti.variable(1)
                    vars_flat.extend([l_tr, m_tr, o_tr, l_tl, m_tl, o_tl])
                    
                    # Store logic for constraints (same as before)
                    # We retrieve them via index or list if needed, but for 'vars_flat' we just push them
        
        X.append(opti.variable(4))
        vars_flat.append(X[-1])
        
        # Combine all vars into one vector for Function Interface
        all_vars = ca.vertcat(*vars_flat)
        self.vars_shape = all_vars.shape
        
        # --- 3. Constraints & Cost Construction (Rebuilding logic) ---
        # We need to access variables from the flattened lists or reconstruct lists?
        # Reconstructing lists is safer to reuse logic.
        
        # Let's re-loop to apply constraints using the created variables
        cost = 0
        idx_counter = 0 # Helper to navigate the flat list if needed, OR just reuse the lists X/U we populated
        
        # We need the Duals in structured lists for the loops below
        Lambda_tr_list, Mu_tr_list, Omega_tr_list = [], [], []
        Lambda_tl_list, Mu_tl_list, Omega_tl_list = [], [], []
        
        # Re-populate structure from vars (since we created them in order)
        # Or simpler: Populate lists while creating (done above?) 
        # Wait, I didn't populate the lists in the loop above to keep it concise for 'vars_flat'.
        # Let's fix that block to do both.
        
        X, U = [], []
        vars_flat = []
        Lambda_tr_struct, Mu_tr_struct, Omega_tr_struct = [], [], [] # [k][obs]
        Lambda_tl_struct, Mu_tl_struct, Omega_tl_struct = [], [], []
        
        for k in range(self.N):
            xk = opti.variable(4); uk = opti.variable(2)
            X.append(xk); U.append(uk)
            vars_flat.extend([xk, uk])
            
            if k < self.N_cbf:
                l_tr_k, m_tr_k, o_tr_k = [], [], []
                l_tl_k, m_tl_k, o_tl_k = [], [], []
                
                for _ in range(self.MAX_OBS):
                    l_tr = opti.variable(4); m_tr = opti.variable(4); o_tr = opti.variable(1)
                    l_tl = opti.variable(4); m_tl = opti.variable(4); o_tl = opti.variable(1)
                    
                    vars_flat.extend([l_tr, m_tr, o_tr, l_tl, m_tl, o_tl])
                    
                    l_tr_k.append(l_tr); m_tr_k.append(m_tr); o_tr_k.append(o_tr)
                    l_tl_k.append(l_tl); m_tl_k.append(m_tl); o_tl_k.append(o_tl)
                
                Lambda_tr_struct.append(l_tr_k); Mu_tr_struct.append(m_tr_k); Omega_tr_struct.append(o_tr_k)
                Lambda_tl_struct.append(l_tl_k); Mu_tl_struct.append(m_tl_k); Omega_tl_struct.append(o_tl_k)

        X.append(opti.variable(4))
        vars_flat.append(X[-1])
        all_vars = ca.vertcat(*vars_flat)

        # --- 4. Apply Constraints ---
        for k in range(self.N):
            xk = X[k]; uk = U[k]; x_next = X[k+1]
            
            # Cost
            err = xk - p_Ref
            cost += ca.mtimes([err.T, self.Q, err]) + ca.mtimes([uk.T, self.R, uk])
            if k == 0: du = uk - p_U_prev
            else: du = uk - U[k-1]
            cost += ca.mtimes([du.T, self.R_smooth, du])
            
            # Dynamics
            x1, y1, th1, th0 = xk[0], xk[1], xk[2], xk[3]
            v0, d0 = uk[0], uk[1]; beta = th0 - th1
            V1 = v0 * ca.cos(beta) + self.M0 * (v0 * ca.tan(d0)/self.L0) * ca.sin(beta)
            x_next_model = ca.vertcat(
                x1 + V1*ca.cos(th1)*self.dt, y1 + V1*ca.sin(th1)*self.dt,
                th1 + (v0*ca.sin(beta)/self.L1 - self.M0*(v0*ca.tan(d0)/self.L0)*ca.cos(beta)/self.L1)*self.dt,
                th0 + v0*ca.tan(d0)/self.L0*self.dt
            )
            opti.subject_to(x_next == x_next_model)
            if k==0: opti.subject_to(X[0] == p_X0)
            
            # Bounds
            opti.subject_to(opti.bounded(-self.V0_MAX, uk[0], self.V0_MAX))
            opti.subject_to(opti.bounded(-self.DELTA0_MAX, uk[1], self.DELTA0_MAX))
            # opti.subject_to(opti.bounded(-self.BETA_MAX, xk[3]-xk[2], self.BETA_MAX))
            # beta should be wrapped before applying bounds
            beta_wrapped = ca.atan2(ca.sin(xk[3]-xk[2]), ca.cos(xk[3]-xk[2]))
            opti.subject_to(opti.bounded(-self.BETA_MAX, beta_wrapped, self.BETA_MAX))

            # CBF
            if k < self.N_cbf:
                x1n, y1n, th1n, th0n = x_next_model[0], x_next_model[1], x_next_model[2], x_next_model[3]
                x0n = x1n + self.L1*ca.cos(th1n) + self.M0*ca.cos(th0n)
                y0n = y1n + self.L1*ca.sin(th1n) + self.M0*ca.sin(th0n)
                
                R_tr = ca.horzcat(ca.vertcat(ca.cos(th0n), ca.sin(th0n)), ca.vertcat(-ca.sin(th0n), ca.cos(th0n)))
                R_tl = ca.horzcat(ca.vertcat(ca.cos(th1n), ca.sin(th1n)), ca.vertcat(-ca.sin(th1n), ca.cos(th1n)))
                T_tr = ca.vertcat(x0n, y0n); T_tl = ca.vertcat(x1n, y1n)
                
                for i in range(self.MAX_OBS):
                    A_obs = p_A_obs[i*4:i*4+4, :]; b_obs = p_b_obs[i*4:i*4+4]
                    
                    # Tractor
                    l, m, o = Lambda_tr_struct[k][i], Mu_tr_struct[k][i], Omega_tr_struct[k][i]
                    opti.subject_to(l>=0); opti.subject_to(m>=0); opti.subject_to(o>=0)
                    opti.subject_to(-ca.mtimes(b_tr_dm.T, m) + ca.mtimes((ca.mtimes([A_obs, T_tr])-b_obs).T, l) >= o * self.gamma**(k+1)*(p_h0_tr[i]-0.2)+0.2)
                    opti.subject_to(ca.mtimes(A_tr_dm.T, m) + ca.mtimes([R_tr, ca.mtimes(A_obs.T, l)]) == 0)
                    tmp=ca.mtimes(A_obs.T, l); opti.subject_to(ca.mtimes(tmp.T, tmp) <= 1)
                    cost += self.p_omega * (o-1)**2
                    
                    # Trailer
                    l, m, o = Lambda_tl_struct[k][i], Mu_tl_struct[k][i], Omega_tl_struct[k][i]
                    opti.subject_to(l>=0); opti.subject_to(m>=0); opti.subject_to(o>=0)
                    opti.subject_to(-ca.mtimes(b_tl_dm.T, m) + ca.mtimes((ca.mtimes([A_obs, T_tl])-b_obs).T, l) >= o * self.gamma**(k+1)*(p_h0_tl[i]-0.2)+0.2)
                    opti.subject_to(ca.mtimes(A_tl_dm.T, m) + ca.mtimes([R_tl, ca.mtimes(A_obs.T, l)]) == 0)
                    tmp=ca.mtimes(A_obs.T, l); opti.subject_to(ca.mtimes(tmp.T, tmp) <= 1)
                    cost += self.p_omega * (o-1)**2

        cost += ca.mtimes([(X[self.N] - p_Ref).T, self.Q_terminal, (X[self.N] - p_Ref)])
        opti.minimize(cost)
        
        # --- 5. Export Function ---
        opts = {"fatrop.print_level": 0, "print_time": 0, "fatrop.max_iter": 50, "structure_detection": "auto", "expand": True}
        opti.solver('fatrop', opts)
        
        # Function Signature: f(Params..., Initial_Guess_All_Vars) -> [Control_Input, Trajectory, Opt_Vars_For_Next_Warm_Start]
        # Inputs:
        input_params = [p_X0, p_Ref, p_U_prev, p_A_obs, p_b_obs, p_h0_tr, p_h0_tl, all_vars]
        # Outputs:
        # 1. Optimal Control U[0]
        # 2. Full State Trajectory (for viz)
        # 3. All Variables (to feed back as warm start)
        X_stack = ca.horzcat(*X)
        output_vars = [U[0], X_stack, all_vars]
        
        self.mpc_func = opti.to_function('mpc_step', input_params, output_vars)

    # =====================================================
    #             RUNTIME LOOP
    # =====================================================
    def state_cb(self, msg):
        d = msg.data
        self.current_state = np.array([d[0], d[1], d[3], d[2]])
        self.state_received = True

    def goal_cb(self, msg):
        self.x_goal = np.array([msg.pose.position.x, msg.pose.position.y, 0.0, 0.0]) # Simple XY goal
        self.goal_received = True

    def obs_cb(self, msg):
        new_obs = []
        idx = 0
        for obs in msg.obstacles:
            if idx >= self.MAX_OBS or obs.num_constraints != 4: continue
            new_obs.append((np.array(obs.a_vec).reshape((4, 2)), np.array(obs.b_vec)))
            idx += 1
        dummy_A, dummy_b = self.get_polytope_from_box(0.1, 0.1, 0.1)
        while len(new_obs) < self.MAX_OBS:
            new_obs.append(self.transform_polytope(dummy_A, dummy_b, 100.+len(new_obs)*2., 100., 0.))
        self.active_obstacles = new_obs

    def control_loop(self):
        if not self.state_received: return

        # 1. Pre-calc Geometry for Duals
        x1, y1, th1, th0 = self.current_state
        x0, y0 = self.get_tractor_position(x1, y1, th1, th0)
        self.publish_polytopes(x1, y1, th1, th0, x0, y0) # Viz
        
        A_tr, b_tr = self.get_polytope_from_box(self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        A_tl, b_tl = self.get_polytope_from_box(self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        A_tr_w, b_tr_w = self.transform_polytope(A_tr, b_tr, x0, y0, th0)
        A_tl_w, b_tl_w = self.transform_polytope(A_tl, b_tl, x1, y1, th1)
        
        # 2. Pack Parameters
        A_flat = np.zeros((4*self.MAX_OBS, 2)); b_flat = np.zeros(4*self.MAX_OBS)
        h0_tr = []; h0_tl = []
        
        for i in range(self.MAX_OBS):
            Ai, bi = self.active_obstacles[i]
            A_flat[i*4:i*4+4] = Ai; b_flat[i*4:i*4+4] = bi
            htr, _, _ = self.compute_separation_dual(Ai, bi, A_tr_w, b_tr_w)
            htl, _, _ = self.compute_separation_dual(Ai, bi, A_tl_w, b_tl_w)
            h0_tr.append(max(htr, 0.01)); h0_tl.append(max(htl, 0.01))

        # 3. Handle Warm Start Vector
        if self.warm_start_val is None:
            # First run: Initialize with zeros or simple guess
            # Since shape is complex, we pass zeros. CasADi handles this as "cold start"
            # To be safer, we could construct a proper vector, but zeros is usually okay for first step
            self.warm_start_val = np.zeros(self.vars_shape)

        # 4. CALL FUNCTION (No 'opti.solve' overhead!)
        try:
            # args: [X0, Ref, U_prev, A_obs, b_obs, h_tr, h_tl, Warm_Start]
            res = self.mpc_func(self.current_state, self.x_goal, self.u_prev, 
                                A_flat, b_flat, h0_tr, h0_tl, self.warm_start_val)
            
            u_opt = res[0].full().flatten()
            traj = res[1].full()
            self.warm_start_val = res[2].full() # Save for next iter
            
            self.u_prev = u_opt
            
            # 5. Publish
            self.publish_action(u_opt)
            self.publish_path(traj)
            # log info
            self.get_logger().info(f"MPC Control: v0={u_opt[0]:.3f}, delta0={np.degrees(u_opt[1]):.2f} deg")
            
        except Exception as e:
            self.get_logger().error(f"MPC Error: {e}")
            self.cmd_pub.publish(Twist())
            self.warm_start_val = np.zeros(self.vars_shape) # Reset on fail

    def publish_action(self, action):        
        lin_vel = action[0]
        steer_angle = action[1]

        ang_vel = lin_vel * tan(steer_angle) / self.L0  # omega = v * tan(delta) / L

        # Create Twist message
        msg = Twist()
        msg.linear.x = lin_vel
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = ang_vel

        # Publish
        self.cmd_pub.publish(msg)

    def publish_path(self, traj):
        msg = Path(); msg.header.frame_id="map"; msg.header.stamp=self.get_clock().now().to_msg()
        for i in range(traj.shape[1]):
            p = PoseStamped(); p.header=msg.header; p.pose.position.x=traj[0,i]; p.pose.position.y=traj[1,i]
            msg.poses.append(p)
        self.traj_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args); node = DCBFNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()