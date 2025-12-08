#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

import casadi as ca
import numpy as np
from numpy import sin, cos, tan
from scipy.optimize import linprog 

class DCBFNode(Node):
    def __init__(self):
        super().__init__('dcbf_mpc_node')
        
        # =====================================================
        #             MODEL PARAMETERS (From Prototype)
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
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64MultiArray, '/truck_trailer/state', self.state_cb, 10)

        self.current_state = np.zeros(4) 
        self.state_received = False
        self.u_prev = np.zeros(2)

        # Hardcoded Target & Obstacle (From Prototype)
        self.x_goal = np.array([2.0, 2.0, 0.0, 0.0])
        
        obs_center = np.array([1.0, 1.0])
        obs_size = (0.6, 0.4)
        A_obs_local, b_obs_local = self.get_polytope_from_box(obs_size[1]/2, obs_size[0]/2, obs_size[0]/2)
        self.A_obs_world, self.b_obs_world = self.transform_polytope(A_obs_local, b_obs_local, obs_center[0], obs_center[1], 0.0)

        self.get_logger().info("Building MPC (Fatrop)...")
        self.setup_mpc()
        self.get_logger().info("MPC Ready.")

        self.timer = self.create_timer(0.1, self.control_loop)

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

    def setup_mpc(self):
        opti = ca.Opti()
        
        # 1. CONSTANT POLYTOPES (NUMPY)
        # Note: We keep these as numpy arrays here to match prototype logic logic
        A_tractor_np, b_tractor_np = self.get_polytope_from_box(self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        A_trailer_np, b_trailer_np = self.get_polytope_from_box(self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        
        # Convert to CasADi DM for symbolic operations later
        # Transpose .T on a DM(4,1) results in (1,4), so we don't need reshape()
        b_tractor_dm = ca.DM(b_tractor_np)
        b_trailer_dm = ca.DM(b_trailer_np)
        A_tractor_dm = ca.DM(A_tractor_np)
        A_trailer_dm = ca.DM(A_trailer_np)
        
        # 2. CREATE VARIABLES (PHASE 1)
        X, U = [], []
        Lambda_tractor, Mu_tractor, Omega_tractor = [], [], []
        Lambda_trailer, Mu_trailer, Omega_trailer = [], [], []
        
        for k in range(self.N):
            X.append(opti.variable(4))
            U.append(opti.variable(2))
            if k < self.N_cbf:
                Lambda_tractor.append(opti.variable(4))
                Mu_tractor.append(opti.variable(4))
                Omega_tractor.append(opti.variable(1))
                Lambda_trailer.append(opti.variable(4))
                Mu_trailer.append(opti.variable(4))
                Omega_trailer.append(opti.variable(1))
        
        X.append(opti.variable(4)) # Final State
        
        # 3. PARAMETERS
        self.par_X0 = opti.parameter(4)
        self.par_X_ref = opti.parameter(4)
        self.par_U_prev = opti.parameter(2)
        self.par_A_obs = opti.parameter(4, 2)
        self.par_b_obs = opti.parameter(4)
        self.par_h0_tractor = opti.parameter(1)
        self.par_h0_trailer = opti.parameter(1)

        # 4. CONSTRAINTS LOOP (PHASE 2) - STRICT ORDERING
        cost = 0
        
        for k in range(self.N):
            xk = X[k]
            uk = U[k]
            x_next = X[k+1]
            
            # --- Cost ---
            err = xk - self.par_X_ref
            cost += ca.mtimes([err.T, self.Q, err])
            cost += ca.mtimes([uk.T, self.R, uk])
            
            if k == 0:
                du = uk - self.par_U_prev
            else:
                du = uk - U[k-1]
            cost += ca.mtimes([du.T, self.R_smooth, du])
            
            # --- Dynamics ---
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
            
            # --- Initial Condition (MUST BE INSIDE LOOP AT k=0) ---
            if k == 0:
                opti.subject_to(X[0] == self.par_X0)
            
            # --- Input Limits ---
            opti.subject_to(opti.bounded(-self.V0_MAX, uk[0], self.V0_MAX))
            opti.subject_to(opti.bounded(-self.DELTA0_MAX, uk[1], self.DELTA0_MAX))
            opti.subject_to(opti.bounded(-self.BETA_MAX, xk[3] - xk[2], self.BETA_MAX))

            # --- CBF Constraints ---
            if k < self.N_cbf:
                x1_n, y1_n, th1_n, th0_n = x_next_model[0], x_next_model[1], x_next_model[2], x_next_model[3]
                
                hx_n = x1_n + self.L1 * ca.cos(th1_n)
                hy_n = y1_n + self.L1 * ca.sin(th1_n)
                x0_n = hx_n + self.M0 * ca.cos(th0_n)
                y0_n = hy_n + self.M0 * ca.sin(th0_n)
                
                R_trac = ca.horzcat(ca.vertcat(ca.cos(th0_n), ca.sin(th0_n)), ca.vertcat(-ca.sin(th0_n), ca.cos(th0_n)))
                T_trac = ca.vertcat(x0_n, y0_n)
                R_trail = ca.horzcat(ca.vertcat(ca.cos(th1_n), ca.sin(th1_n)), ca.vertcat(-ca.sin(th1_n), ca.cos(th1_n)))
                T_trail = ca.vertcat(x1_n, y1_n)
                
                # Tractor
                opti.subject_to(Lambda_tractor[k] >= 0)
                opti.subject_to(Mu_tractor[k] >= 0)
                opti.subject_to(Omega_tractor[k] >= 0)
                
                # Fix: Use DM.T (transpose) instead of reshape
                term1 = -ca.mtimes(b_tractor_dm.T, Mu_tractor[k]) 
                term2 = ca.mtimes((ca.mtimes([self.par_A_obs, T_trac]) - self.par_b_obs).T, Lambda_tractor[k])
                opti.subject_to(term1 + term2 >= Omega_tractor[k] * self.gamma**(k+1) * (self.par_h0_tractor - self.margin_dist) + self.margin_dist)
                
                opti.subject_to(
                    ca.mtimes(A_tractor_dm.T, Mu_tractor[k]) + 
                    ca.mtimes([R_trac.T, ca.mtimes([self.par_A_obs.T, Lambda_tractor[k]])]) == 0
                )
                
                tmp_tr = ca.mtimes(self.par_A_obs.T, Lambda_tractor[k])
                opti.subject_to(ca.mtimes(tmp_tr.T, tmp_tr) <= 1)
                cost += self.p_omega * (Omega_tractor[k] - 1)**2

                # Trailer
                opti.subject_to(Lambda_trailer[k] >= 0)
                opti.subject_to(Mu_trailer[k] >= 0)
                opti.subject_to(Omega_trailer[k] >= 0)
                
                term1_tl = -ca.mtimes(b_trailer_dm.T, Mu_trailer[k])
                term2_tl = ca.mtimes((ca.mtimes([self.par_A_obs, T_trail]) - self.par_b_obs).T, Lambda_trailer[k])
                opti.subject_to(term1_tl + term2_tl >= Omega_trailer[k] * self.gamma**(k+1) * (self.par_h0_trailer - self.margin_dist) + self.margin_dist)
                
                opti.subject_to(
                    ca.mtimes(A_trailer_dm.T, Mu_trailer[k]) + 
                    ca.mtimes([R_trail.T, ca.mtimes([self.par_A_obs.T, Lambda_trailer[k]])]) == 0
                )
                
                tmp_tl = ca.mtimes(self.par_A_obs.T, Lambda_trailer[k])
                opti.subject_to(ca.mtimes(tmp_tl.T, tmp_tl) <= 1)
                cost += self.p_omega * (Omega_trailer[k] - 1)**2

        # Terminal Cost
        cost += ca.mtimes([(X[self.N] - self.par_X_ref).T, self.Q_terminal, (X[self.N] - self.par_X_ref)])
        opti.minimize(cost)
        
        # Solver
        opts = {
            "fatrop.print_level": 0, "print_time": 0,
            "fatrop.max_iter": 100, "fatrop.tol": 1e-4,
            "structure_detection": "auto", "expand": True
        }
        opti.solver('fatrop', opts)
        
        self.opti = opti
        self.var_X = X
        self.var_U = U
        self.var_Lambda_tr = Lambda_tractor
        self.var_Mu_tr = Mu_tractor
        self.var_Lambda_tl = Lambda_trailer
        self.var_Mu_tl = Mu_trailer

    def state_cb(self, msg):
        d = msg.data
        self.current_state = np.array([d[0], d[1], d[3], d[2]])
        self.state_received = True

    def control_loop(self):
        if not self.state_received:
            self.get_logger().info("Waiting for state...", throttle_duration_sec=2.0)
            return

        x1, y1, th1, th0 = self.current_state
        x0, y0 = self.get_tractor_position(x1, y1, th1, th0)
        
        A_tr, b_tr = self.get_polytope_from_box(self.TRACTOR_W, self.TRACTOR_FRONT, self.TRACTOR_BACK)
        A_tl, b_tl = self.get_polytope_from_box(self.TRAILER_W, self.TRAILER_FRONT, self.TRAILER_BACK)
        
        A_tr_w, b_tr_w = self.transform_polytope(A_tr, b_tr, x0, y0, th0)
        A_tl_w, b_tl_w = self.transform_polytope(A_tl, b_tl, x1, y1, th1)
        
        h_tr, l_tr, m_tr = self.compute_separation_dual(self.A_obs_world, self.b_obs_world, A_tr_w, b_tr_w)
        h_tl, l_tl, m_tl = self.compute_separation_dual(self.A_obs_world, self.b_obs_world, A_tl_w, b_tl_w)
        
        self.opti.set_value(self.par_X0, self.current_state)
        self.opti.set_value(self.par_X_ref, self.x_goal)
        self.opti.set_value(self.par_U_prev, self.u_prev)
        self.opti.set_value(self.par_A_obs, self.A_obs_world)
        self.opti.set_value(self.par_b_obs, self.b_obs_world)
        self.opti.set_value(self.par_h0_tractor, max(h_tr, 0.01))
        self.opti.set_value(self.par_h0_trailer, max(h_tl, 0.01))
        
        for k in range(self.N + 1):
            alpha = k / self.N
            interp = (1 - alpha) * self.current_state + alpha * self.x_goal
            self.opti.set_initial(self.var_X[k], interp)
        
        for k in range(self.N_cbf):
            self.opti.set_initial(self.var_Lambda_tr[k], l_tr)
            self.opti.set_initial(self.var_Mu_tr[k], m_tr)
            self.opti.set_initial(self.var_Lambda_tl[k], l_tl)
            self.opti.set_initial(self.var_Mu_tl[k], m_tl)

        try:
            sol = self.opti.solve()
            u_opt = sol.value(self.var_U[0])
            self.u_prev = u_opt
            
            msg = Twist()
            msg.linear.x = float(u_opt[0])
            msg.angular.z = float(u_opt[1])
            self.cmd_pub.publish(msg)
            
        except RuntimeError:
            self.get_logger().error("Solver failed!")
            self.cmd_pub.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = DCBFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()