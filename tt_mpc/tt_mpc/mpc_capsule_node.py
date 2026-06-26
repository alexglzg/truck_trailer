#!/usr/bin/env python3
"""
mpc_capsule_node.py  --  ROS2 (Foxy) wrapper around the capsule-DCBF MPC.

Thin glue only: all OCP logic lives in capsule_mpc.CapsuleMPC (ROS-free, tested
offline). Each control tick this node
  1. reads the latest [x1,y1,th0,th1] state,
  2. selects the closest n_obstacles by CURRENT distance and rolls out the rest
     as far-parked dummies,
  3. solves the OCP,
  4. publishes /cmd_vel (linear.x=V0, angular.z=delta0), the predicted trailer /
     truck / obstacle paths, and CasADi's t_wall_total.
On an infeasible solve it publishes a safe stop (V0=0, delta0=0). There is no
authority / heartbeat logic here: the node just re-plans every tick.
"""
import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Float64
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

from tt_interfaces.msg import CapsuleObstacleArray

from .capsule_mpc import CapsuleMPC
from . import capsule_geometry as geom


class MpcCapsuleNode(Node):
    def __init__(self):
        super().__init__('mpc_capsule_node')
        self._declare_params()
        cfg = self._build_cfg()
        self.cfg = cfg

        self.get_logger().info(
            f"Building OCP: N={cfg['N']} dt={cfg['dt']} n_obstacles={cfg['n_obstacles']} "
            f"walls={cfg['walls_enabled']}")
        self.mpc = CapsuleMPC(cfg)
        self.get_logger().info(
            f"OCP built (solver={self.mpc.solver_name}, cbf_rows={self.mpc.cbf_rows}).")

        # runtime state
        self.state = None                     # np.array([x1,y1,th0,th1])
        self.goal = None                      # np.array([gx,gy])
        self.obstacles = []                   # list of dicts from the latest array msg
        self.u_prev = np.zeros(2)

        # topics
        t = self._topics
        self.create_subscription(Float64MultiArray, t['state'], self._on_state, 10)
        self.create_subscription(CapsuleObstacleArray, t['obstacles'], self._on_obstacles, 10)
        self.create_subscription(PoseStamped, t['goal'], self._on_goal, 10)

        self.pub_cmd = self.create_publisher(Twist, t['cmd_vel'], 10)
        self.pub_solve_time = self.create_publisher(Float64, '/mpc/solve_time', 10)
        self.pub_path_trailer = self.create_publisher(Path, '/mpc/pred_trailer', 1)
        self.pub_path_truck = self.create_publisher(Path, '/mpc/pred_truck', 1)
        self.pub_path_obstacle = self.create_publisher(Path, '/mpc/pred_obstacle', 1)

        self.create_timer(1.0 / self._rate_hz, self._control_step)
        self.get_logger().info("mpc_capsule_node ready.")

    # ------------------------------------------------------------------
    # parameters
    # ------------------------------------------------------------------
    def _declare_params(self):
        d = self.declare_parameter
        d('N', 20)
        d('dt', 0.1)
        d('n_obstacles', 1)
        d('rate_hz', 10.0)
        # kinematics / footprint / walls (shared /** block)
        d('kinematics.L0', 0.42); d('kinematics.M0', -0.02); d('kinematics.L1', 0.537)
        d('footprint.tractor.length', 0.55); d('footprint.tractor.width', 0.35); d('footprint.tractor.offset', 0.215)
        d('footprint.trailer.length', 0.55); d('footprint.trailer.width', 0.35); d('footprint.trailer.offset', 0.22)
        d('virtual_walls.enabled', True)
        d('virtual_walls.x_min', -0.9); d('virtual_walls.x_max', 2.65)
        d('virtual_walls.y_min', -1.25); d('virtual_walls.y_max', 4.9)
        # limits
        d('limits.V0_max', 0.5); d('limits.delta0_max', 1.2217); d('limits.beta_max', 1.3963)
        # cbf
        d('cbf.gamma_obs', 0.5); d('cbf.gamma_wall', 0.9); d('cbf.gamma_jack', 0.9)
        d('cbf.safe_marg', 0.05); d('cbf.softabs_eps', 0.02)
        # cost
        for k, v in (('Q_x', 1.0), ('Q_y', 1.0), ('Q_th0', 0.0), ('Q_th1', 0.0),
                     ('R_V0', 0.5), ('R_delta0', 0.01), ('R_smooth_V0', 0.2),
                     ('R_smooth_delta0', 0.3), ('P_term_x', 2.0), ('P_term_y', 2.0)):
            d(f'cost.{k}', v)
        # solver
        d('solver.max_iter', 300); d('solver.tol', 1.0e-5)
        d('solver.acceptable_tol', 1.0e-3); d('solver.acceptable_iter', 5)
        d('solver.constr_viol_tol', 1.0e-6)
        # topics
        d('topics.state', '/truck_trailer/state')
        d('topics.obstacles', '/capsule_obstacles')
        d('topics.goal', '/goal_pose')
        d('topics.cmd_vel', '/cmd_vel')

    def _g(self, name):
        return self.get_parameter(name).value

    def _build_cfg(self):
        self._rate_hz = float(self._g('rate_hz'))
        self._topics = {k: self._g(f'topics.{k}') for k in ('state', 'obstacles', 'goal', 'cmd_vel')}
        n_obs = int(self._g('n_obstacles'))
        return dict(
            N=int(self._g('N')), dt=float(self._g('dt')), n_obstacles=n_obs,
            L0=self._g('kinematics.L0'), M0=self._g('kinematics.M0'), L1=self._g('kinematics.L1'),
            tractor_len=self._g('footprint.tractor.length'),
            tractor_wid=self._g('footprint.tractor.width'),
            tractor_off=self._g('footprint.tractor.offset'),
            trailer_len=self._g('footprint.trailer.length'),
            trailer_wid=self._g('footprint.trailer.width'),
            trailer_off=self._g('footprint.trailer.offset'),
            V0_max=self._g('limits.V0_max'), delta0_max=self._g('limits.delta0_max'),
            beta_max=self._g('limits.beta_max'),
            gamma_obs=self._g('cbf.gamma_obs'), gamma_wall=self._g('cbf.gamma_wall'),
            gamma_jack=self._g('cbf.gamma_jack'),
            safe_marg=self._g('cbf.safe_marg'), softabs_eps=self._g('cbf.softabs_eps'),
            Q_x=self._g('cost.Q_x'), Q_y=self._g('cost.Q_y'),
            Q_th0=self._g('cost.Q_th0'), Q_th1=self._g('cost.Q_th1'),
            R_V0=self._g('cost.R_V0'), R_delta0=self._g('cost.R_delta0'),
            R_smooth_V0=self._g('cost.R_smooth_V0'), R_smooth_delta0=self._g('cost.R_smooth_delta0'),
            P_term_x=self._g('cost.P_term_x'), P_term_y=self._g('cost.P_term_y'),
            walls_enabled=bool(self._g('virtual_walls.enabled')),
            x_min=self._g('virtual_walls.x_min'), x_max=self._g('virtual_walls.x_max'),
            y_min=self._g('virtual_walls.y_min'), y_max=self._g('virtual_walls.y_max'),
            max_iter=int(self._g('solver.max_iter')), tol=float(self._g('solver.tol')),
            acceptable_tol=float(self._g('solver.acceptable_tol')),
            acceptable_iter=int(self._g('solver.acceptable_iter')),
            constr_viol_tol=float(self._g('solver.constr_viol_tol')),
        )

    # ------------------------------------------------------------------
    # callbacks
    # ------------------------------------------------------------------
    def _on_state(self, msg):
        if len(msg.data) >= 4:
            self.state = np.array(msg.data[:4], dtype=float)

    def _on_goal(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y], dtype=float)
        self.get_logger().info(f"new goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def _on_obstacles(self, msg):
        obs = []
        for o in msg.obstacles:
            obs.append(dict(pos=np.array([o.x, o.y]), vel=np.array([o.vx, o.vy]),
                            psi=o.psi, length=o.length, radius=o.radius))
        self.obstacles = obs

    # ------------------------------------------------------------------
    # obstacle selection / padding
    # ------------------------------------------------------------------
    def _select_obstacles(self, state):
        """Return exactly n_obstacles dicts: closest by true boundary distance
        (seg-seg minus both radii, min over tractor and trailer), padded with
        far-parked dummies if fewer were seen."""
        n = self.cfg['n_obstacles']
        pT, qT, _, _ = geom.tractor_capsule(state, self.cfg['L0'], self.cfg['M0'], self.cfg['L1'],
                                             self.cfg['tractor_off'], self.cfg['tractor_len'])
        pTr, qTr, _, _ = geom.trailer_capsule(state, self.cfg['trailer_off'], self.cfg['trailer_len'])
        R_T, R_Tr = self.cfg['tractor_wid'] / 2.0, self.cfg['trailer_wid'] / 2.0
        scored = []
        for o in self.obstacles:
            pO, qO = geom.obstacle_capsule(o['pos'][0], o['pos'][1], o['psi'], o['length'])
            hT  = np.sqrt(geom.seg_seg_sq_dist(pT,  qT,  pO, qO)) - R_T  - o['radius']
            hTr = np.sqrt(geom.seg_seg_sq_dist(pTr, qTr, pO, qO)) - R_Tr - o['radius']
            scored.append((min(hT, hTr), o))
        scored.sort(key=lambda s: s[0])
        chosen = [o for _, o in scored[:n]]
        while len(chosen) < n:
            chosen.append(dict(pos=np.array([1.0e3, 1.0e3]), vel=np.zeros(2),
                               psi=0.0, length=0.55, radius=0.0))
        return chosen

    # ------------------------------------------------------------------
    # control loop
    # ------------------------------------------------------------------
    def _control_step(self):
        if self.state is None:
            return
        goal = self.goal if self.goal is not None else self.state[:2].copy()
        obstacles = self._select_obstacles(self.state)

        res = self.mpc.solve(self.state, goal, self.u_prev, obstacles)
        u = res['u']

        cmd = Twist()
        cmd.linear.x = float(u[0])
        cmd.angular.z = float(u[1])
        self.pub_cmd.publish(cmd)
        self.u_prev = u.copy()

        st = Float64(); st.data = float(res['t_wall_total']) if np.isfinite(res['t_wall_total']) else -1.0
        self.pub_solve_time.publish(st)

        if not res['ok']:
            self.get_logger().warn("solve infeasible -> safe stop (V0=0, delta0=0)",
                                   throttle_duration_sec=1.0)

        self._publish_paths(res['X'], obstacles[0])

    # ------------------------------------------------------------------
    # predicted-trajectory publishing (rviz native Path display)
    # ------------------------------------------------------------------
    def _publish_paths(self, X, obs0):
        stamp = self.get_clock().now().to_msg()
        N = self.cfg['N']

        def make_path(xy):
            p = Path(); p.header.stamp = stamp; p.header.frame_id = 'map'
            for x, y in xy:
                ps = PoseStamped(); ps.header = p.header
                ps.pose.position.x = float(x); ps.pose.position.y = float(y)
                ps.pose.orientation.w = 1.0
                p.poses.append(ps)
            return p

        # trailer: X[0:2]
        self.pub_path_trailer.publish(make_path(X[:2, :].T))

        # truck: reconstruct rear-axle (x0,y0) per stage from hitch geometry
        L1, M0 = self.cfg['L1'], self.cfg['M0']
        x1, y1, th0, th1 = X[0], X[1], X[2], X[3]
        x0 = x1 + L1 * np.cos(th1) + M0 * np.cos(th0)
        y0 = y1 + L1 * np.sin(th1) + M0 * np.sin(th0)
        self.pub_path_truck.publish(make_path(np.column_stack([x0, y0])))

        # obstacle 0: constant-velocity rollout
        obs_xy = np.array([obs0['pos'] + k * self.cfg['dt'] * obs0['vel'] for k in range(N + 1)])
        self.pub_path_obstacle.publish(make_path(obs_xy))


def main(args=None):
    rclpy.init(args=args)
    node = MpcCapsuleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()