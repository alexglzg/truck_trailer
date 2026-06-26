"""
capsule_mpc.py  --  ROS-free core of the capsule-DCBF tractor-trailer MPC.

Velocity-level kinematic model, first-order discrete CBFs (m = 1), NO slacks.
Three safety families, all imposed as HARD inequalities:
  * obstacle  : capsule(body)-vs-capsule(obstacle) closed-form distance
  * walls     : each body rectangle kept inside an axis-aligned box (support fn)
  * jackknife : articulation angle bounded, |beta| <= beta_max

State  x = [x1, y1, theta0, theta1]   (trailer position, tractor & trailer heading)
Input  u = [V0, delta0]               (tractor speed, steering angle)

Because the input is at the velocity level, h is relative degree 1 and a plain
DCBF  h(x_{k+1}) >= (1 - gamma) h(x_k)  is the matched construction -- no psi_2
cascade. With no slacks the OCP can be infeasible; the caller must treat a failed
solve as a safe-stop event.

This module has no ROS dependency so it can be unit-tested and reused for
offline benchmarking. The ROS node (mpc_capsule_node.py) is a thin wrapper.
"""
import numpy as np
import casadi as ca


# ----------------------------------------------------------------------
# Dynamics  (explicit Euler -- matches the simulator/plant integration)
# ----------------------------------------------------------------------
def tt_step_sym(x, u, L0, M0, L1, dt):
    x1, y1, th0, th1 = x[0], x[1], x[2], x[3]
    V0, d0 = u[0], u[1]
    beta = th0 - th1
    om0 = V0 * ca.tan(d0) / L0
    V1 = V0 * ca.cos(beta) + M0 * om0 * ca.sin(beta)
    dth1 = (V0 * ca.sin(beta) - M0 * om0 * ca.cos(beta)) / L1
    return ca.vertcat(
        x1 + V1 * ca.cos(th1) * dt,
        y1 + V1 * ca.sin(th1) * dt,
        th0 + om0 * dt,
        th1 + dth1 * dt,
    )


def tt_step_np(x, u, L0, M0, L1, dt):
    x1, y1, th0, th1 = x
    V0, d0 = u
    beta = th0 - th1
    om0 = V0 * np.tan(d0) / L0
    V1 = V0 * np.cos(beta) + M0 * om0 * np.sin(beta)
    dth1 = (V0 * np.sin(beta) - M0 * om0 * np.cos(beta)) / L1
    return np.array([
        x1 + V1 * np.cos(th1) * dt,
        y1 + V1 * np.sin(th1) * dt,
        th0 + om0 * dt,
        th1 + dth1 * dt,
    ])


# ----------------------------------------------------------------------
# Capsule endpoints  (symbolic).  Each returns (p, q, center).
# Capsule is centered on the body footprint, segment along the body axis.
# ----------------------------------------------------------------------
def tractor_endpoints_sym(x, L0, M0, L1, off, length):
    x1, y1, th0, th1 = x[0], x[1], x[2], x[3]
    x0 = x1 + L1 * ca.cos(th1) + M0 * ca.cos(th0)
    y0 = y1 + L1 * ca.sin(th1) + M0 * ca.sin(th0)
    cp, sp = ca.cos(th0), ca.sin(th0)
    cx = x0 + off * cp
    cy = y0 + off * sp
    p = ca.vertcat(cx - 0.5 * length * cp, cy - 0.5 * length * sp)
    q = ca.vertcat(cx + 0.5 * length * cp, cy + 0.5 * length * sp)
    return p, q, ca.vertcat(cx, cy)


def trailer_endpoints_sym(x, off, length):
    x1, y1, th1 = x[0], x[1], x[3]
    cp, sp = ca.cos(th1), ca.sin(th1)
    cx = x1 + off * cp
    cy = y1 + off * sp
    p = ca.vertcat(cx - 0.5 * length * cp, cy - 0.5 * length * sp)
    q = ca.vertcat(cx + 0.5 * length * cp, cy + 0.5 * length * sp)
    return p, q, ca.vertcat(cx, cy)


def obstacle_endpoints_np(center, psi, length):
    cp, sp = np.cos(psi), np.sin(psi)
    p = np.array([center[0] - 0.5 * length * cp, center[1] - 0.5 * length * sp])
    q = np.array([center[0] + 0.5 * length * cp, center[1] + 0.5 * length * sp])
    return p, q


# ----------------------------------------------------------------------
# Smooth segment-segment squared distance (closed form, lse-free)
# ----------------------------------------------------------------------
def seg_seg_sq_dist_sym(p1, q1, p2, q2):
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = ca.dot(d1, d1)
    e = ca.dot(d2, d2)
    f = ca.dot(d2, r)
    c = ca.dot(d1, r)
    b = ca.dot(d1, d2)
    denom = a * e - b * b + 1e-9
    s_star = (b * f - c * e) / denom
    s_clip = ca.fmin(ca.fmax(s_star, 0.0), 1.0)
    t_star = (b * s_clip + f) / (e + 1e-9)
    t_clip = ca.fmin(ca.fmax(t_star, 0.0), 1.0)
    s_final = ca.fmin(ca.fmax((b * t_clip - c) / (a + 1e-9), 0.0), 1.0)
    diff = (p1 + s_final * d1) - (p2 + t_clip * d2)
    return ca.dot(diff, diff)


# ----------------------------------------------------------------------
# Smooth absolute value for the wall support function.
# softabs(z) = sqrt(z^2 + eps^2) >= |z|, so the projected half-extent is
# over-estimated -> body kept slightly FARTHER inside the walls (safe side).
# Deliberately NOT casadi lse(): one sqrt, no exp/log, cheap to evaluate.
# ----------------------------------------------------------------------
def _softabs(z, eps):
    return ca.sqrt(z * z + eps * eps)


def _support_x(th, L, W, eps):
    return 0.5 * L * _softabs(ca.cos(th), eps) + 0.5 * W * _softabs(ca.sin(th), eps)


def _support_y(th, L, W, eps):
    return 0.5 * L * _softabs(ca.sin(th), eps) + 0.5 * W * _softabs(ca.cos(th), eps)


# ----------------------------------------------------------------------
# MPC
# ----------------------------------------------------------------------
class CapsuleMPC:
    """Build the OCP once; call solve() every control tick."""

    REQUIRED = (
        'N', 'dt', 'n_obstacles',
        'L0', 'M0', 'L1',
        'tractor_len', 'tractor_wid', 'tractor_off',
        'trailer_len', 'trailer_wid', 'trailer_off',
        'V0_max', 'delta0_max', 'beta_max',
        'gamma_obs', 'gamma_wall', 'gamma_jack',
        'safe_marg', 'softabs_eps',
        'Q_x', 'Q_y', 'Q_th0', 'Q_th1',
        'R_V0', 'R_delta0', 'R_smooth_V0', 'R_smooth_delta0',
        'P_term_x', 'P_term_y',
        'walls_enabled', 'x_min', 'x_max', 'y_min', 'y_max',
        'max_iter', 'tol', 'acceptable_tol', 'acceptable_iter', 'constr_viol_tol',
    )

    def __init__(self, cfg):
        for k in self.REQUIRED:
            if k not in cfg:
                raise KeyError(f"CapsuleMPC cfg missing '{k}'")

        self.cfg = cfg
        N = self.N = int(cfg['N'])
        dt = self.dt = float(cfg['dt'])
        n_obs = self.n_obs = int(cfg['n_obstacles'])
        L0, M0, L1 = cfg['L0'], cfg['M0'], cfg['L1']

        R_T = cfg['tractor_wid'] / 2.0
        R_Tr = cfg['trailer_wid'] / 2.0
        marg = cfg['safe_marg']
        eps = cfg['softabs_eps']
        gob, gwl, gjk = cfg['gamma_obs'], cfg['gamma_wall'], cfg['gamma_jack']
        beta_max_sq = cfg['beta_max'] ** 2

        opti = ca.Opti()

        # --- parameters ---
        p_X0 = opti.parameter(4)
        p_goal = opti.parameter(2)
        p_Uprev = opti.parameter(2)
        # one (2, N+1) endpoint-trajectory parameter PER obstacle (fatrop staging)
        p_pB = [opti.parameter(2, N + 1) for _ in range(n_obs)]
        p_qB = [opti.parameter(2, N + 1) for _ in range(n_obs)]
        p_Robs = opti.parameter(n_obs)   # obstacle radii, set per tick

        # --- decision variables, interleaved X[0],U[0],X[1],U[1],...,X[N] ---
        X, U = [], []
        for _ in range(N):
            X.append(opti.variable(4))
            U.append(opti.variable(2))
        X.append(opti.variable(4))

        cost = 0
        n_cbf_rows = 0
        for k in range(N):
            xk, uk = X[k], U[k]
            xn = tt_step_sym(xk, uk, L0, M0, L1, dt)

            if k == 0:
                opti.subject_to(xk == p_X0)
            opti.subject_to(X[k + 1] == xn)

            # input box bounds (cannot themselves cause infeasibility)
            opti.subject_to(opti.bounded(-cfg['V0_max'], uk[0], cfg['V0_max']))
            opti.subject_to(opti.bounded(-cfg['delta0_max'], uk[1], cfg['delta0_max']))

            # ---- cost ----
            ex = xk[0] - p_goal[0]
            ey = xk[1] - p_goal[1]
            cost += cfg['Q_x'] * ex * ex + cfg['Q_y'] * ey * ey
            cost += cfg['Q_th0'] * xk[2] ** 2 + cfg['Q_th1'] * xk[3] ** 2
            cost += cfg['R_V0'] * uk[0] ** 2 + cfg['R_delta0'] * uk[1] ** 2
            u_ref = p_Uprev if k == 0 else U[k - 1]
            cost += cfg['R_smooth_V0'] * (uk[0] - u_ref[0]) ** 2
            cost += cfg['R_smooth_delta0'] * (uk[1] - u_ref[1]) ** 2

            # ---- body capsules at k and k+1 ----
            pT_k, qT_k, cT_k = tractor_endpoints_sym(xk, L0, M0, L1,
                                                     cfg['tractor_off'], cfg['tractor_len'])
            pTr_k, qTr_k, cTr_k = trailer_endpoints_sym(xk, cfg['trailer_off'], cfg['trailer_len'])
            pT_n, qT_n, cT_n = tractor_endpoints_sym(xn, L0, M0, L1,
                                                     cfg['tractor_off'], cfg['tractor_len'])
            pTr_n, qTr_n, cTr_n = trailer_endpoints_sym(xn, cfg['trailer_off'], cfg['trailer_len'])

            # ---- obstacle DCBF (one row per body per obstacle per stage) ----
            for i in range(n_obs):
                pBk, qBk = p_pB[i][:, k], p_qB[i][:, k]
                pBn, qBn = p_pB[i][:, k + 1], p_qB[i][:, k + 1]

                hT_k  = ca.sqrt(seg_seg_sq_dist_sym(pT_k,  qT_k,  pBk, qBk) + 1e-9) - (R_T  + p_Robs[i] + marg)
                hT_n  = ca.sqrt(seg_seg_sq_dist_sym(pT_n,  qT_n,  pBn, qBn) + 1e-9) - (R_T  + p_Robs[i] + marg)
                opti.subject_to(hT_n >= (1.0 - gob) * hT_k)

                hTr_k = ca.sqrt(seg_seg_sq_dist_sym(pTr_k, qTr_k, pBk, qBk) + 1e-9) - (R_Tr + p_Robs[i] + marg)
                hTr_n = ca.sqrt(seg_seg_sq_dist_sym(pTr_n, qTr_n, pBn, qBn) + 1e-9) - (R_Tr + p_Robs[i] + marg)
                opti.subject_to(hTr_n >= (1.0 - gob) * hTr_k)
                n_cbf_rows += 2

            # ---- wall DCBF (each body rectangle inside the box) ----
            if cfg['walls_enabled']:
                for (cx_k, cy_k, th_k, cx_n, cy_n, th_n, Lb, Wb) in (
                    (cT_k[0], cT_k[1], xk[2], cT_n[0], cT_n[1], xn[2],
                     cfg['tractor_len'], cfg['tractor_wid']),
                    (cTr_k[0], cTr_k[1], xk[3], cTr_n[0], cTr_n[1], xn[3],
                     cfg['trailer_len'], cfg['trailer_wid']),
                ):
                    rx_k, ry_k = _support_x(th_k, Lb, Wb, eps), _support_y(th_k, Lb, Wb, eps)
                    rx_n, ry_n = _support_x(th_n, Lb, Wb, eps), _support_y(th_n, Lb, Wb, eps)
                    walls = (
                        (cx_k - rx_k - cfg['x_min'] - marg, cx_n - rx_n - cfg['x_min'] - marg),
                        (cfg['x_max'] - cx_k - rx_k - marg, cfg['x_max'] - cx_n - rx_n - marg),
                        (cy_k - ry_k - cfg['y_min'] - marg, cy_n - ry_n - cfg['y_min'] - marg),
                        (cfg['y_max'] - cy_k - ry_k - marg, cfg['y_max'] - cy_n - ry_n - marg),
                    )
                    for h_k, h_n in walls:
                        opti.subject_to(h_n >= (1.0 - gwl) * h_k)
                        n_cbf_rows += 1

            # ---- jackknife DCBF (beta wrapped to avoid +-pi differencing) ----
            bk = ca.atan2(ca.sin(xk[2] - xk[3]), ca.cos(xk[2] - xk[3]))
            bn = ca.atan2(ca.sin(xn[2] - xn[3]), ca.cos(xn[2] - xn[3]))
            hJ_k = beta_max_sq - bk * bk
            hJ_n = beta_max_sq - bn * bn
            opti.subject_to(hJ_n >= (1.0 - gjk) * hJ_k)
            n_cbf_rows += 1

        # terminal position cost
        cost += cfg['P_term_x'] * (X[N][0] - p_goal[0]) ** 2
        cost += cfg['P_term_y'] * (X[N][1] - p_goal[1]) ** 2
        opti.minimize(cost)

        self._setup_solver(opti, cfg)

        # stash handles
        self.opti = opti
        self.X, self.U = X, U
        self.p_X0, self.p_goal, self.p_Uprev = p_X0, p_goal, p_Uprev
        self.p_pB, self.p_qB = p_pB, p_qB
        self.p_Robs = p_Robs
        self.cbf_rows = n_cbf_rows
        self._warm_X = None
        self._warm_U = None

    def _setup_solver(self, opti, cfg):
        # print_time=True is REQUIRED for fatrop to populate t_wall_total in
        # sol.stats(); without it the stat reads None. Cost is one timing line
        # to stdout per solve.
        #
        # acceptable_tol/acceptable_iter matter A LOT here: with NO slacks the
        # KKT Hessian loses the diagonal curvature the slack penalties used to
        # add, so the dual (stationarity) residual stalls ~1e-3 on top of an
        # already primal-feasible point and never reaches tol=1e-5 before
        # max_iter -> spurious "failures". Accepting a feasible near-stationary
        # point (as ipopt does by default) fixes this WITHOUT relaxing any
        # constraint: constr_viol_tol stays tight, so the DCBFs -- hence safety
        # -- are still enforced; only the optimality bar is loosened.
        common = {"expand": True, "print_time": True, "structure_detection": "auto"}
        try:
            opti.solver("fatrop", {**common,
                                   "fatrop": {"print_level": 0,
                                              "max_iter": int(cfg['max_iter']),
                                              "tol": float(cfg['tol']),
                                              "acceptable_tol": float(cfg['acceptable_tol']),
                                              "acceptable_iter": int(cfg['acceptable_iter']),
                                              "constr_viol_tol": float(cfg['constr_viol_tol'])}})
            self.solver_name = "fatrop"
        except Exception as e:  # pragma: no cover
            opti.solver("ipopt",
                        {"expand": True, "print_time": True},
                        {"print_level": 0, "max_iter": int(cfg['max_iter']),
                         "tol": float(cfg['tol']),
                         "acceptable_tol": float(cfg['acceptable_tol']),
                         "constr_viol_tol": float(cfg['constr_viol_tol']),
                         "linear_solver": "mumps"})
            self.solver_name = "ipopt"

    # ------------------------------------------------------------------
    def _set_obstacle_params(self, obstacles):
        """obstacles: list of dicts (len == n_obs) with keys
        pos (2,), vel (2,), psi, length, radius. Constant-velocity rollout."""
        N, dt = self.N, self.dt
        for i, ob in enumerate(obstacles):
            pB = np.zeros((2, N + 1))
            qB = np.zeros((2, N + 1))
            pos = np.asarray(ob['pos'], float)
            vel = np.asarray(ob['vel'], float)
            for j in range(N + 1):
                c = pos + j * dt * vel
                pe, qe = obstacle_endpoints_np(c, ob['psi'], ob['length'])
                pB[:, j] = pe
                qB[:, j] = qe
            self.opti.set_value(self.p_pB[i], pB)
            self.opti.set_value(self.p_qB[i], qB)
        self.opti.set_value(self.p_Robs, [float(ob['radius']) for ob in obstacles])

    def solve(self, x0, goal, u_prev, obstacles):
        """Returns dict: ok, u (2,), X (4,N+1), t_wall_total, iters."""
        N = self.N
        x0 = np.asarray(x0, float)
        self.opti.set_value(self.p_X0, x0)
        self.opti.set_value(self.p_goal, np.asarray(goal, float))
        self.opti.set_value(self.p_Uprev, np.asarray(u_prev, float))
        self._set_obstacle_params(obstacles)

        # warm start
        if self._warm_X is not None:
            for j in range(N + 1):
                self.opti.set_initial(self.X[j], self._warm_X[:, j])
            for j in range(N):
                self.opti.set_initial(self.U[j], self._warm_U[:, j])
        else:
            for j in range(N + 1):
                self.opti.set_initial(self.X[j], x0)
            for j in range(N):
                self.opti.set_initial(self.U[j], np.zeros(2))

        ok = True
        t_wall = float('nan')
        iters = -1
        try:
            sol = self.opti.solve()
            st = sol.stats()
            t_wall = float(st.get('t_wall_total') or float('nan'))
            iters = int(st.get('iter_count', -1))
            X_sol = np.array([sol.value(self.X[j]) for j in range(N + 1)]).T
            U_sol = np.array([sol.value(self.U[j]) for j in range(N)]).T
            u0 = U_sol[:, 0].copy()
        except Exception:
            ok = False
            u0 = np.array([0.0, 0.0])           # safe stop: V0 = 0, delta0 = 0
            X_sol = np.tile(x0.reshape(-1, 1), (1, N + 1))
            U_sol = np.zeros((2, N))

        if ok:
            # shift warm start
            self._warm_X = np.zeros((4, N + 1))
            self._warm_X[:, :N] = X_sol[:, 1:]
            self._warm_X[:, N] = X_sol[:, N]
            self._warm_U = np.zeros((2, N))
            self._warm_U[:, :N - 1] = U_sol[:, 1:]
            self._warm_U[:, N - 1] = U_sol[:, N - 1]
        else:
            self._warm_X = None                 # drop bad iterate

        return {'ok': ok, 'u': u0, 'X': X_sol, 't_wall_total': t_wall, 'iters': iters}
