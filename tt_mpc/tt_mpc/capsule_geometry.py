"""
capsule_geometry.py  --  numpy capsule geometry shared by the viz node.

Mirrors the symbolic endpoint formulas in capsule_mpc.py EXACTLY. If you change
an offset/length convention, change it in both places (numpy here, casadi there).
"""
import numpy as np


def tractor_capsule(state, L0, M0, L1, off, length):
    """state=[x1,y1,th0,th1] -> (p, q, center, theta) for the tractor capsule."""
    x1, y1, th0, th1 = state
    x0 = x1 + L1 * np.cos(th1) + M0 * np.cos(th0)
    y0 = y1 + L1 * np.sin(th1) + M0 * np.sin(th0)
    cp, sp = np.cos(th0), np.sin(th0)
    c = np.array([x0 + off * cp, y0 + off * sp])
    d = np.array([cp, sp])
    return c - 0.5 * length * d, c + 0.5 * length * d, c, th0


def trailer_capsule(state, off, length):
    """state=[x1,y1,th0,th1] -> (p, q, center, theta) for the trailer capsule."""
    x1, y1, _, th1 = state
    cp, sp = np.cos(th1), np.sin(th1)
    c = np.array([x1 + off * cp, y1 + off * sp])
    d = np.array([cp, sp])
    return c - 0.5 * length * d, c + 0.5 * length * d, c, th1


def obstacle_capsule(x, y, psi, length):
    """-> (p, q) endpoints for an obstacle capsule."""
    cp, sp = np.cos(psi), np.sin(psi)
    c = np.array([x, y])
    d = np.array([cp, sp])
    return c - 0.5 * length * d, c + 0.5 * length * d


def stadium_outline(p, q, R, n_arc=10):
    """Polyline (closed) tracing a capsule (stadium): two sides + two semicircle
    caps of radius R around segment endpoints p, q. Returns (M,2) array."""
    p = np.asarray(p, float)
    q = np.asarray(q, float)
    axis = q - p
    L = np.linalg.norm(axis)
    if L < 1e-9:
        ang = np.linspace(0, 2 * np.pi, 2 * n_arc + 1)
        return p + R * np.column_stack([np.cos(ang), np.sin(ang)])
    t = axis / L
    n = np.array([-t[1], t[0]])               # left normal
    base = np.arctan2(n[1], n[0])             # angle of +normal
    pts = []
    # cap around q: from +n to -n (half turn, CW)
    for a in np.linspace(0, -np.pi, n_arc):
        d = np.array([np.cos(base + a), np.sin(base + a)])
        pts.append(q + R * d)
    # cap around p: from -n to +n
    for a in np.linspace(np.pi, 0, n_arc):
        d = np.array([np.cos(base + a), np.sin(base + a)])
        pts.append(p + R * d)
    pts.append(pts[0])                        # close
    return np.array(pts)


def box_outline(x_min, x_max, y_min, y_max):
    return np.array([[x_min, y_min], [x_max, y_min], [x_max, y_max],
                     [x_min, y_max], [x_min, y_min]])