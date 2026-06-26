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


def seg_seg_sq_dist(p1, q1, p2, q2):
    """Numpy mirror of the symbolic seg_seg_sq_dist_sym in capsule_mpc.py."""
    p1, q1, p2, q2 = (np.asarray(a, float) for a in (p1, q1, p2, q2))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f, c, b = d1 @ d1, d2 @ d2, d2 @ r, d1 @ r, d1 @ d2
    denom = a * e - b * b + 1e-9
    s = np.clip((b * f - c * e) / denom, 0.0, 1.0)
    t = np.clip((b * s + f) / (e + 1e-9), 0.0, 1.0)
    s = np.clip((b * t - c) / (a + 1e-9), 0.0, 1.0)
    diff = (p1 + s * d1) - (p2 + t * d2)
    return diff @ diff


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


def rect_outline(center, theta, length, width):
    """Closed 5-point polyline of a body footprint rectangle (LINE_STRIP)."""
    c, s = np.cos(theta), np.sin(theta)
    ax = np.array([c, s])          # along body axis
    ay = np.array([-s, c])         # lateral
    hl, hw = 0.5 * length, 0.5 * width
    center = np.asarray(center, float)
    corners = [center + hl * ax + hw * ay,
               center + hl * ax - hw * ay,
               center - hl * ax - hw * ay,
               center - hl * ax + hw * ay]
    corners.append(corners[0])
    return np.array(corners)


def skeleton_points(state, L0, M0, L1, tractor_off, trailer_off):
    """Kinematic points of interest (map frame). Mirrors the offset chain used
    by the capsule formulas: trailer axle -> hitch -> truck axle, plus the two
    body centers and the front steering axle."""
    x1, y1, th0, th1 = state
    d1 = np.array([np.cos(th1), np.sin(th1)])
    d0 = np.array([np.cos(th0), np.sin(th0)])
    trailer_axle = np.array([x1, y1])
    hitch = trailer_axle + L1 * d1
    truck_axle = hitch + M0 * d0
    return dict(
        trailer_axle=trailer_axle,
        trailer_center=trailer_axle + trailer_off * d1,
        hitch=hitch,
        truck_axle=truck_axle,
        tractor_center=truck_axle + tractor_off * d0,
        front_axle=truck_axle + L0 * d0,
    )