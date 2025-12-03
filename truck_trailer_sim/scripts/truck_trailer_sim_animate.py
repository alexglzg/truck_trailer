import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from numpy import sin, cos, tan

# =====================================================
#             MODEL PARAMETERS
# =====================================================
L0 = 0.42   # tractor COM → front axle
M0 = -0.02  # tractor COM → hitch   (negative → hitch behind COM)
L1 = 0.537   # hitch → trailer COM

# vehicle rectangle sizes (half-widths and lengths)
TRACTOR_W = 0.35
TRAILER_W = 0.35

TRACTOR_FRONT = L0+0.07
TRACTOR_BACK  = 0.06

TRAILER_FRONT = L1+0.015
TRAILER_BACK  = 0.064

# =====================================================
#             GEOMETRY FUNCTIONS
# =====================================================
def get_vehicle_polygon(x, y, theta, w, lf, lb):
    """Return 4×2 array of vehicle rectangle vertices."""
    pts = np.array([
        [-lb,    -w/2],
        [ lf,    -w/2],
        [ lf,     w/2],
        [-lb,     w/2]
    ])
    R = np.array([
        [cos(theta), -sin(theta)],
        [sin(theta),  cos(theta)]
    ])
    return (R @ pts.T).T + np.array([x, y])

def hinge_position(x1, y1, theta1):
    return np.array([
        x1 + L1*cos(theta1),
        y1 + L1*sin(theta1)
    ])

def tractor_com_from_hinge(hx, hy, theta0):
    return np.array([
        hx + M0*cos(theta0),
        hy + M0*sin(theta0)
    ])


# =====================================================
#             DIFFERENTIAL DYNAMICS
# =====================================================
def tractor_trailer_dynamics(state, u):
    """
    state = [x1, y1, theta1, theta0]
    u     = [V0, delta0]
    """
    x1, y1, th1, th0 = state
    V0, delta0 = u

    beta = th0 - th1
    
    V1 = V0*cos(beta) + M0*(V0*tan(delta0)/L0)*sin(beta)

    dx1 = V1 * cos(th1)
    dy1 = V1 * sin(th1)
    dth1 = (V0*sin(beta))/L1 - (M0*(V0*tan(delta0)/L0)*cos(beta))/L1
    dth0 = V0 * tan(delta0) / L0

    return np.array([dx1, dy1, dth1, dth0])


# =====================================================
#                SIMULATION SETUP
# =====================================================
dt = 0.05
T = 12.0
N = int(T/dt)

state = np.array([0.0, 0.0, 0.0, 0.0])  # x1, y1, theta1, theta0

u = np.array([0.2, -0.2])  # constant input

# logging
history = []

for k in range(N):
    x1, y1, th1, th0 = state
    hx, hy = hinge_position(x1, y1, th1)
    x0, y0 = tractor_com_from_hinge(hx, hy, th0)

    history.append((x0, y0, x1, y1, hx, hy, th0, th1))

    u[1] = -0.2 * np.exp(-0.2 * k * dt)  # steering decay

    state = state + dt * tractor_trailer_dynamics(state, u)

history = np.array(history)


# =====================================================
#                    ANIMATION
# =====================================================
fig, ax = plt.subplots(figsize=(10, 7))
ax.set_aspect("equal")

# trajectory lines
tractor_line, = ax.plot([], [], 'b-', lw=2, label="Tractor COM")
trailer_line, = ax.plot([], [], 'r-', lw=2, label="Trailer COM")
hinge_line,   = ax.plot([], [], 'k--', lw=1, label="Hinge")

# polygons (tractor + trailer)
tractor_patch = plt.Polygon([[0,0],[0,0],[0,0],[0,0]], color='blue', alpha=0.4)
trailer_patch = plt.Polygon([[0,0],[0,0],[0,0],[0,0]], color='red', alpha=0.4)
ax.add_patch(tractor_patch)
ax.add_patch(trailer_patch)

ax.legend()
ax.set_xlim(-2, 6)
ax.set_ylim(-3, 3)
ax.set_title("Tractor–Trailer Simulation")
ax.set_xlabel("x")
ax.set_ylabel("y")


def animate(i):
    x0, y0, x1, y1, hx, hy, th0, th1 = history[i]

    # update trajectories
    tractor_line.set_data(history[:i,0], history[:i,1])
    trailer_line.set_data(history[:i,2], history[:i,3])
    hinge_line.set_data(history[:i,4],   history[:i,5])

    # update polygons
    tractor_poly = get_vehicle_polygon(
        x0, y0, th0, TRACTOR_W, TRACTOR_FRONT, TRACTOR_BACK
    )
    trailer_poly = get_vehicle_polygon(
        x1, y1, th1, TRAILER_W, TRAILER_FRONT, TRAILER_BACK
    )

    tractor_patch.set_xy(tractor_poly)
    trailer_patch.set_xy(trailer_poly)

    return tractor_patch, trailer_patch, tractor_line, trailer_line, hinge_line


anim = animation.FuncAnimation(
    fig, animate, frames=N, interval=40, blit=True
)

# define axis at a fixed scale with equal aspect ratio
ax.set_xlim(-2, 6)
ax.set_ylim(-3, 3)
ax.set_aspect("equal")

plt.show()
