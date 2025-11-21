import numpy as np
import matplotlib.pyplot as plt
from numpy import sin, cos, tan

# =====================================================
#             MODEL PARAMETERS
# =====================================================
L0 = 0.421   # tractor COM → front axle
M0 = -0.045  # tractor COM → hitch   (negative → hitch behind COM)
L1 = 0.495   # hitch → trailer COM

# vehicle rectangle sizes (half-widths and lengths)
TRACTOR_W = 0.35
TRAILER_W = 0.35

TRACTOR_FRONT = L0+0.07
TRACTOR_BACK  = 0.055

TRAILER_FRONT = L1+0.05
TRAILER_BACK  = 0.05

# constraints
V0_MAX = 0.1       # m/s -> alternatively 0.2
DELTA0_MAX = np.radians(50)  # rad -> alternatively  np.pi/3
BETA_MAX = np.radians(50)    # rad -> alternatively  np.pi/2

# =====================================================
#             GEOMETRY FUNCTIONS
# =====================================================
def get_vehicle_polygon(x, y, theta, w, lf, lb):
    """Return 4×2 array of the rectangle vertices."""
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
    """Trailer COM → hitch."""
    return np.array([x1 + L1*cos(theta1),
                     y1 + L1*sin(theta1)])

def tractor_com_from_hinge(hx, hy, theta0):
    """Hitch → tractor COM."""
    return np.array([hx + M0*cos(theta0),
                     hy + M0*sin(theta0)])


# =====================================================
#             DIFFERENTIAL DYNAMICS
# =====================================================
def tractor_trailer_dynamics(state, u):
    """
    States:
      state = [x1, y1, theta1, theta0]

    Inputs:
      u = [V0, delta0]
    """
    x1, y1, th1, th0 = state
    V0, delta0 = u

    # articulation angle (tractor vs trailer)
    beta = th0 - th1

    # trailer velocity (kinematic relation)
    V1 = V0 * cos(beta) + M0 * (V0 * tan(delta0) / L0) * sin(beta)

    # trailer dynamics
    dx1 = V1 * cos(th1)
    dy1 = V1 * sin(th1)
    dth1 = (V0 * sin(beta)) / L1 - (M0 * (V0 * tan(delta0) / L0) * cos(beta)) / L1

    # tractor yaw dynamics
    dth0 = V0 * tan(delta0) / L0

    return np.array([dx1, dy1, dth1, dth0])


# =====================================================
#                SIMULATION SETUP
# =====================================================
dt = 0.05
T = 10.0
N = int(T/dt)

# initial state
state = np.array([0.0, 0.0, 0.0, 0.0])   # x1, y1, theta1, theta0

# constant open loop inputs
V0_cmd = 0.1
delta0_cmd = 0.1    # steering angle

u = np.array([V0_cmd, delta0_cmd])

# history logs
x1_hist, y1_hist = [], []
x0_hist, y0_hist = [], []
hinge_hist = []

# =====================================================
#                     SIMULATION LOOP
# =====================================================
for k in range(N):

    # log trailer COM
    x1, y1, th1, th0 = state
    x1_hist.append(x1)
    y1_hist.append(y1)

    # hinge position
    hx, hy = hinge_position(x1, y1, th1)
    hinge_hist.append([hx, hy])

    # tractor COM
    x0, y0 = tractor_com_from_hinge(hx, hy, th0)
    x0_hist.append(x0)
    y0_hist.append(y0)

    # simulate
    state = state + dt * tractor_trailer_dynamics(state, u)


# =====================================================
#                 VISUALIZATION
# =====================================================
plt.figure(figsize=(10,7))
plt.plot(x0_hist, y0_hist, 'b-', label="Tractor COM")
plt.plot(x1_hist, y1_hist, 'r-', label="Trailer COM")

plt.axis("equal")
plt.title("Tractor–Trailer Trajectory")
plt.xlabel("x")
plt.ylabel("y")

# final geometry display
x1, y1, th1, th0 = state
hx, hy = hinge_position(x1, y1, th1)
x0, y0 = tractor_com_from_hinge(hx, hy, th0)

tractor_poly = get_vehicle_polygon(x0, y0, th0,
                                   TRACTOR_W, TRACTOR_FRONT, abs(M0))

trailer_poly = get_vehicle_polygon(x1, y1, th1,
                                   TRAILER_W, TRAILER_FRONT, TRAILER_BACK)

# draw polygons
plt.fill(tractor_poly[:,0], tractor_poly[:,1], 'b', alpha=0.4)
plt.fill(trailer_poly[:,0], trailer_poly[:,1], 'r', alpha=0.4)
plt.plot(hx, hy, 'ko', label="Hitch Point")
plt.legend()

plt.show()
