uint32  id
# pose, map frame
float64 x
float64 y
float64 psi          # capsule heading [rad] (tracker yaw, NOT atan2(vy,vx))
# constant-velocity prediction, map frame (manager rotates body twist -> world)
float64 vx
float64 vy
# capsule geometry
float64 length       # segment length [m]
float64 radius       # inflation radius [m]