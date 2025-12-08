# Represents a convex polytope defined by Ax <= b
# For a rectangle/box, this usually requires 4 inequalities.

geometry_msgs/Point[] vertices  # Useful for visualization
float64[] a_vec                 # Flattened A matrix
float64[] b_vec                 # b vector
int32 num_constraints           # Number of rows in A