# Cartesian pose error as the controller sees it, published BEFORE clipping.
#
# The error is what the clip acts on, and the clip is what bounds contact force
# (force ceiling = stiffness x clip). Publishing the pre-clip value plus the
# saturation flags lets a recorder count clip events per step without knowing the
# configured clip values, which is what lets a downstream check tell a genuinely
# small error from one the clip flattened, and catch an axis that stays saturated.

Header header

# Unclipped translational error, metres, base frame. position - position_d.
float64[3] translation

# Unclipped rotational error: the vector part of the difference quaternion,
# rotated into the base frame. Roughly half the physical angle in radians.
float64[3] rotation

# True on each axis where the clip actually bound the error this tick,
# ordered [x, y, z, rx, ry, rz].
bool[6] saturated
