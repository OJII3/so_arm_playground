// teleop_ik/include/teleop_ik/coordinate_utils.hpp
#ifndef TELEOP_IK__COORDINATE_UTILS_HPP_
#define TELEOP_IK__COORDINATE_UTILS_HPP_

#include <Eigen/Core>

namespace teleop_ik
{

// Unity (left-handed Y-up, X-right Z-forward) → ROS (right-handed Z-up,
// X-forward Y-left).
// ros_x =  unity_z
// ros_y = -unity_x
// ros_z =  unity_y
Eigen::Vector3d unity_position_to_ros(double x, double y, double z, double scale = 1.0);

// Vector part follows the same axis mapping. w is flipped for handedness.
Eigen::Vector4d unity_quaternion_to_ros(double x, double y, double z, double w);

}  // namespace teleop_ik

#endif  // TELEOP_IK__COORDINATE_UTILS_HPP_
