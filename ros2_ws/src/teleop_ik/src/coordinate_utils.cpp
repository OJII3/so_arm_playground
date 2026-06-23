// teleop_ik/src/coordinate_utils.cpp
#include "teleop_ik/coordinate_utils.hpp"

namespace teleop_ik
{

Eigen::Vector3d unity_position_to_ros(double x, double y, double z, double scale)
{
  return Eigen::Vector3d(z * scale, -x * scale, y * scale);
}

Eigen::Vector4d unity_quaternion_to_ros(double x, double y, double z, double w)
{
  Eigen::Vector4d q;
  q.x() = z;
  q.y() = -x;
  q.z() = y;
  q.w() = -w;
  return q;
}

}  // namespace teleop_ik
