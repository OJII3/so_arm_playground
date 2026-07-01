# teleop_ik/msg/ResetCommand.msg
# Signal to publish a JointTrajectory to move the arm + gripper to a
# pre-configured home pose, bypassing IK.

# Standard ROS header
std_msgs/Header header

# Home joint angles in radians.
# Element order matches joint_names ["1", "2", "3", "4", "5", "6"].
# NaN value at index i = use node parameter `home_j{i+1}_rad` for that joint.
# This allows partial overrides (e.g., override arm but keep gripper default).
float32[6] home_joints

# Trajectory time_from_start in seconds.
# <= 0.0 = use node parameter `reset_duration_sec`.
float32 duration_sec
