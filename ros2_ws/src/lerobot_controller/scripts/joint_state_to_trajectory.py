#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointStateToTrajectory(Node):
    def __init__(self):
        super().__init__('joint_state_to_trajectory')

        # Parameters
        self.declare_parameter('input_topic', '/leader/joint_states')
        self.declare_parameter('arm_output_topic', '/follower/arm_controller/joint_trajectory')
        self.declare_parameter('gripper_output_topic', '/follower/gripper_controller/joint_trajectory')
        self.declare_parameter('arm_joints', ['1', '2', '3', '4', '5'])
        self.declare_parameter('gripper_joints', ['6'])
        self.declare_parameter('time_from_start_sec', 0.1)

        input_topic = self.get_parameter('input_topic').value
        arm_output_topic = self.get_parameter('arm_output_topic').value
        gripper_output_topic = self.get_parameter('gripper_output_topic').value
        self.arm_joints = self.get_parameter('arm_joints').value
        self.gripper_joints = self.get_parameter('gripper_joints').value
        time_from_start = self.get_parameter('time_from_start_sec').value

        self.time_from_start = Duration(sec=int(time_from_start), nanosec=int((time_from_start % 1) * 1e9))

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            input_topic,
            self.joint_state_callback,
            10
        )

        # Publishers
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            arm_output_topic,
            10
        )
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            gripper_output_topic,
            10
        )

        self.get_logger().info(f'Joint state to trajectory converter started')
        self.get_logger().info(f'  Input: {input_topic}')
        self.get_logger().info(f'  Arm output: {arm_output_topic}')
        self.get_logger().info(f'  Gripper output: {gripper_output_topic}')
        self.get_logger().info(f'  Arm joints: {self.arm_joints}')
        self.get_logger().info(f'  Gripper joints: {self.gripper_joints}')

    def joint_state_callback(self, msg):
        # Create a mapping from joint name to position
        joint_dict = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_dict[name] = msg.position[i]

        # Publish arm trajectory
        arm_positions = []
        for joint in self.arm_joints:
            if joint in joint_dict:
                arm_positions.append(joint_dict[joint])
            else:
                self.get_logger().warn(f'Joint {joint} not found in joint_states', throttle_duration_sec=5.0)
                return

        if len(arm_positions) == len(self.arm_joints):
            arm_traj = JointTrajectory()
            arm_traj.header.stamp = self.get_clock().now().to_msg()
            arm_traj.joint_names = self.arm_joints

            point = JointTrajectoryPoint()
            point.positions = arm_positions
            point.time_from_start = self.time_from_start
            arm_traj.points = [point]

            self.arm_pub.publish(arm_traj)

        # Publish gripper trajectory
        gripper_positions = []
        for joint in self.gripper_joints:
            if joint in joint_dict:
                gripper_positions.append(joint_dict[joint])
            else:
                self.get_logger().warn(f'Joint {joint} not found in joint_states', throttle_duration_sec=5.0)
                return

        if len(gripper_positions) == len(self.gripper_joints):
            gripper_traj = JointTrajectory()
            gripper_traj.header.stamp = self.get_clock().now().to_msg()
            gripper_traj.joint_names = self.gripper_joints

            point = JointTrajectoryPoint()
            point.positions = gripper_positions
            point.time_from_start = self.time_from_start
            gripper_traj.points = [point]

            self.gripper_pub.publish(gripper_traj)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateToTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
