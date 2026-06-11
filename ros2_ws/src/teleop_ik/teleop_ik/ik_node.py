"""IK node for VR teleop of the SO-101 arm.

Subscribes to VR controller pose and gripper commands, solves IK using
Pinocchio, and publishes JointTrajectory commands.
"""

import subprocess

import numpy as np
import pinocchio as pin
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from teleop_ik.coordinate_utils import unity_position_to_ros, unity_quaternion_to_ros

# Joint names used by the arm controller (joints 1-5)
ARM_JOINT_NAMES = ["1", "2", "3", "4", "5"]
# Joint name used by the gripper controller (joint 6)
GRIPPER_JOINT_NAMES = ["6"]

# Gripper joint limits (rad)
GRIPPER_LOWER = -0.174533
GRIPPER_UPPER = 1.74533


class TeleopIKNode(Node):
    """ROS 2 node that converts VR teleop poses to joint trajectories via IK."""

    def __init__(self) -> None:
        super().__init__("teleop_ik_node")

        # -- Declare parameters --
        self.declare_parameter("urdf_path", "")
        self.declare_parameter("end_effector_frame", "gripper")
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("ik_damping", 1e-6)
        self.declare_parameter("ik_max_iterations", 100)
        self.declare_parameter("ik_tolerance", 1e-4)
        self.declare_parameter("trajectory_time_from_start", 0.1)
        self.declare_parameter("unity_conversion", True)

        # -- Load URDF via xacro --
        urdf_path = self.get_parameter("urdf_path").get_parameter_value().string_value
        if not urdf_path:
            self.get_logger().fatal("Parameter 'urdf_path' is required")
            raise RuntimeError("Parameter 'urdf_path' is required")

        urdf_xml = self._process_xacro(urdf_path)

        # -- Build Pinocchio model --
        self._model = pin.buildModelFromXML(urdf_xml)
        self._data = self._model.createData()

        ee_frame_name = (
            self.get_parameter("end_effector_frame")
            .get_parameter_value()
            .string_value
        )
        if not self._model.existFrame(ee_frame_name):
            self.get_logger().fatal(
                f"Frame '{ee_frame_name}' not found in URDF. "
                f"Available frames: {[f.name for f in self._model.frames]}"
            )
            raise RuntimeError(f"Frame '{ee_frame_name}' not found in URDF")
        self._ee_frame_id = self._model.getFrameId(ee_frame_name)

        # Build mapping from arm joint names to Pinocchio joint indices
        self._arm_joint_ids: list[int] = []
        for jname in ARM_JOINT_NAMES:
            if not self._model.existJointName(jname):
                self.get_logger().fatal(f"Joint '{jname}' not found in URDF")
                raise RuntimeError(f"Joint '{jname}' not found in URDF")
            self._arm_joint_ids.append(self._model.getJointId(jname))

        self.get_logger().info(
            f"Pinocchio model loaded: {self._model.nq} DOF, "
            f"EE frame='{ee_frame_name}' (id={self._ee_frame_id})"
        )

        self._unity_conversion = (
            self.get_parameter("unity_conversion").get_parameter_value().bool_value
        )

        # -- Session state --
        self._active = False
        self._arm_init_pos: np.ndarray | None = None  # EE position at session start
        self._unity_anchor_pos: np.ndarray | None = None  # First Unity pose (ROS frame)
        self._q_current = pin.neutral(self._model)

        # -- Subscribers --
        self.create_subscription(
            Bool, "/teleop/active", self._on_active, 10
        )
        self.create_subscription(
            PoseStamped, "/teleop/target_pose", self._on_target_pose, 10
        )
        self.create_subscription(
            Float64, "/teleop/gripper", self._on_gripper, 10
        )
        self.create_subscription(
            JointState, "/follower/joint_states", self._on_joint_states, 10
        )

        # -- Publishers --
        self._arm_pub = self.create_publisher(
            JointTrajectory,
            "/follower/arm_controller/joint_trajectory",
            10,
        )
        self._gripper_pub = self.create_publisher(
            JointTrajectory,
            "/follower/gripper_controller/joint_trajectory",
            10,
        )

        self.get_logger().info("TeleopIKNode initialized, waiting for /teleop/active")

    # ------------------------------------------------------------------ #
    # URDF loading
    # ------------------------------------------------------------------ #

    def _process_xacro(self, xacro_path: str) -> str:
        """Process a xacro file and return the URDF XML string."""
        try:
            import xacro  # type: ignore[import-untyped]

            doc = xacro.process_file(xacro_path)
            return doc.toxml()
        except Exception:
            self.get_logger().warn(
                "xacro Python module failed; falling back to CLI"
            )
            result = subprocess.run(
                ["xacro", xacro_path],
                capture_output=True,
                text=True,
                check=True,
            )
            return result.stdout

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _on_active(self, msg: Bool) -> None:
        if msg.data and not self._active:
            self._start_session()
        elif not msg.data and self._active:
            self._stop_session()

    def _start_session(self) -> None:
        """Start a teleop session: capture current EE position via FK."""
        pin.forwardKinematics(self._model, self._data, self._q_current)
        pin.updateFramePlacements(self._model, self._data)
        self._arm_init_pos = self._data.oMf[self._ee_frame_id].translation.copy()
        self._unity_anchor_pos = None  # Will be set on first target_pose
        self._active = True
        self.get_logger().info(
            f"Session started. EE init pos: {self._arm_init_pos}"
        )

    def _stop_session(self) -> None:
        """Stop the teleop session."""
        self._active = False
        self._arm_init_pos = None
        self._unity_anchor_pos = None
        self.get_logger().info("Session stopped")

    def _on_joint_states(self, msg: JointState) -> None:
        """Update current joint configuration from joint_states."""
        for i, name in enumerate(msg.name):
            if self._model.existJointName(name):
                jid = self._model.getJointId(name)
                idx_q = self._model.joints[jid].idx_q
                if i < len(msg.position):
                    self._q_current[idx_q] = msg.position[i]

    def _on_target_pose(self, msg: PoseStamped) -> None:
        """Receive target pose from Unity and solve IK."""
        if not self._active or self._arm_init_pos is None:
            return

        scale = self.get_parameter("position_scale").get_parameter_value().double_value

        p = msg.pose.position
        o = msg.pose.orientation

        if self._unity_conversion:
            ros_pos = unity_position_to_ros(p.x, p.y, p.z, scale)
            ros_quat = unity_quaternion_to_ros(o.x, o.y, o.z, o.w)
        else:
            ros_pos = np.array([p.x, p.y, p.z]) * scale
            ros_quat = np.array([o.x, o.y, o.z, o.w])

        # On first pose, record anchor
        if self._unity_anchor_pos is None:
            self._unity_anchor_pos = ros_pos.copy()
            self.get_logger().info(f"Anchor set: {self._unity_anchor_pos}")
            return

        # Compute delta from anchor
        delta = ros_pos - self._unity_anchor_pos
        target_pos = self._arm_init_pos + delta

        # Build target SE3
        target_rot = pin.Quaternion(
            ros_quat[3], ros_quat[0], ros_quat[1], ros_quat[2]
        ).toRotationMatrix()
        oMdes = pin.SE3(target_rot, target_pos)

        # Solve IK
        q_result = self._solve_ik(oMdes)
        if q_result is not None:
            self._publish_arm_trajectory(q_result)

    def _on_gripper(self, msg: Float64) -> None:
        """Map gripper value (0..1) to joint 6 angle and publish."""
        if not self._active:
            return

        # Linear mapping: 0 -> GRIPPER_LOWER, 1 -> GRIPPER_UPPER
        val = max(0.0, min(1.0, msg.data))
        angle = GRIPPER_LOWER + val * (GRIPPER_UPPER - GRIPPER_LOWER)
        self._publish_gripper_trajectory(angle)

    # ------------------------------------------------------------------ #
    # IK solver (damped least squares CLIK)
    # ------------------------------------------------------------------ #

    def _solve_ik(self, oMdes: pin.SE3) -> np.ndarray | None:
        """Solve IK using damped least-squares (CLIK).

        Position (3DOF) is prioritized; orientation is best-effort for this
        5-DOF arm.
        """
        damping = self.get_parameter("ik_damping").get_parameter_value().double_value
        max_iter = (
            self.get_parameter("ik_max_iterations").get_parameter_value().integer_value
        )
        tol = self.get_parameter("ik_tolerance").get_parameter_value().double_value

        q = self._q_current.copy()
        dt = 1.0  # integration step

        for _ in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)

            oMcur = self._data.oMf[self._ee_frame_id]
            err_se3 = pin.log6(oMcur.actInv(oMdes))
            err = err_se3.vector  # 6D error

            # Check convergence on position only (first 3 components = linear)
            if np.linalg.norm(err[:3]) < tol:
                # Clamp to joint limits
                q = self._clamp_joints(q)
                return q

            # Compute frame Jacobian in LOCAL_WORLD_ALIGNED frame
            J = pin.computeFrameJacobian(
                self._model,
                self._data,
                q,
                self._ee_frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )

            # Damped least squares: dq = J^T (J J^T + lambda^2 I)^{-1} err
            JJt = J @ J.T + (damping**2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)

            q = pin.integrate(self._model, q, dq * dt)
            q = self._clamp_joints(q)

        self.get_logger().warn("IK did not converge within max iterations")
        # Return best-effort result even if not converged
        return self._clamp_joints(q)

    def _clamp_joints(self, q: np.ndarray) -> np.ndarray:
        """Clamp joint values to their limits defined in the URDF."""
        return np.clip(q, self._model.lowerPositionLimit, self._model.upperPositionLimit)

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #

    def _publish_arm_trajectory(self, q: np.ndarray) -> None:
        """Publish JointTrajectory for the arm (joints 1-5)."""
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        for jid in self._arm_joint_ids:
            idx_q = self._model.joints[jid].idx_q
            point.positions.append(float(q[idx_q]))

        t = self.get_parameter(
            "trajectory_time_from_start"
        ).get_parameter_value().double_value
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points.append(point)
        self._arm_pub.publish(traj)

    def _publish_gripper_trajectory(self, angle: float) -> None:
        """Publish JointTrajectory for the gripper (joint 6)."""
        traj = JointTrajectory()
        traj.joint_names = GRIPPER_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions.append(float(angle))

        t = self.get_parameter(
            "trajectory_time_from_start"
        ).get_parameter_value().double_value
        sec = int(t)
        nanosec = int((t - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        traj.points.append(point)
        self._gripper_pub.publish(traj)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TeleopIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
