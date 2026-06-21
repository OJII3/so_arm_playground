"""IK node for VR teleop of the SO-101 arm.

Subscribes to VR controller pose+stick and gripper commands, solves IK using
Pinocchio, and publishes JointTrajectory commands.
"""

import subprocess

from builtin_interfaces.msg import Duration
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from teleop_ik.coordinate_utils import unity_position_to_ros
from teleop_ik.msg import TargetPoseWithInput
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        # -- Stick integration parameters --
        self.declare_parameter("stick_velocity_scale", 1.5)
        self.declare_parameter("stick_deadzone", 0.1)
        self.declare_parameter("stick_max_delta_per_msg", 0.2)
        self.declare_parameter("stick_fallback_dt", 0.0111)

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
        self._position_joint_ids = self._arm_joint_ids[:3]
        self._wrist_joint_ids = self._arm_joint_ids[3:]

        self.get_logger().info(
            f"Pinocchio model loaded: {self._model.nq} DOF, "
            f"EE frame='{ee_frame_name}' (id={self._ee_frame_id})"
        )

        self._unity_conversion = (
            self.get_parameter("unity_conversion").get_parameter_value().bool_value
        )

        # -- Stick integration state --
        self._integrated_stick: tuple[float, float] = (0.0, 0.0)
        self._last_msg_stamp = None  # float seconds (None when uninitialized)

        # -- Session state --
        self._active = False
        self._arm_init_pos: np.ndarray | None = None  # EE position at session start
        self._unity_anchor_pos: np.ndarray | None = None  # First Unity pose (ROS frame)
        self._q_current = pin.neutral(self._model)
        self._q_solution: np.ndarray | None = None
        self._wrist_init_pos: np.ndarray | None = None

        # -- Subscribers --
        self.create_subscription(
            Bool, "/teleop/active", self._on_active, 10
        )
        self.create_subscription(
            TargetPoseWithInput,
            "/teleop/target",
            self._on_target_with_input,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            ),
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
        if msg.data:
            if not self._active:
                self._start_session()
            else:
                # Re-activation: reset stick state while keeping current session.
                self._integrated_stick = (0.0, 0.0)
                self._last_msg_stamp = None
        elif self._active:
            self._stop_session()

    def _start_session(self) -> None:
        """Start a teleop session: capture current EE position via FK and reset stick state."""
        pin.forwardKinematics(self._model, self._data, self._q_current)
        pin.updateFramePlacements(self._model, self._data)
        self._arm_init_pos = self._data.oMf[self._ee_frame_id].translation.copy()
        self._unity_anchor_pos = None  # Will be set on first target_pose
        self._q_solution = self._q_current.copy()
        self._wrist_init_pos = np.array(
            [
                self._q_current[self._model.joints[jid].idx_q]
                for jid in self._wrist_joint_ids
            ]
        )
        self._integrated_stick = (0.0, 0.0)
        self._last_msg_stamp = None
        self._active = True
        self.get_logger().info(
            f"Session started. EE init pos: {self._arm_init_pos}"
        )

    def _stop_session(self) -> None:
        """Stop the teleop session."""
        self._active = False
        self._arm_init_pos = None
        self._unity_anchor_pos = None
        self._q_solution = None
        self._wrist_init_pos = None
        self.get_logger().info("Session stopped")

    def _on_joint_states(self, msg: JointState) -> None:
        """Update current joint configuration from joint_states."""
        for i, name in enumerate(msg.name):
            if self._model.existJointName(name):
                jid = self._model.getJointId(name)
                idx_q = self._model.joints[jid].idx_q
                if i < len(msg.position):
                    self._q_current[idx_q] = msg.position[i]

    def _on_target_with_input(self, msg: TargetPoseWithInput) -> None:
        """Receive target pose + stick from Unity and solve IK."""
        if (
            not self._active
            or self._arm_init_pos is None
            or self._q_solution is None
            or self._wrist_init_pos is None
        ):
            return

        scale = self.get_parameter("position_scale").get_parameter_value().double_value

        p = msg.pose.position
        # pose.orientation is intentionally ignored; wrist is driven by stick.
        _ = msg.pose.orientation  # noqa: F841

        if self._unity_conversion:
            ros_pos = unity_position_to_ros(p.x, p.y, p.z, scale)
        else:
            ros_pos = np.array([p.x, p.y, p.z]) * scale

        # On first pose, record anchor
        if self._unity_anchor_pos is None:
            self._unity_anchor_pos = ros_pos.copy()
            self.get_logger().info(f"Anchor set: {self._unity_anchor_pos}")
            self._last_msg_stamp = self._stamp_to_time(msg.header.stamp)
            return

        # --- Stick integration ---
        stick_scale = (
            self.get_parameter("stick_velocity_scale")
            .get_parameter_value()
            .double_value
        )
        deadzone = (
            self.get_parameter("stick_deadzone")
            .get_parameter_value()
            .double_value
        )
        max_delta = (
            self.get_parameter("stick_max_delta_per_msg")
            .get_parameter_value()
            .double_value
        )
        fallback_dt = (
            self.get_parameter("stick_fallback_dt")
            .get_parameter_value()
            .double_value
        )

        now = self._stamp_to_time(msg.header.stamp)
        if self._last_msg_stamp is None or now is None:
            delta_t = fallback_dt
        else:
            delta_t = now - self._last_msg_stamp
            if delta_t <= 0.0 or delta_t > 0.5:
                delta_t = fallback_dt
        self._last_msg_stamp = now

        # Compute delta from anchor
        delta = ros_pos - self._unity_anchor_pos
        target_pos = self._arm_init_pos + delta

        vx, vy = self._apply_stick_deadzone(
            float(msg.stick_x), float(msg.stick_y), deadzone
        )
        # Per-message cap on the *resulting* delta
        cap_v = max_delta / max(stick_scale * delta_t, 1e-6)
        vx = float(np.clip(vx, -cap_v, cap_v))
        vy = float(np.clip(vy, -cap_v, cap_v))
        delta_vx = vx * stick_scale * delta_t
        delta_vy = vy * stick_scale * delta_t
        self._integrated_stick = (
            self._integrated_stick[0] + delta_vx,
            self._integrated_stick[1] + delta_vy,
        )

        q_seed = self._q_solution.copy()
        # stick_y → joint 4 (pitch), stick_x → joint 5 (roll)
        q_seed[self._model.joints[self._wrist_joint_ids[0]].idx_q] = (
            self._wrist_init_pos[0] + self._integrated_stick[1]
        )
        q_seed[self._model.joints[self._wrist_joint_ids[1]].idx_q] = (
            self._wrist_init_pos[1] + self._integrated_stick[0]
        )
        q_seed = self._clamp_joints(q_seed)

        q_result = self._solve_ik(target_pos, q_seed)
        if q_result is not None:
            self._q_solution = q_result
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

    def _solve_ik(
        self, target_position: np.ndarray, q_seed: np.ndarray
    ) -> np.ndarray | None:
        """Solve position IK using damped least-squares (CLIK)."""
        damping = self.get_parameter("ik_damping").get_parameter_value().double_value
        max_iter = (
            self.get_parameter("ik_max_iterations").get_parameter_value().integer_value
        )
        tol = self.get_parameter("ik_tolerance").get_parameter_value().double_value

        q = self._clamp_joints(q_seed.copy())
        position_velocity_indexes = [
            self._model.joints[jid].idx_v
            for jid in self._position_joint_ids
        ]
        dt = 0.2

        for _ in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)

            oMcur = self._data.oMf[self._ee_frame_id]
            err = target_position - oMcur.translation

            if np.linalg.norm(err) < tol:
                return self._clamp_joints(q)

            J = pin.computeFrameJacobian(
                self._model,
                self._data,
                q,
                self._ee_frame_id,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
            )[:3, position_velocity_indexes]

            JJt = J @ J.T + damping * np.eye(3)
            position_dq = J.T @ np.linalg.solve(JJt, err)
            dq = np.zeros(self._model.nv)
            dq[position_velocity_indexes] = position_dq

            q = pin.integrate(self._model, q, dq * dt)
            q = self._clamp_joints(q)

        self.get_logger().warn("IK did not converge within max iterations")
        return None

    def _apply_stick_deadzone(
        self, x: float, y: float, deadzone: float
    ) -> tuple[float, float]:
        """Apply radial deadzone then rescale to preserve full-range feel."""
        mag = float(np.hypot(x, y))
        if mag <= deadzone or mag < 1e-9 or deadzone >= 1.0:
            return 0.0, 0.0
        scale = (mag - deadzone) / (mag * (1.0 - deadzone))
        return x * scale, y * scale

    def _stamp_to_time(self, stamp) -> float | None:
        """Convert a builtin_interfaces/Time to a float seconds, or None if invalid."""
        try:
            sec = int(stamp.sec)
            nsec = int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return None
        if sec == 0 and nsec == 0:
            # Treat uninitialized stamp as invalid -> caller falls back.
            return None
        return float(sec) + float(nsec) * 1e-9

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
