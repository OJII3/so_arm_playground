"""Gamepad teleop node: converts Joy input to IK target pose via velocity control."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float64

from teleop_ik.msg import TargetPoseWithInput

# PS3 DualShock 3 default mapping (Linux kernel driver)
# Axes: 0=LX, 1=LY, 2=RX, 3=RY
# Buttons: 0=Cross, 1=Circle, 2=Triangle, 3=Square,
#          4=L1, 5=R1, 6=L2, 7=R2, 8=Select, 9=Start


class GamepadTeleopNode(Node):
    def __init__(self) -> None:
        super().__init__("gamepad_teleop_node")

        self.declare_parameter("publish_rate", 50.0)
        self.declare_parameter("linear_speed", 0.05)
        self.declare_parameter("vertical_speed", 0.05)
        self.declare_parameter("deadzone", 0.15)
        self.declare_parameter("axis_x", 1)  # left stick Y → forward/back
        self.declare_parameter("axis_y", 0)  # left stick X → left/right
        self.declare_parameter("axis_z", 3)  # right stick Y → up/down
        self.declare_parameter("button_gripper_open", 5)   # R1
        self.declare_parameter("button_gripper_close", 4)  # L1
        self.declare_parameter("button_toggle_active", 0)  # Cross

        self._linear_speed = (
            self.get_parameter("linear_speed").get_parameter_value().double_value
        )
        self._vertical_speed = (
            self.get_parameter("vertical_speed").get_parameter_value().double_value
        )
        self._deadzone = (
            self.get_parameter("deadzone").get_parameter_value().double_value
        )

        self._axis_x = self.get_parameter("axis_x").get_parameter_value().integer_value
        self._axis_y = self.get_parameter("axis_y").get_parameter_value().integer_value
        self._axis_z = self.get_parameter("axis_z").get_parameter_value().integer_value
        self._btn_gripper_open = (
            self.get_parameter("button_gripper_open").get_parameter_value().integer_value
        )
        self._btn_gripper_close = (
            self.get_parameter("button_gripper_close").get_parameter_value().integer_value
        )
        self._btn_toggle = (
            self.get_parameter("button_toggle_active").get_parameter_value().integer_value
        )

        rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._dt = 1.0 / rate

        # State
        self._active = False
        self._prev_toggle_pressed = False
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_z = 0.0
        self._gripper_value = 0.5
        self._joy_received = False
        self._latest_joy: Joy | None = None

        self.create_subscription(Joy, "/joy", self._on_joy, 10)

        self._target_pub = self.create_publisher(
            TargetPoseWithInput, "/teleop/target", 10
        )
        self._active_pub = self.create_publisher(Bool, "/teleop/active", 10)
        self._gripper_pub = self.create_publisher(Float64, "/teleop/gripper", 10)

        self.create_timer(self._dt, self._timer_callback)
        self.get_logger().info(
            f"GamepadTeleopNode ready (rate={rate}Hz). "
            f"Press Cross to activate session."
        )

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self._deadzone:
            return 0.0
        return value

    def _safe_axis(self, joy: Joy, index: int) -> float:
        if index < len(joy.axes):
            return float(joy.axes[index])
        return 0.0

    def _safe_button(self, joy: Joy, index: int) -> bool:
        if index < len(joy.buttons):
            return bool(joy.buttons[index])
        return False

    def _on_joy(self, msg: Joy) -> None:
        if not self._joy_received:
            self._joy_received = True
            self.get_logger().info(
                f"Joy message received (axes={len(msg.axes)}, buttons={len(msg.buttons)})"
            )
        self._latest_joy = msg

    def _timer_callback(self) -> None:
        joy = self._latest_joy
        if joy is None:
            return

        # Toggle active on button press (rising edge)
        toggle_pressed = self._safe_button(joy, self._btn_toggle)
        if toggle_pressed and not self._prev_toggle_pressed:
            self._active = not self._active
            self._active_pub.publish(Bool(data=self._active))
            if self._active:
                self._target_x = 0.0
                self._target_y = 0.0
                self._target_z = 0.0
                self._gripper_value = 0.5
                self.get_logger().info("Session activated")
            else:
                self.get_logger().info("Session deactivated")
        self._prev_toggle_pressed = toggle_pressed

        if not self._active:
            return

        # Velocity integration
        vx = self._apply_deadzone(self._safe_axis(joy, self._axis_x))
        vy = self._apply_deadzone(self._safe_axis(joy, self._axis_y))
        vz = self._apply_deadzone(self._safe_axis(joy, self._axis_z))

        self._target_x += vx * self._linear_speed * self._dt
        self._target_y += vy * self._linear_speed * self._dt
        self._target_z += vz * self._vertical_speed * self._dt

        # Gripper: R1 opens, L1 closes (incremental while held)
        gripper_speed = 2.0  # full travel per second
        if self._safe_button(joy, self._btn_gripper_open):
            self._gripper_value = min(1.0, self._gripper_value + gripper_speed * self._dt)
        if self._safe_button(joy, self._btn_gripper_close):
            self._gripper_value = max(0.0, self._gripper_value - gripper_speed * self._dt)

        # Publish target pose + stick (stick_x=stick_y=0 for gamepad; wrist stays neutral)
        msg = TargetPoseWithInput()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self._target_x
        msg.pose.position.y = self._target_y
        msg.pose.position.z = self._target_z
        msg.pose.orientation.w = 1.0
        msg.stick_x = 0.0
        msg.stick_y = 0.0
        self._target_pub.publish(msg)

        self._gripper_pub.publish(Float64(data=self._gripper_value))


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GamepadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
