# MIT License

# Copyright (c) 2023 Miguel Ángel González Santamarta

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


"""
Motor controller node.
"""

from rclpy.node import Node

from rover_msgs.msg import MotorsCommand
from rover_motor_controller.lx16a import MotorController


class ControllerNode(Node):

    def __init__(self):
        super().__init__("controller_node")

        # declaring params
        self.declare_parameter("motor_controller_device", "/dev/lx16a") # /dev/ttyTHS1
        self.declare_parameter("baud_rate", 115200)
        # Seconds without a /motors_command before the drive motors are
        # stopped. 0 disables the watchdog.
        self.declare_parameter("command_timeout", 0.5)

        # getting params
        motor_controller_device = (
            self.get_parameter("motor_controller_device")
            .get_parameter_value()
            .string_value
        )
        baud_rate = self.get_parameter("baud_rate").get_parameter_value().integer_value
        self.command_timeout = (
            self.get_parameter("command_timeout").get_parameter_value().double_value
        )

        self.motor_controller = MotorController(motor_controller_device, baud_rate)

        # sub
        self.subscription = self.create_subscription(
            MotorsCommand, "motors_command", self.callback, 10
        )

        # Watchdog: whoever was driving (teleop or Nav2) can die, lose its
        # network link or be killed without ever sending a zero command. The
        # servos hold the last duty they were given, so the rover would keep
        # rolling.
        self.last_command_time = self.get_clock().now()
        self.motors_stopped = True

        if self.command_timeout > 0.0:
            self.watchdog_timer = self.create_timer(0.1, self.watchdog_callback)
            self.get_logger().info(
                f"Command watchdog armed: drive motors stop after "
                f"{self.command_timeout:.2f} s without /motors_command"
            )
        else:
            self.get_logger().warn(
                "Command watchdog DISABLED (command_timeout = 0). The rover "
                "will keep driving if the commanding node dies."
            )

    def callback(self, msg: MotorsCommand) -> None:
        """
        Callback function called when a MotorsCommand message is received from /motors_command topic
        :param list msg: A list of corner and motor lists values
        """

        self.last_command_time = self.get_clock().now()

        if self.motors_stopped:
            self.get_logger().info("Commands resumed")
            self.motors_stopped = False

        # Send angle values to corner motors
        self.motor_controller.corner_to_position(msg.corner_motor)

        # Send speed values to drive motors
        self.motor_controller.send_motor_duty(msg.drive_motor)

    def watchdog_callback(self) -> None:
        """
        Stop the drive motors when /motors_command goes silent
        """

        if self.motors_stopped:
            return

        elapsed = self.get_clock().now() - self.last_command_time
        elapsed = elapsed.nanoseconds / 1e9

        if elapsed < self.command_timeout:
            return

        # Only the drive motors are zeroed. The corner servos are left where
        # they are: snapping the steering to centre while the rover still has
        # momentum would be its own hazard.
        self.get_logger().warn(
            f"No /motors_command for {elapsed:.2f} s, stopping drive motors"
        )
        self.motor_controller.send_motor_duty([0, 0, 0, 0, 0, 0])
        self.motors_stopped = True

    def shutdown(self) -> None:
        """
        Stop motors
        """

        self.get_logger().info("Killing motors")
        self.motor_controller.kill_motors()
