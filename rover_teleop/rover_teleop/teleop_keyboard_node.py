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

"""Keyboard teleop for the rover.

Press w/x to start driving and keep driving until s is pressed. The command
is published continuously while the node is running, so terminal key
auto-repeat does not affect the rover's motion.

Steering is different on purpose: a/d nudge the corner servos and the setting
STAYS until you change it. A terminal delivers one character at a time, so
"hold w and a together" is impossible; persistent steering is what lets you
drive an arc - set the steering, then hold the throttle.

Speeds are normalized, NOT m/s. rover_motor_controller's vel_parser_node maps
linear.x in [-1, 1] onto the LX-16A duty range [-1000, 1000], so speed:=1.0 is
already full throttle.
"""

import sys
import select
import termios
import tty

import geometry_msgs.msg
import rclpy


BANNER = """
Rover keyboard teleop
---------------------------
        w            w / x : drive forward / back
   a    s    d       a / d : steer left / right (stays until changed)
        x            s     : stop and centre the steering

   q / z : speed  +/- 10%
   e / c : steering range +/- 10%

   CTRL-C to quit
---------------------------
"""


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(old_settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def read_keys():
    """Drain every character waiting on stdin, without blocking.

    A held key queues up repeats faster than the publish loop runs; draining
    the whole buffer each tick keeps the rover from lagging behind the keys.
    """

    keys = []

    while select.select([sys.stdin], [], [], 0.0)[0]:
        char = sys.stdin.read(1)

        if not char:
            break

        keys.append(char)

    return keys


def clamp(value, limit):
    return max(-limit, min(limit, value))


def main():
    settings = save_terminal_settings()
    # cbreak, not raw: characters arrive without waiting for Enter and are not
    # echoed, but Ctrl-C still raises KeyboardInterrupt and print() still gets
    # proper newlines.
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init()
    node = rclpy.create_node("teleop_keyboard_node")

    # Normalized, not m/s: vel_parser_node maps [-1, 1] onto the LX-16A duty
    # range. 1.0 is full throttle.
    node.declare_parameter("speed", 0.3)
    node.declare_parameter("turn", 0.5)
    node.declare_parameter("publish_rate", 20.0)
    node.declare_parameter("steer_increment", 0.2)

    speed = node.get_parameter("speed").value
    turn = node.get_parameter("turn").value
    publish_rate = node.get_parameter("publish_rate").value
    steer_increment = node.get_parameter("steer_increment").value

    pub = node.create_publisher(geometry_msgs.msg.Twist, "cmd_vel", 10)

    period = 1.0 / publish_rate
    drive = 0.0  # -1, 0 or +1, stays active until the stop key is pressed
    steering = 0.0  # persists until changed

    def status():
        return f"speed {speed:.2f}\tturn {turn:.2f}\tsteering {steering:+.2f}"

    try:
        print(BANNER)
        print(status())

        while rclpy.ok():
            for key in read_keys():
                if key == "\x03":
                    raise KeyboardInterrupt

                if key == "w":
                    drive = 1.0
                elif key == "x":
                    drive = -1.0
                elif key == "a":
                    steering = clamp(steering + steer_increment, 1.0)
                    print(status())
                elif key == "d":
                    steering = clamp(steering - steer_increment, 1.0)
                    print(status())
                elif key == "s":
                    drive = 0.0
                    steering = 0.0
                    print(status())
                elif key in ("q", "z"):
                    speed = clamp(speed * (1.1 if key == "q" else 0.9), 1.0)
                    print(status())
                elif key in ("e", "c"):
                    turn = clamp(turn * (1.1 if key == "e" else 0.9), 1.0)
                    print(status())

            twist = geometry_msgs.msg.Twist()
            twist.linear.x = drive * speed
            twist.angular.z = steering * turn
            pub.publish(twist)

            rclpy.spin_once(node, timeout_sec=period)

    except (KeyboardInterrupt, Exception) as e:
        if not isinstance(e, KeyboardInterrupt):
            print(e)

    finally:
        pub.publish(geometry_msgs.msg.Twist())
        restore_terminal_settings(settings)


if __name__ == "__main__":
    main()
