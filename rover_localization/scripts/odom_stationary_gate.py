#!/usr/bin/env python3
"""Hold rf2o's pose while the rover is standing still.

rf2o compares each scan with the previous one only, so people walking past a
parked rover read as ego-motion: in a 12 min stationary run the odometry drifted
0.8 m while the scans showed < 8 cm. The EKF takes x/y from rf2o alone, so it
drifted with it.

The rover moves only when it is commanded (the motor controller stops the drive
motors 0.5 s after the last command) or when it is turned by hand (the gyro sees
that). While neither is the case, rf2o's motion increments are dropped and the
output pose holds; once moving, increments are chained onto the held pose, so
the output never jumps.

    odom_rf2o        nav_msgs/Odometry  input (rf2o)
    cmd_vel          geometry_msgs/Twist
    imu              sensor_msgs/Imu
    odom_rf2o_gated  nav_msgs/Odometry  output, same frames as the input
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class StationaryGate:
    """ROS-free core: chains (x, y, yaw) increments of the input only while moving."""

    def __init__(self, cmd_hold_s=1.0, cmd_epsilon=0.005, gyro_threshold=0.05):
        self.cmd_hold = cmd_hold_s
        self.cmd_eps = cmd_epsilon
        self.gyro_thr = gyro_threshold
        self.last_cmd_motion = -math.inf   # time of the last non-zero command [s]
        self.gyro_z = 0.0
        self.prev = None                   # previous input pose (x, y, yaw)
        self.out = None                    # output pose (x, y, yaw)

    def on_cmd(self, t, linear_x, angular_z):
        if abs(linear_x) > self.cmd_eps or abs(angular_z) > self.cmd_eps:
            self.last_cmd_motion = t

    def on_gyro(self, gyro_z):
        self.gyro_z = gyro_z

    def moving(self, t):
        return t - self.last_cmd_motion <= self.cmd_hold or abs(self.gyro_z) > self.gyro_thr

    def on_pose(self, t, x, y, yaw):
        """Returns (x, y, yaw, moving) of the gated pose."""
        if self.prev is None:
            self.prev = self.out = (x, y, yaw)
            return (*self.out, self.moving(t))
        px, py, pyaw = self.prev
        self.prev = (x, y, yaw)
        moving = self.moving(t)
        if moving:
            # increment in the previous input frame, applied in the output frame
            c, s = math.cos(pyaw), math.sin(pyaw)
            dx, dy = c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py)
            dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            ox, oy, oyaw = self.out
            c, s = math.cos(oyaw), math.sin(oyaw)
            self.out = (ox + c * dx - s * dy, oy + s * dx + c * dy, oyaw + dyaw)
        return (*self.out, moving)


class OdomStationaryGate(Node):
    def __init__(self):
        super().__init__('odom_stationary_gate')
        self.gate = StationaryGate(
            cmd_hold_s=self.declare_parameter('cmd_hold_s', 1.0).value,
            cmd_epsilon=self.declare_parameter('cmd_epsilon', 0.005).value,
            gyro_threshold=self.declare_parameter('gyro_threshold', 0.05).value)
        self.was_moving = None
        self.pub = self.create_publisher(Odometry, 'odom_rf2o_gated', 10)
        self.create_subscription(Odometry, 'odom_rf2o', self.on_odom, 10)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd, 10)
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        self.gate.on_cmd(self.now_s(), msg.linear.x, msg.angular.z)

    def on_imu(self, msg):
        self.gate.on_gyro(msg.angular_velocity.z)

    def on_odom(self, msg):
        p = msg.pose.pose
        x, y, yaw, moving = self.gate.on_pose(self.now_s(), p.position.x, p.position.y, yaw_of(p.orientation))
        if moving != self.was_moving:
            self.get_logger().info('moving: passing rf2o through' if moving else 'stationary: holding pose')
            self.was_moving = moving
        p.position.x, p.position.y = x, y
        p.orientation.x = p.orientation.y = 0.0
        p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        if not moving:
            msg.twist.twist = Twist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomStationaryGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
