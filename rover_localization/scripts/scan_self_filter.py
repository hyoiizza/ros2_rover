#!/usr/bin/env python3
"""Drop the lidar returns that hit the rover's own body.

The RPLIDAR C1 sits 3.3 cm above the floor at the front of the rover, so ~28%
of its beams hit the chassis and wheels behind it (~34% of all points). Those
points move with the robot: rf2o read them as "no motion" and under-counted the
travelled distance by ~4x, and rtabmap / the costmaps marked the robot itself as
an obstacle.

Every beam whose end point falls inside a box around base_link (the footprint
plus a margin) is set to +inf ("no return"); everything outside the box is kept,
so obstacles right in front of the rover are still seen.

    scan          sensor_msgs/LaserScan  input
    scan_filtered sensor_msgs/LaserScan  output, same header and beam layout
"""
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__('scan_self_filter')
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # Box in base_link [m]: footprint 0.6 x 0.6 (rover_navigation costmaps) + 5 cm margin.
        self.x_min = self.declare_parameter('box_x_min', -0.35).value
        self.x_max = self.declare_parameter('box_x_max', 0.35).value
        self.y_min = self.declare_parameter('box_y_min', -0.35).value
        self.y_max = self.declare_parameter('box_y_max', 0.35).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mask = None
        self.mask_key = None

        self.pub = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)

    def body_mask(self, msg):
        """Per-beam upper range limit inside the box (0 where the beam never enters it)."""
        key = (msg.header.frame_id, msg.angle_min, msg.angle_increment, len(msg.ranges))
        if self.mask is not None and self.mask_key == key:
            return self.mask
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, msg.header.frame_id, Time(),
                                                 Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().warn(f'No {msg.header.frame_id} -> {self.base_frame} yet: {e}',
                                   throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        angles = yaw + msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        dx, dy = np.cos(angles), np.sin(angles)
        # Largest range r at which (t + r * d) is still inside the box, per beam (ray/box slab test).
        with np.errstate(divide='ignore', invalid='ignore'):
            tx = np.where(dx > 0, (self.x_max - t.x) / dx, np.where(dx < 0, (self.x_min - t.x) / dx, np.inf))
            ty = np.where(dy > 0, (self.y_max - t.y) / dy, np.where(dy < 0, (self.y_min - t.y) / dy, np.inf))
        inside_origin = self.x_min <= t.x <= self.x_max and self.y_min <= t.y <= self.y_max
        self.mask = np.minimum(tx, ty) if inside_origin else np.zeros(len(msg.ranges))
        self.mask_key = key
        self.get_logger().info(
            f'Dropping returns inside the body box: beams clipped at {self.mask.min():.2f}-{self.mask.max():.2f} m '
            f'from {msg.header.frame_id}')
        return self.mask

    def on_scan(self, msg):
        mask = self.body_mask(msg)
        if mask is None:
            return
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        ranges[ranges <= mask] = np.inf
        msg.ranges = ranges.tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanSelfFilter()
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
