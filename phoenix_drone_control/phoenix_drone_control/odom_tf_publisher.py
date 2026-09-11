#!/usr/bin/env python3
"""
odom_tf_publisher.py — Ground-truth odometry → TF bridge
═════════════════════════════════════════════════════════
Gazebo's p3d plugin publishes the drone pose as a nav_msgs/Odometry
message, but not as a TF transform.  RViz needs TF to place the robot
model, the LiDAR scan and the camera frustum in the world.

This node republishes the odometry as the odom → base_link transform,
and also re-stamps the odometry with a consistent frame_id.

Usage:
    ros2 run phoenix_drone_control odom_tf_publisher
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomTfPublisher(Node):
    """Bridges ground-truth odometry into the TF tree."""

    def __init__(self):
        super().__init__('odom_tf_publisher')

        self.declare_parameter('odom_topic', '/phoenix/ground_truth/odom')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        self.declare_parameter('republish_odom', True)

        odom_topic = self.get_parameter('odom_topic').value
        self._parent = self.get_parameter('parent_frame').value
        self._child = self.get_parameter('child_frame').value
        self._republish = self.get_parameter('republish_odom').value

        self._br = TransformBroadcaster(self)

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)

        self._odom_pub = None
        if self._republish:
            self._odom_pub = self.create_publisher(Odometry, '/phoenix/odom', 10)

        self._count = 0
        self.get_logger().info(
            f'Odom→TF bridge: {odom_topic} → '
            f'TF({self._parent} → {self._child})')

    def _odom_cb(self, msg: Odometry):
        now = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self._parent
        t.child_frame_id = self._child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)

        if self._odom_pub is not None:
            out = Odometry()
            out.header.stamp = now
            out.header.frame_id = self._parent
            out.child_frame_id = self._child
            out.pose = msg.pose
            out.twist = msg.twist
            self._odom_pub.publish(out)

        self._count += 1
        if self._count == 1:
            self.get_logger().info('First odometry received — TF is live')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
