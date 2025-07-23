#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import serial
import struct

class BinaryOdomNode(Node):
    def __init__(self):
        super().__init__('binary_odom_node')
        self.ser = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0', 115200, timeout=1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting >= 12:
            raw = self.ser.read(12)
            try:
                x, y, theta = struct.unpack('<fff', raw)
            except struct.error:
                self.get_logger().warn("Lỗi unpack dữ liệu")
                return
            print(x,y,theta)
            self.publish_odom_and_tf(x, y, theta)

    def publish_odom_and_tf(self, x, y, theta):
        now = self.get_clock().now().to_msg()
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)

        # Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )
        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = BinaryOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()