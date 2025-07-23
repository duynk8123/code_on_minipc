#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import tf_transformations
import serial
import struct
import select
import csv
import os

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')

        self.serial_port = '/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.01)
            self.get_logger().info(f"Đã mở cổng serial {self.serial_port} với baudrate {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở cổng serial: {e}")
            self.ser = None

        self.wheel_base = 0.568      # mét
        self.wheel_radius = 0.0725   # mét

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.038, self.read_serial)  # khoảng 26Hz

        # --- Ghi CSV ---
        self.csv_file_path = os.path.expanduser("/home/intel/robotlaunha/src/articubot_one/odom_log.csv")
        try:
            self.csv_file = open(self.csv_file_path, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'x', 'y', 'theta'])  # header
            self.get_logger().info(f"Đã tạo file CSV: {self.csv_file_path}")
        except Exception as e:
            self.get_logger().error(f"Lỗi tạo file CSV: {e}")
            self.csv_writer = None

    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            return

        v = msg.linear.x             # m/s
        omega = msg.angular.z        # rad/s

        v_left = v - (self.wheel_base / 2.0) * omega
        v_right = v + (self.wheel_base / 2.0) * omega

        w_left = 2 * v_left / self.wheel_radius
        w_right = 2 * v_right / self.wheel_radius

        w_left_i = int(w_left * 1000)
        w_right_i = int(w_right * 1000)

        try:
            payload = struct.pack('<BhhB', 0xAA, w_left_i, w_right_i, 0xFE)
            self.ser.write(payload)
        except Exception as e:
            self.get_logger().error(f"Lỗi gửi UART: {e}")

    def read_serial(self):
        if self.ser is None:
            return

        if select.select([self.ser], [], [], 0)[0]:
            if self.ser.in_waiting >= 14:
                raw = self.ser.read(14)

                if raw[0] != 0xAA or raw[13] != 0xFE:
                    self.get_logger().warn("Dữ liệu không hợp lệ: sai byte đầu/cuối")
                    return

                try:
                    x, y, theta = struct.unpack('<fff', raw[1:13])

                    # --- Ghi CSV ---
                    if self.csv_writer:
                        timestamp = self.get_clock().now().nanoseconds / 1e9  # giây
                        self.csv_writer.writerow([timestamp, x, y, theta])
                        self.csv_file.flush()

                    self.publish_odom_and_tf(x, y, theta)
                except struct.error:
                    self.get_logger().warn("Lỗi unpack dữ liệu")

    def publish_odom_and_tf(self, x, y, theta):
        now = self.get_clock().now().to_msg()
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)

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

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        # Đảm bảo đóng file CSV khi node bị hủy
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Đã dừng node bằng Ctrl+C")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
