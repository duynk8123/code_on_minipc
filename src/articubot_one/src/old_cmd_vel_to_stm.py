#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
class WheelCmdNode(Node):
    def __init__(self):
        super().__init__('wheel_cmd_node')

        # Cấu hình serial
        self.serial_port = '/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0'  # thay theo cổng của bạn
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Đã mở cổng serial {self.serial_port} với baudrate {self.baudrate}")
        except Exception as e:
            self.get_logger().error(f"Lỗi mở cổng serial: {e}")
            self.ser = None

        self.wheel_base = 0.568  # m
        self.wheel_radius = 0.0725  # m

        # Ngưỡng tối thiểu thay đổi để gửi lệnh
        self.linear_threshold = 0.02      # Ngưỡng thay đổi vận tốc tuyến tính
        self.angular_threshold = 0.01     # Ngưỡng thay đổi vận tốc góc
        self.send_interval = 0.2
        
        self.last_stop_send_time = 0.0
        self.stop_send_interval = 0.5  # giây: chỉ gửi lệnh dừng tối đa mỗi 0.5s
        
        # Biến lưu vận tốc cũ
        self.last_v = 0.0
        self.last_w = 0.0
        self.last_send_time = time.time()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )        


    def cmd_vel_callback(self, msg: Twist):
        if self.ser is None:
            return

        v = msg.linear.x
        omega = msg.angular.z
        
        now = time.time()
        dt = now - self.last_send_time
        
        delta_v = abs(v - self.last_v)
        delta_w = abs(omega - self.last_w)
        if delta_v > self.linear_threshold or delta_w > self.angular_threshold or dt > self.send_interval:
             
            v_left = v - (self.wheel_base / 2.0) * omega
            v_right = v + (self.wheel_base / 2.0) * omega
        
            w_left = 2 * v_left / self.wheel_radius
            w_right = 2 * v_right / self.wheel_radius
            cmd_str = f"L{w_left:.2f}R{w_right:.2f}\n"

            try:
                self.ser.write(cmd_str.encode('utf-8'))
                self.get_logger().info(f"Gửi lệnh qua UART: {cmd_str.strip()}")
                print(f"Gửi lệnh qua UART: {cmd_str.strip()}")
                self.last_w_left = w_left
                self.last_w_right = w_right
                self.last_send_time = now
            except Exception as e:
                self.get_logger().error(f"Lỗi gửi dữ liệu UART: {e}")
        else:
            if abs(v) < 0.005 and abs(omega) < 0.01:
                # Nếu tốc độ gần như 0 → gửi lệnh dừng
                if now - self.last_stop_send_time > self.stop_send_interval:
                    try:
                        self.ser.write(b'L0.00R0.00\n')
                        self.get_logger().info("Gửi lệnh DỪNG: L0.00R0.00")
                        self.last_v = 0.0
                        self.last_w = 0.0
                        self.last_send_time = now
                        self.last_stop_send_time = now
                        time.sleep(0.002)
                    except Exception as e:
                        self.get_logger().error(f"Lỗi gửi lệnh dừng: {e}")                        


def main(args=None):
    rclpy.init(args=args)
    node = WheelCmdNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
