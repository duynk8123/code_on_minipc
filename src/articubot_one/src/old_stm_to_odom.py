#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
from sensor_msgs.msg import JointState
import serial
import struct
import math

class STM32OdomNode(Node):
    def __init__(self):
        super().__init__('stm32_odom_node')

        # Mở cổng UART (chỉnh lại đúng cổng nếu cần)
        self.ser = serial.Serial('/dev/serial/by-path/pci-0000:00:14.0-usb-0:4:1.0-port0', 115200, timeout=1)

        # Publisher cho Odometry và JointState
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Broadcaster TF
        self.br = TransformBroadcaster(self)

        # Timer để đọc dữ liệu từ serial    
        self.timer = self.create_timer(0.01, self.read_serial)

        # Pose robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheelbase và bán kính bánh xe (đơn vị: m)
        self.wheel_base = 0.568
        self.wheel_radius = 0.0725

        # Vị trí bánh xe
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        self.last_time = self.get_clock().now()

    def read_serial(self):

            raw = self.ser.read(18)
            if raw[0] == 0xAA and raw[17] == 0x55:
                pos_trai, pos_phai, vel_trai, vel_phai = struct.unpack('<ffff', raw[1:17])
                
                vel_trai = vel_trai/2
                vel_phai = vel_phai/2
                
                # Tính tốc độ tuyến tính và góc quay
                v_trai_tuyentinh = vel_trai * self.wheel_radius # m/s
                v_phai_tuyentinh = vel_phai * self.wheel_radius # m/s
                v = (v_trai_tuyentinh + v_phai_tuyentinh) / 2.0 # trung bình
                w = (v_phai_tuyentinh - v_trai_tuyentinh) / self.wheel_base # rad/s
                
                print(f"vel_trai: {vel_trai:.3f}, vel_phai: {vel_phai:.3f}")
                #print(f"van toc tuyen tinh la: {v:.3f}, van toc goc la:{w:.3f}")
                # Tính thời gian trôi qua
                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds * 1e-9
                self.last_time = now

                # Tính sự thay đổi trong vị trí và góc
                delta_x = v * math.cos(self.theta) * dt
                delta_y = v * math.sin(self.theta) * dt
                delta_theta = w * dt

                # Cập nhật vị trí robot
                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta
                self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
                # Tính quaternion từ góc theta
                q = euler2quat(0, 0, self.theta)  # returns w, x, y, z
                qx, qy, qz, qw = q[1], q[2], q[3], q[0]

                # Gửi TF
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_footprint'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.x = qx
                t.transform.rotation.y = qy
                t.transform.rotation.z = qz
                t.transform.rotation.w = qw
                self.br.sendTransform(t)

                # Gửi Odometry
                odom = Odometry()
                odom.header.stamp = now.to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_footprint'
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation.x = qx
                odom.pose.pose.orientation.y = qy
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x = v
                odom.twist.twist.angular.z = w
                self.odom_pub.publish(odom)

                # Cập nhật vị trí bánh xe (rad)
                self.left_wheel_pos = pos_trai  # đơn vị rad
                self.right_wheel_pos = pos_phai  # đơn vị rad

                # Gửi JointState
                joint_state = JointState()
                joint_state.header.stamp = now.to_msg()
                joint_state.name = ['dc_bt_joint', 'dc_bp_joint']  # Tên joint trong URDF
                joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
                joint_state.velocity = [vel_trai, vel_phai]
                self.joint_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = STM32OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        rclpy.shutdown()

if __name__ == '__main__':

    main()
