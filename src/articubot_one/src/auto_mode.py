#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class UARTSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender')

        # Cấu hình cổng serial
        try:
            self.ser_tx = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info('Mở UART2 truyền xuống chuỗi "eWIPE"')
        except serial.SerialException:
            self.ser_tx = None
            self.get_logger().error('Could not open serial port')
            return
        try:
            self.ser_rx = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.1)
            self.get_logger().info('Mở UART1 để nhận chuỗi "sWIPE"')
        except serial.SerialException:
            self.ser_rx = None
            self.get_logger().error('Could not open serial port')
            return
        # Gửi lệnh eWIPE một lần khi khởi động
        self.send_wipe_command()

        # Kiểm tra phản hồi UART mỗi 0.1 giây
        self.recv_timer = self.create_timer(0.1, self.read_uart1_response)
        
    def send_wipe_command(self):
        if self.ser_tx and self.ser_tx.is_open:
            command = 'eWIPE\n'
            self.ser_tx.write(command.encode('utf-8'))
            self.get_logger().info(f'Sent to UART2: {command.strip()}')

    def read_uart1_response(self):
        if self.ser_rx and self.ser_rx.in_waiting > 0:
            try:
                data = self.ser_rx.read(64)  # đọc tối đa 64 byte
                # DEBUG: in raw dữ liệu nhị phân
                #self.get_logger().info(f"📦 Raw data: {data}")
                # Tìm sWIPE ở bất kỳ vị trí nào
                if b'sWIPE' in data:
                    self.get_logger().info('Received sWIPE from STM32 (UART1)')
            except Exception as e:
                self.get_logger().error(f"UART1 read error: {str(e)}")

                
    def destroy_node(self):
        if self.ser_tx and self.ser_tx.is_open:
            self.ser_tx.close()
        if self.ser_rx and self.ser_rx.is_open:
            self.ser_rx.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UARTSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down UART sender node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
