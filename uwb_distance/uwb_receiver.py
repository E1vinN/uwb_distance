#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class UWBReceiver(Node):
    def __init__(self):
        super().__init__('uwb_receiver')
        self.publisher_ = self.create_publisher(Float32, 'uwb_distance', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        self.timer = self.create_timer(1.0, self.read_distance)

    def read_distance(self):
        if self.serial_port.in_waiting > 0:
            distance_str = self.serial_port.readline().decode('utf-8').strip()
            try:
                distance = float(distance_str)
                msg = Float32()
                msg.data = distance
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')
            except ValueError:
                self.get_logger().warn(f'Invalid distance data: {distance_str}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

