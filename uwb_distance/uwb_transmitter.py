#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial

class UWBTransmitter(Node):
    def __init__(self):
        super().__init__('uwb_transmitter')
        self.serial_port = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)
        self.timer = self.create_timer(1.0, self.transmit_uwb)

    def transmit_uwb(self):
        command = "UWB_TRANSMIT_COMMAND\n"  # Replace with the actual command
        self.serial_port.write(command.encode())
        self.get_logger().info("Transmitting UWB signal...")

def main(args=None):
    rclpy.init(args=args)
    node = UWBTransmitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

