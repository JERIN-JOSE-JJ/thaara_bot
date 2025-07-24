#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class MotorSerialNode(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        try:
            self.arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info("Connected to Arduino")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.arduino = None

    def cmd_vel_callback(self, msg):
        if msg.linear.x > 0:
            cmd = 'F'
        elif msg.linear.x < 0:
            cmd = 'B'
        elif msg.angular.z > 0:
            cmd = 'L'
        elif msg.angular.z < 0:
            cmd = 'R'
        else:
            cmd = 'S'

        if self.arduino:
            self.arduino.write(cmd.encode())
            self.get_logger().info(f"Sent to Arduino: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
