import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import serial

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.pub = self.create_publisher(Range, '/ultrasonic', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.read_sensor)

    def read_sensor(self):
        line = self.serial_port.readline().decode('utf-8').strip()
        if line:
            try:
                distance = float(line)
                msg = Range()
                msg.range = distance / 100.0  # Assuming cm to meters
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'ultrasonic_frame'
                self.pub.publish(msg)
            except ValueError:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()