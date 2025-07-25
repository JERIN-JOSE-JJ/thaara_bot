import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.cmd_vel_pub = self.create_publisher(Twist, '/manual_cmd_vel', 10)  # Manual commands go to /manual_cmd_vel
        self.mode_pub = self.create_publisher(String, '/mode', 10)  # Mode switching topic
        self.servo_pub = self.create_publisher(String, '/servo_control', 10)  # Optional: Servo control topic if needed

    def publish_cmd(self, direction):
        msg = Twist()
        if direction == "forward":
            msg.linear.x = 0.5
        elif direction == "backward":
            msg.linear.x = -0.5
        elif direction == "left":
            msg.angular.z = 0.5
        elif direction == "right":
            msg.angular.z = -0.5
        elif direction == "stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def publish_mode(self, mode_msg):
        self.mode_pub.publish(mode_msg)

    def publish_servo(self, joint, angle):
        # Optional function for servo control (depends on how you implemented it in Arduino/ROS)
        msg = String()
        msg.data = f"{joint}:{angle}"
        self.servo_pub.publish(msg)
