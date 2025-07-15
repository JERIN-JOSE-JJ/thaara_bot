import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ROSPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

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
        self.publisher_.publish(msg)
