import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux_node')
        self.mode = 'manual'  # Default mode
        self.manual_sub = self.create_subscription(Twist, '/manual_cmd_vel', self.manual_callback, 10)
        self.auto_sub = self.create_subscription(Twist, '/autonomous_cmd_vel', self.auto_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/mode', self.mode_switch_callback, 10)

    def mode_switch_callback(self, msg):
        self.mode = msg.data.lower()
        self.get_logger().info(f"Switched to {self.mode} mode")

    def manual_callback(self, msg):
        if self.mode == 'manual':
            self.cmd_pub.publish(msg)

    def auto_callback(self, msg):
        if self.mode == 'autonomous':
            self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
