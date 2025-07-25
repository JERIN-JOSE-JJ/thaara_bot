import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

import rclpy
from std_msgs.msg import String

class AutonomyWindow(Gtk.Box):
    def __init__(self, stack):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        self.stack = stack

        # Initialize ROS2 Node & Publisher
        self.node = rclpy.create_node('autonomy_window_node')
        self.mode_pub = self.node.create_publisher(String, '/mode', 10)

        self.set_border_width(20)

        label = Gtk.Label(label="Autonomous Mode")
        self.pack_start(label, True, True, 0)

        start_btn = Gtk.Button(label="Start Autonomy")
        start_btn.connect("clicked", self.on_start_autonomy)
        self.pack_start(start_btn, True, True, 0)

        back_btn = Gtk.Button(label="Back to Start")
        back_btn.connect("clicked", self.on_back_clicked)
        self.pack_start(back_btn, False, False, 0)

    def on_start_autonomy(self, widget):
        msg = String()
        msg.data = 'autonomous'
        self.mode_pub.publish(msg)
        print("Switched to Autonomous Mode")

    def on_back_clicked(self, widget):
        self.stack.set_visible_child_name("start")

