import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from robot_gui.ros_publisher import ROSPublisher

class ManualWindow(Gtk.Box):
    def __init__(self, stack):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.stack = stack
        self.ros_pub = ROSPublisher()

        self.set_valign(Gtk.Align.CENTER)
        self.set_halign(Gtk.Align.CENTER)

        label = Gtk.Label(label="Manual Control Mode")
        label.set_margin_bottom(10)
        self.pack_start(label, False, False, 0)

        # Create a button grid
        button_grid = Gtk.Grid()
        button_grid.set_row_spacing(10)
        button_grid.set_column_spacing(10)
        self.pack_start(button_grid, False, False, 0)

        # Movement buttons
        self.add_control_button(button_grid, "Forward", 1, 0, self.move_forward)
        self.add_control_button(button_grid, "Left", 0, 1, self.move_left)
        self.add_control_button(button_grid, "Stop", 1, 1, self.stop)
        self.add_control_button(button_grid, "Right", 2, 1, self.move_right)
        self.add_control_button(button_grid, "Backward", 1, 2, self.move_backward)

        # Back to start
        back_button = Gtk.Button(label="Back to Start")
        back_button.connect("clicked", self.on_back_clicked)
        back_button.set_size_request(200, 50)
        back_button.set_margin_top(20)
        self.pack_start(back_button, False, False, 0)

    def add_control_button(self, grid, label, col, row, callback):
        btn = Gtk.Button(label=label)
        btn.set_size_request(100, 50)
        btn.connect("clicked", callback)
        grid.attach(btn, col, row, 1, 1)

    def on_back_clicked(self, button):
        self.stack.set_visible_child_name("start")

    def move_forward(self, button):
        print("FORWARD button clicked")
        self.ros_pub.publish_cmd("forward")

    def move_backward(self, button):
        print("BACKWARD button clicked")
        self.ros_pub.publish_cmd("backward")

    def move_left(self, button):
        print("LEFT button clicked")
        self.ros_pub.publish_cmd("left")

    def move_right(self, button):
        print("RIGHT button clicked")
        self.ros_pub.publish_cmd("right")

    def stop(self, button):
        print("STOP button clicked")
        self.ros_pub.publish_cmd("stop")
