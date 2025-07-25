import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

from robot_gui.ros_publisher import ROSPublisher
from std_msgs.msg import String

class ManualWindow(Gtk.Box):
    def __init__(self, stack):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.stack = stack
        self.ros_pub = ROSPublisher()

        self.set_name("manual-screen")
        self.set_margin_top(20)
        self.set_margin_bottom(20)
        self.set_margin_start(20)
        self.set_margin_end(20)

        # === Title ===
        label = Gtk.Label(label="Manual Control Mode")
        label.set_name("manual-title")
        label.set_margin_bottom(10)
        self.pack_start(label, False, False, 0)

        # === Main content box (Horizontal) ===
        content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=40)
        content_box.set_hexpand(True)

        # === LEFT SIDE: Movement Buttons ===
        control_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=20)

        button_grid = Gtk.Grid()
        button_grid.set_row_spacing(20)
        button_grid.set_column_spacing(20)

        self.add_control_button(button_grid, "↑ Forward", 1, 0, self.move_forward)
        self.add_control_button(button_grid, "← Left", 0, 1, self.move_left)
        self.add_control_button(button_grid, "■ Stop", 1, 1, self.stop)
        self.add_control_button(button_grid, "→ Right", 2, 1, self.move_right)
        self.add_control_button(button_grid, "↓ Backward", 1, 2, self.move_backward)

        control_box.pack_start(button_grid, False, False, 0)

        # === RIGHT SIDE: Servo Sliders (horizontal) ===
        servo_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=30)
        slider_data = [
            ("Head Angle", self.on_head_changed),
            ("Left Arm", self.on_left_hand_changed),
            ("Right Arm", self.on_right_hand_changed)
        ]

        for name, callback in slider_data:
            slider_row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
            label = Gtk.Label(label=name)
            label.set_halign(Gtk.Align.CENTER)

            scale = Gtk.Scale(orientation=Gtk.Orientation.HORIZONTAL)
            scale.set_range(0, 180)
            scale.set_value(90)
            scale.set_digits(0)
            scale.set_increments(1, 10)
            scale.set_size_request(200, 40)
            scale.connect("value-changed", callback)

            slider_row.pack_start(label, False, False, 5)
            slider_row.pack_start(scale, False, False, 5)
            servo_box.pack_start(slider_row, False, False, 10)

        # Add both sides
        content_box.pack_start(control_box, True, True, 0)
        content_box.pack_start(servo_box, True, True, 0)
        self.pack_start(content_box, True, True, 0)

        # === Bottom: Back button ===
        back_button = Gtk.Button(label="⏎ Back to Start")
        back_button.set_name("back-button")
        back_button.set_size_request(250, 60)
        back_button.connect("clicked", self.on_back_clicked)
        back_button.set_halign(Gtk.Align.CENTER)
        self.pack_start(back_button, False, False, 10)

        # === Publish Mode Switch to 'manual' ===
        self.publish_mode_switch()

    def add_control_button(self, grid, label, col, row, callback):
        btn = Gtk.Button(label=label)
        btn.set_size_request(150, 80)
        btn.set_name("movement-button")
        btn.connect("clicked", callback)
        grid.attach(btn, col, row, 1, 1)

    def on_back_clicked(self, button):
        self.stack.set_visible_child_name("start")

    # === ROS Publishing Functions ===
    def move_forward(self, button): self.ros_pub.publish_cmd("forward")
    def move_backward(self, button): self.ros_pub.publish_cmd("backward")
    def move_left(self, button): self.ros_pub.publish_cmd("left")
    def move_right(self, button): self.ros_pub.publish_cmd("right")
    def stop(self, button): self.ros_pub.publish_cmd("stop")

    def on_head_changed(self, scale): self.ros_pub.publish_servo("head", int(scale.get_value()))
    def on_left_hand_changed(self, scale): self.ros_pub.publish_servo("left_hand", int(scale.get_value()))
    def on_right_hand_changed(self, scale): self.ros_pub.publish_servo("right_hand", int(scale.get_value()))

    # === Mode Switching ===
    def publish_mode_switch(self):
        mode_msg = String()
        mode_msg.data = 'manual'
        self.ros_pub.publish_mode(mode_msg)
