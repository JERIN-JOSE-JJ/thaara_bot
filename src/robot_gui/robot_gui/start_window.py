import gi
gi.require_version("Gtk", "3.0")
from gi.repository import Gtk

import rclpy
from std_msgs.msg import String
import threading

class HomeScreen(Gtk.Box):
    def __init__(self, stack):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=20)
        self.stack = stack
        self.set_name("home-screen")
        self.set_border_width(20)

        # ==== ROS 2 Initialization ====
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('gui_mode_switcher')
        self.mode_pub = self.ros_node.create_publisher(String, '/mode', 10)

        # Spin ROS in a background thread
        threading.Thread(target=self.ros_spin, daemon=True).start()

        # ==== Header bar layout (menu + title) ====
        header_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        header_box.set_hexpand(True)
        header_box.set_halign(Gtk.Align.FILL)

        # Menu button (hamburger icon)
        self.menu_button = Gtk.Button(label="☰")
        self.menu_button.set_name("menu-button")
        self.menu_button.set_size_request(40, 40)
        self.menu_button.connect("clicked", self.on_menu_clicked)
        header_box.pack_start(self.menu_button, False, False, 0)

        # Add a title
        title_label = Gtk.Label(label="Home")
        title_label.set_name("home-title")
        title_label.set_halign(Gtk.Align.CENTER)
        header_box.pack_start(title_label, True, True, 0)

        self.pack_start(header_box, False, False, 0)

        # ==== Main control options ====
        control_label = Gtk.Label(label="Select Control Mode")
        control_label.set_name("home-subtitle")
        control_label.set_margin_top(20)
        self.pack_start(control_label, False, False, 0)

        # Buttons box
        button_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=15)
        button_box.set_halign(Gtk.Align.CENTER)
        button_box.set_valign(Gtk.Align.CENTER)

        # Manual Control Button
        manual_btn = Gtk.Button(label="🕹️  Manual Control")
        manual_btn.set_name("home-button")
        manual_btn.set_size_request(300, 70)
        manual_btn.connect("clicked", self.on_manual_clicked)
        button_box.pack_start(manual_btn, False, False, 0)

        # Autonomy Control Button
        auto_btn = Gtk.Button(label="🤖  Autonomy Control")
        auto_btn.set_name("home-button")
        auto_btn.set_size_request(300, 70)
        auto_btn.connect("clicked", self.on_autonomy_clicked)
        button_box.pack_start(auto_btn, False, False, 0)

        self.pack_start(button_box, True, True, 0)

    def ros_spin(self):
        rclpy.spin(self.ros_node)

    def publish_mode(self, mode_str):
        msg = String()
        msg.data = mode_str
        self.mode_pub.publish(msg)
        print(f"Switched to {mode_str.capitalize()} Mode")

    def on_manual_clicked(self, button):
        self.stack.set_visible_child_name("manual")
        self.publish_mode("manual")

    def on_autonomy_clicked(self, button):
        self.stack.set_visible_child_name("autonomy")
        self.publish_mode("autonomous")

    def on_menu_clicked(self, button):
        menu = Gtk.Menu()

        about_item = Gtk.MenuItem(label="About")
        about_item.connect("activate", self.on_about_clicked)
        menu.append(about_item)

        exit_item = Gtk.MenuItem(label="Exit")
        exit_item.connect("activate", Gtk.main_quit)
        menu.append(exit_item)

        menu.show_all()
        menu.popup(None, None, None, None, 0, Gtk.get_current_event_time())

    def on_about_clicked(self, widget):
        dialog = Gtk.MessageDialog(
            transient_for=self.get_toplevel(),
            flags=0,
            message_type=Gtk.MessageType.INFO,
            buttons=Gtk.ButtonsType.OK,
            text="ThaaraBot Control System",
        )
        dialog.format_secondary_text("Developed using GTK3 and ROS 2 on Raspberry Pi 5.")
        dialog.run()
        dialog.destroy()
