import rclpy
from rclpy.node import Node
from kivymd.app import MDApp
from kivy.lang import Builder
from kivy.core.window import Window
from threading import Thread
from example_interfaces.msg import Int64

Window.size = (300, 300)
Window.fullscreen = False

KV = '''
MDScreen:
    md_bg_color: 1, 0.627, 0.478, 1

    MDFabButton:
        id: fab_button
        icon: "power"
        style: "large"
        pos_hint: {"center_x": .5, "center_y": .80}
        on_release: app.on_button_press()
    MDLabel:
        id: label_start
        text: "Start"
        halign: "center"
        pos_hint: {"center_x": .5, "center_y": .58}
        font_style: "Headline"
        role: "small"
    MDLabel:
        text: "Time:"
        halign: "center"
        pos_hint: {"center_x": .45, "center_y": .3}
        font_style: "Headline"
        role: "large"
    MDLabel:
        id: temp_label
        text: "0"
        halign: "center"
        pos_hint: {"center_x": .65, "center_y": .3}
        font_style: "Headline"
        role: "large"
'''

class ROSNode(Node):
    def __init__(self, temp_label):
        super().__init__("graphical_user_interface")
        self.counter_ = 0
        self.temp_label = temp_label
        self.number_subscriber_ = self.create_subscription(Int64, "robot_news", self.callback_number, 10)
        self.get_logger().info("Number counter has been started.")
        self.is_running = False

    def callback_number(self, msg):
        if self.is_running:
            self.counter_ += 1
            if self.temp_label:
                self.temp_label.text = str(self.counter_)

class InterfaceApp(MDApp):
    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.button_state = False

    def build(self):
        self.theme_cls.primary_palette = "Blue"
        self.gui = Builder.load_string(KV)
        self.root = self.gui
        return self.gui

    def on_button_press(self):
        self.button_state = not self.button_state
        if self.button_state:
            self.start_ros_node()
            self.root.ids.label_start.text = "Stop"
        else:
            self.stop_ros_node()
            self.root.ids.label_start.text = "Start"

    def start_ros_node(self):
        if self.ros_node is None:
            self.ros_node = ROSNode(self.root.ids.temp_label)
            spin_thread = Thread(target=self.spin_ros_node, args=(self.ros_node,))
            spin_thread.start()
        else:
            self.ros_node.is_running = True

    def stop_ros_node(self):
        if self.ros_node is not None:
            self.ros_node.is_running = False

    def spin_ros_node(self, node):
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    app = InterfaceApp()
    app.run()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
