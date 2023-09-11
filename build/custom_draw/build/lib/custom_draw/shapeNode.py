import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import tkinter as tk

class ShapeNode(Node):
    def __init__(self):
        super().__init__("shape_node")

        self.shape_pub_ = self.create_publisher(String, "/shape_selection", 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.get_logger().info("ShapeNode has been started")

        self.root = tk.Tk()
        self.root.title("Shape Selection")
        self.create_shape_buttons()

        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def create_shape_buttons(self):
        button_frame = tk.Frame(self.root)
        button_frame.pack(padx=10, pady=10)

        stop_button = tk.Button(button_frame, text="Stop", command=self.on_stop)
        stop_button.pack(side=tk.LEFT, padx=5)

        heart_button = tk.Button(button_frame, text="Dodecagon", command=self.on_dodecagon)
        heart_button.pack(side=tk.LEFT, padx=5)

        spiral_button = tk.Button(button_frame, text="Spiral", command=self.on_spiral)
        spiral_button.pack(side=tk.LEFT, padx=5)

        sine_button = tk.Button(button_frame, text="spring", command=self.on_spring)
        sine_button.pack(side=tk.LEFT, padx=5)

    def on_stop(self):
        self.stop_turtle()

    def on_dodecagon(self):
        self.stop_turtle()
        self.publish_shape_command("dodecagon")

    def on_spiral(self):
        self.stop_turtle()
        self.publish_shape_command("spiral")

    def on_spring(self):
        self.stop_turtle()
        self.publish_shape_command("spring")

    def publish_shape_command(self, shape):
        shape_msg = String()
        shape_msg.data = shape
        self.shape_pub_.publish(shape_msg)

    def stop_turtle(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd_vel_msg)
        self.get_logger().info("Stopping turtle")

    def on_shutdown(self):
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    shape_node = ShapeNode()
    shape_node.run()
    shape_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()