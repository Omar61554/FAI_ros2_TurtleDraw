import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math
import time


class TurtleCommander(Node):
    def __init__(self):
        super().__init__("turtle_commander")

        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.shape_sub_ = self.create_subscription(
            String,
            "/shape_selection",
            self.shape_callback,
            10
        )
        self.get_logger().info("TurtleCommander has been started")

    def shape_callback(self, msg):
        shape = msg.data.lower()
        self.get_logger().info(f"Received shape command: {shape}")

        if shape == "stop":
            self.stop_turtle()
        elif shape == "dodecagon":
            self.draw_dodecagon()
        elif shape == "spiral":
            self.draw_spiral()
        elif shape == "spring":
            self.draw_spring()
        else:
            self.get_logger().warn("Invalid shape command received")

    def stop_turtle(self):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.0
        cmd_vel_msg.angular.z = 0.0
        self.cmd_vel_pub_.publish(cmd_vel_msg)
        self.get_logger().info("Stopping turtle")

    def move_turtle(self, linear_speed, angular_speed):
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = linear_speed
        cmd_vel_msg.angular.z = angular_speed
        self.cmd_vel_pub_.publish(cmd_vel_msg)

    def draw_dodecagon(self):
        self.get_logger().info("Drawing dodecagon shape")

        
        for _ in range(12):
            # Move the turtle to the starting position for drawing a dodecagon
            self.move_turtle(2.0, 0.0)
            time.sleep(1.0)

            side_length = 0.8
            angle = math.pi / 3  # 60 degrees in radians
            for _ in range(6):
                self.move_turtle(side_length, 0.0)
                self.move_turtle(0.0, angle)
                self.get_logger().info("Moving turtle")
                time.sleep(0.1)

            self.stop_turtle()
            time.sleep(1.0)

        self.get_logger().info("Drawing dodecagoncomplete")
    
    def draw_spiral(self):
        self.get_logger().info("Drawing spiral shape")

        # Move the turtle to draw a spiral shape
        linear_speed = 1.0
        angular_speed = 2.0
        t0 = time.time()
        while (time.time() - t0) < 10:  # Draw for 10 seconds
            self.move_turtle(linear_speed, angular_speed)
            linear_speed += 0.1  # Increase the linear speed to create the spiral shape
            self.get_logger().info("Moving turtle")
            time.sleep(0.1)
        self.stop_turtle()
        self.get_logger().info("Drawing spiral complete")

    def draw_spring(self):
        self.get_logger().info("Drawing spring shape")

        # Move the turtle to draw a spring shape
        amplitude = 10.0
        frequency = 1.0
        t0 = time.time()
        while (time.time() - t0) < 20:  # Draw for 20 seconds
            t = time.time() - t0  # Get elapsed time in seconds
            x = amplitude * math.sin(2 * math.pi * frequency * t)  # Compute x position based on the sine wave equation
            y = 0.5  # Set y position to create a visible sine wave
            theta = 0.0  # Keep heading angle constant at 0

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = x
            cmd_vel_msg.linear.y = y
            cmd_vel_msg.angular.z = theta

            self.cmd_vel_pub_.publish(cmd_vel_msg)
            self.get_logger().info("Moving turtle")
            time.sleep(0.1)

        self.stop_turtle()
        self.get_logger().info("Drawing spring complete")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()