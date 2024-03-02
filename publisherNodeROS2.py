#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
 
from example_interfaces.msg import Int64
 
class RobotNewStationNode(Node):
    def __init__(self):
        super().__init__("robot_news_station")
        self.number_ = 0
        self.number_publisher_ = self.create_publisher(Int64, "robot_news", 10)
        self.number_timer_ = self.create_timer(1.0, self.publish_news)
        self.get_logger().info("Number publisher has been started.")

    def publish_news(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = RobotNewStationNode()
    rclpy.spin(node)
    rclpy.shutdown()

 
if __name__ == "__main__":
    main()


