import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np


class EndEffectorPublisher(Node):
    def __init__(self):
        super().__init__("ee_publisher")

        self.publisher_0 = self.create_publisher(Float64, "cmd_joint/_0", 10)
        self.publisher_2 = self.create_publisher(Float64, "cmd_joint/_2", 10)

        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.data = Float64(data=0.0)

    def timer_callback(self):
        self.data.data += 0.1
        self.publisher_0.publish(self.data)
        self.publisher_2.publish(Float64(data=0.5))


def main(args=None):
    rclpy.init(args=args)

    test_cmd_ee_pub = EndEffectorPublisher()

    rclpy.spin(test_cmd_ee_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_cmd_ee_pub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
