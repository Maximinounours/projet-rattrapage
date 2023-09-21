import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("publisher_cmd_vel")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.declare_parameter("linear", 2.0)
        self.declare_parameter("angular", 1.0)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = float(
            self.get_parameter("linear").get_parameter_value().double_value
        )
        msg.angular.z = float(
            self.get_parameter("angular").get_parameter_value().double_value
        )
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
