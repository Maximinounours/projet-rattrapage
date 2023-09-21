import sys

from custom_srv_maxime.srv import TargetPos

import rclpy
from rclpy.node import Node


class UserCmdVelClient(Node):
    def __init__(self):
        # Name of client server
        super().__init__("client_target_pos")

        # Create client
        self.cli = self.create_client(TargetPos, "usr_target_pos")

        # Handle timeouts
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        # Request
        self.req = TargetPos.Request()

    def send_request(self, x, y):
        # Fill request content
        self.req.target_pos.x = x
        self.req.target_pos.y = y

        # Wait for available
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future.result()


def main():
    rclpy.init()

    client_cmd_vel = UserCmdVelClient()

    # Send request & get response
    response = client_cmd_vel.send_request(float(sys.argv[1]), float(sys.argv[2]))

    # Display
    client_cmd_vel.get_logger().info(
        f"Distance to target: {response.vector_angle[0]}\nAngle to target: {response.vector_angle[1]}"
    )

    # Shutdown
    client_cmd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
