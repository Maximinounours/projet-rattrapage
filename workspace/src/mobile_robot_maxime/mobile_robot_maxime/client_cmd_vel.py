import sys

from custom_srv_maxime.srv import Velocity
import rclpy
from rclpy.node import Node

class UserCmdVelClient(Node):

    def __init__(self):
        # Name of client server
        super().__init__('client_cmd_vel')

        # Create client
        self.cli = self.create_client(Velocity, 'secured_cmd_vel')

        # Handle timeouts
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Request
        self.req = Velocity.Request()


    def send_request(self, a, b):

        # Fill request content
        self.req.linear = a
        self.req.angular = b

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
        f'Result of request: {response.command}')

    # Shutdown
    client_cmd_vel.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()