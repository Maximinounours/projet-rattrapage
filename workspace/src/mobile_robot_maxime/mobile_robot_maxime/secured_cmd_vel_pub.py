import sys

# pynput throws an error if we import it before $DISPLAY is set on LINUX
if sys.platform not in ("darwin", "win32"):
    import os

    os.environ.setdefault("DISPLAY", ":0")

from pynput.keyboard import Key
import rclpy
from rclpy.constants import S_TO_NS
from std_msgs.msg import UInt32
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Vector3
from custom_srv_maxime.srv import Velocity
from std_msgs.msg import UInt32

TIMER_PERIOD = 0.2  # second
SECURED_ITERATIONS = 5


class SecuredCmdVelPublisher(Node):
    """Publishes to /cmd_vel every TIMER_PERIOD seconds. Default velocity is 0
    Interface with outside via usr_cmd_vel service. Use given request for SECURED_ITERATIONS
    iterations before going back to default value.
    """

    def __init__(self):
        super().__init__("secured_cmd_vel_publisher")

        # Init attributes for incoming request
        self.usr_command = False
        self.usr_cmd_vel = Twist()

        # Publisher to interface with robot
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        self.subscription = self.create_subscription(
            UInt32, "key_pressed", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        # Read requested velocity (only for x linear and z angular with Velocity.srv)
        # callback updates cmd_vel for SECURED_ITERATIONS iterations
        self.srv = self.create_service(
            Velocity, "secured_cmd_vel", self.request_callback
        )

        # Counter to ensure default value in case of no command
        self.count = 0

        # Publisher loop
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def listener_callback(self, msg):
        cmd_vel = Twist()
        linear_X = 0.0
        angular_Z = 0.0

        if msg.data == Key.f1.value.vk:
            self.logger.info(
                "\n".join(
                    [
                        "Use the arrow keys to change speed.",
                        "[F1] = Show this help",
                        "[Up]/[Down] = Forward and backward",
                        "[Left]/[Right] = Clockwise and counterclockwise"
                        "[Space] = Stop",
                    ]
                )
            )
        elif msg.data == Key.up.value.vk:
            linear_X = 2.0
        elif msg.data == Key.down.value.vk:
            linear_X = -2.0
        elif msg.data == Key.left.value.vk:
            linear_X = 2.0
            angular_Z = 2.0
        elif msg.data == Key.right.value.vk:
            linear_X = 2.0
            angular_Z = -2.0
        else:
            self.logger.debug("Key ignored: {}".format(msg.data))

        cmd_vel.linear.x = linear_X
        cmd_vel.angular.z = angular_Z
        self.publisher_.publish(cmd_vel)

    def timer_callback(self):
        """Publishes cmd_vel every TIMER_PERIOD
        Published value is 0 by default, else it is incoming request for SECURED_ITERATIONS
        iterations
        """
        msg = Twist()

        # If a request is in progress, take those values
        if self.usr_command and self.count < SECURED_ITERATIONS:
            msg.linear.x = self.usr_cmd_vel.linear.x
            msg.angular.z = self.usr_cmd_vel.angular.z

        # Otherwise default value
        else:
            # Assumes Twist() default values are 0
            self.count = 0
            self.usr_command = False

        self.get_logger().info(
            f"Publishing /cmd_vel: lin_x:{msg.linear.x} ang_z:{msg.angular.z}"
        )
        self.publisher_.publish(msg)

        self.count += 1

    def request_callback(self, request, response):
        """Triggers when a value is sent from the client server.
        Stores the incoming request to be used as new published value for some iterations.
        """
        self.usr_command = True
        # Restart secured loop
        self.count = 0

        # Stores command
        # TODO: Why it reads an integer when the client casts the value as float...
        self.usr_cmd_vel.linear.x = float(request.linear)
        self.usr_cmd_vel.angular.z = float(request.angular)

        # Display request
        self.get_logger().info(
            "Incoming request\nlinear: %d angular: %d"
            % (request.linear, request.angular)
        )

        # Fill response
        response.command = self.usr_cmd_vel

        return response


def main(args=None):
    rclpy.init(args=args)

    secured_cmd_vel_node = SecuredCmdVelPublisher()

    rclpy.spin(secured_cmd_vel_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    secured_cmd_vel_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
