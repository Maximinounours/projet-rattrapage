import rclpy
from rclpy.node import Node

import numpy as np
from geometry_msgs.msg import Twist
from custom_srv_maxime.srv import TargetPos
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose

MIN_DISTANCE = 0.2
MIN_ANGLE = 0.005
TIMER_PERIOD = 0.1


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class ComputeNav(Node):
    def __init__(self):
        super().__init__("compute_nav")
        self.usr_command = False
        self.usr_target_pos = Point()
        self.current_pos = Pose()
        self.locked_target = False

        # Subscribed to Gazebo topic odometry to know current position
        self.odom_sub = self.create_subscription(
            Odometry, "/model/mobile_base/odometry", self.odom_sub_callback, 10
        )
        self.odom_sub

        # Service server to receive target position
        self.target_pos_srv = self.create_service(
            TargetPos, "usr_target_pos", self.target_pos_srv_callback
        )

        # Publisher to send velocity commands
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Publisher loop
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def timer_callback(self):
        """Publishes cmd_vel every TIMER_PERIOD
        Published value is 0 by default, else it is incoming request for SECURED_ITERATIONS
        iterations
        """
        msg = Twist()

        if self.usr_command:
            target_distance, target_rad = compute_target(
                self.usr_target_pos, self.current_pos
            )
            if target_distance < MIN_DISTANCE:
                self.usr_command = False
                self.locked_target = False
                self.get_logger().info(f"POS REACHED")

            else:
                (*_, yaw) = euler_from_quaternion(self.current_pos.orientation)
                diff_angle = target_rad - yaw
                msg.linear.x = 0.0
                msg.angular.z = 0.0

                if np.abs(diff_angle) > MIN_ANGLE and not self.locked_target:
                    diff_angle = diff_angle / 3.14
                    msg.angular.z = cmd_angle(2 * diff_angle)

                else:
                    self.locked_target = True
                    msg.linear.x = cmd_angle(np.min([target_distance, 0.4]))

        self.publisher_.publish(msg)

    def target_pos_srv_callback(self, request, response):
        self.usr_command = True

        # Stores command
        self.usr_target_pos.x = float(request.target_pos.x)
        self.usr_target_pos.y = float(request.target_pos.y)
        self.usr_target_pos.z = float(request.target_pos.z)

        # Display request
        self.get_logger().info(f"Incoming request: {request.target_pos}")

        # Fill response
        out_distance = compute_target(self.usr_target_pos, self.current_pos)
        response.vector_angle = list(out_distance)
        return response

    def odom_sub_callback(self, msg):
        self.current_pos = msg.pose.pose
        roll, pitch, yaw = euler_from_quaternion(self.current_pos.orientation)

        if self.usr_command:
            self.get_logger().info(
                f"X: {self.current_pos.position.x:.3f}"
                + f" Y: {self.current_pos.position.y:.3f}"
                + f" Z: {self.current_pos.position.z:.3f}"
                + f" RPY: {roll:.3f}, {pitch:.3f}, {yaw:.3f}"
            )
            target_distance, target_rad = compute_target(
                self.usr_target_pos, self.current_pos
            )
            dist_angle = target_rad - yaw
            self.get_logger().info(
                f"DISTANCE: {target_distance:.3f} ANGLE: {dist_angle:.3f}"
            )

    def cmd_vel_send_req(self, a, b):
        # Fill request content
        self.req.linear = a
        self.req.angular = b

        # Wait for available
        self.future = self.cmd_vel_cli.call_async(self.req)
        self.get_logger().info(f"SENT: {self.req}")
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"RECEIVED: {self.future.result().command}")


def compute_target(target, current):
    current_pos = current.position
    array1 = np.array([target.x, target.y, target.z])
    array2 = np.array([current_pos.x, current_pos.y, current_pos.z])
    vector_target = array1 - array2
    angle = np.arctan2(vector_target[0], vector_target[1])
    distance = np.linalg.norm(array1 - array2)
    return distance, angle


def cmd_angle(x):
    return 1.2 * (3 * x - 2 * np.sin(x))


def main(args=None):
    rclpy.init(args=args)

    compute_nav_node = ComputeNav()

    rclpy.spin(compute_nav_node)


if __name__ == "__main__":
    main()
