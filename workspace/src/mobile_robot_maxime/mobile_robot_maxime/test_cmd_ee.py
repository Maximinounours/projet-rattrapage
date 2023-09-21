import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
import numpy as np

class EndEffectorPublisher(Node):

    def __init__(self):
        super().__init__('ee_publisher')
        
        self.publisher_0 = self.create_publisher(Float64, 'cmd_joint/_0', 10) 
        self.publisher_1 = self.create_publisher(Float64, 'cmd_joint/_1', 10) 
        self.publisher_2 = self.create_publisher(Float64, 'cmd_joint/_2', 10) 
        self.publisher_3 = self.create_publisher(Float64, 'cmd_joint/_3', 10) 
        self.publisher_4 = self.create_publisher(Float64, 'cmd_joint/_4', 10) 
        self.publisher_5 = self.create_publisher(Float64, 'cmd_joint/_5', 10)

        
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.data = 6.28*np.random.random(6)-3.14
        msgs = [Float64(data=self.data[i]).data for i in range(6)]
        self.publisher_0.publish(Float64(data=msgs[0]))
        self.publisher_1.publish(Float64(data=msgs[1]))
        self.publisher_2.publish(Float64(data=msgs[2]))
        self.publisher_3.publish(Float64(data=msgs[3]))
        self.publisher_4.publish(Float64(data=msgs[4]))
        self.publisher_5.publish(Float64(data=msgs[5]))

def main(args=None):
    rclpy.init(args=args)

    test_cmd_ee_pub = EndEffectorPublisher()

    rclpy.spin(test_cmd_ee_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_cmd_ee_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()