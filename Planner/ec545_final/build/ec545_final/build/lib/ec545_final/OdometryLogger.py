import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion as eq
import math

class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            'robot1/odometry',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        odometryPose = msg.pose.pose
        x = odometryPose.position.x
        y = odometryPose.position.y
        _,_,theta = eq([odometryPose.orientation.x,
                                                               odometryPose.orientation.y,
                                                               odometryPose.orientation.z,
                                                               odometryPose.orientation.w
                                                               ])
        
        theta = theta*180/math.pi
        self.get_logger().info('April Tag orientation: "%s"' % theta)

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
