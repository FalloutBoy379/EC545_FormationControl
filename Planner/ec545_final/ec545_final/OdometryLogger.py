import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion as eq
import math

def adjust_angle_difference(angle_diff):
    """
    Adjusts the angle difference to account for the discontinuity at -180/180 degrees.

    :param angle_diff: The original angle difference which might cross the -180/180 boundary.
    :return: The adjusted angle difference.
    """
    while angle_diff > 180:
        angle_diff -= 360
    while angle_diff < -180:
        angle_diff += 360
    return angle_diff

class OdomListener(Node):

    def __init__(self):
        super().__init__('odom_listener')
        self.subscription = self.create_subscription(
            Odometry,
            'robot4/odometry',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.prevTheta = 0
        self.prevThetaUpdateFlag = 0

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
        if self.prevThetaUpdateFlag == 0:
            self.prevThetaUpdateFlag = 1
            self.prevTheta=theta
        self.get_logger().info('April Tag Raw: "%s"' % theta)
        
        theta1 = theta + adjust_angle_difference(theta - self.prevTheta)
        self.prevTheta = theta
        self.get_logger().info('April Tag orientation: "%s"' % theta1)

def main(args=None):
    rclpy.init(args=args)
    odom_listener = OdomListener()
    rclpy.spin(odom_listener)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
