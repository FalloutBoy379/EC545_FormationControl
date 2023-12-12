import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class PoseProvider(Node):
    def __init__(self):
        super().__init__('odometryPublisher')
        self.pose_publishers = []
        for i in range(1, 7):
            publisher = self.create_publisher(Twist, f'apriltag/robot{i}/position', 10)
            self.pose_publishers.append(publisher)
            self.create_subscription(Pose, f'turtle{i}/pose', lambda msg, i=i-1: self.pose_callback(msg, i), 10)

    def pose_callback(self, pose_msg, robot_id):
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.x
        twist_msg.linear.y = pose_msg.y
        twist_msg.angular.z = pose_msg.theta

        self.pose_publishers[robot_id].publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    provider = PoseProvider()
    rclpy.spin(provider)
    provider.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
