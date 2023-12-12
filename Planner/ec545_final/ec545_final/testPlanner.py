import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')
        self.control_publishers = []
        for i in range(1, 7):  # For robots 1 to 6
            control_publisher = self.create_publisher(Twist, f'robot{i}/cmd_vel', 10)
            self.control_publishers.append(control_publisher)

        self.timer = self.create_timer(0.1, self.publish_cmd_vel)  # Adjust the timer rate as needed

    def publish_cmd_vel(self):
        msg = Twist()
        msg.linear.x = 0.0  # Set your desired linear speed
        msg.angular.z = 255.0  # Set your desired angular speed

        for controller_publisher in self.control_publishers:
            controller_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    multi_robot_controller = MultiRobotController()
    rclpy.spin(multi_robot_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
