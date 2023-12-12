import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

class MultiRobotController(Node):
    def __init__(self):
        super().__init__('multi_robot_controller')

        self.num_robots = 6
        self.current_robot = 0
        self.robot_positions = [None] * self.num_robots
        self.robot_reached_target = [False] * self.num_robots

        # Subscribers and publishers for each robot
        self.odometry_subscribers = []
        self.velocity_publishers_robot = []
        self.velocity_publishers_turtle = []

        for robot_id in range(1, self.num_robots + 1):
            self.odometry_subscribers.append(
                self.create_subscription(Odometry, f'robot{robot_id}/odometry', 
                lambda msg, robot_id=robot_id: self.odometry_callback(msg, robot_id), 10)
            )

            self.velocity_publishers_robot.append(
                self.create_publisher(Twist, f'/robot{robot_id}/cmd_vel', 10)
            )

            self.velocity_publishers_turtle.append(
                self.create_publisher(Twist, f'/turtle{robot_id}/cmd_vel', 10)
            )

        self.command_subscriber = self.create_subscription(Int32, 'command_topic', self.command_callback, 10)

        # Target positions for each robot
        self.target_positions = self.generate_target_positions(0)

    def odometry_callback(self, msg, robot_id):
        self.robot_positions[robot_id - 1] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.is_at_position(self.robot_positions[robot_id - 1], self.target_positions[robot_id - 1]):
            self.robot_reached_target[robot_id - 1] = True
            self.move_next_robot()

    def command_callback(self, msg):
        command = msg.data
        self.target_positions = self.generate_target_positions(command)
        self.robot_reached_target = [False] * self.num_robots
        self.current_robot = 0
        self.move_robot(1, self.target_positions[0])

    def generate_target_positions(self, command):
        return [(command + i, command - i) for i in range(self.num_robots)]

    def move_robot(self, robot_id, target_position):
        if self.robot_positions[robot_id - 1] is None:
            self.get_logger().info(f"No position data for robot {robot_id}")
            return

        current_position = self.robot_positions[robot_id - 1]
        twist = Twist()

        # Calculate distance and angle to target
        distance = math.sqrt((target_position[0] - current_position[0]) ** 2 +
                             (target_position[1] - current_position[1]) ** 2)
        angle_to_target = math.atan2(target_position[1] - current_position[1],
                                     target_position[0] - current_position[0])
        current_yaw = self.get_robot_yaw(robot_id)
        angular_difference = self.angle_difference(current_yaw, angle_to_target)

        # Set linear and angular velocities
        twist.linear.x = 0.5 * distance  # Basic proportional control
        twist.angular.z = angular_difference

        # Publish the velocity command
        self.velocity_publishers_robot[robot_id - 1].publish(twist)
        self.velocity_publishers_turtle[robot_id - 1].publish(twist)

    def get_robot_yaw(self, robot_id):
        odometryPose = msg.pose.pose
        x = odometryPose.position.x
        y = odometryPose.position.y
        _,_,theta = eq([odometryPose.orientation.x,
                                                               odometryPose.orientation.y,
                                                               odometryPose.orientation.z,
                                                               odometryPose.orientation.w
                                                               ])
        return yaw_angle

    def angle_difference(self, angle1, angle2):
        diff = angle2 - angle1
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def is_at_position(self, position, target_position):
        distance = math.sqrt((position[0] - target_position[0]) ** 2 +
                             (position[1] - target_position[1]) ** 2)
        return distance < 0.1

    def move_next_robot(self):
        if all(self.robot_reached_target[:self.current_robot + 1]):
            self.current_robot += 1
            if self.current_robot < self.num_robots:
                self.move_robot(self.current_robot + 1, self.target_positions[self.current_robot])

def main(args=None):
    rclpy.init(args=args)
    multi_robot_controller = MultiRobotController()
    rclpy.spin(multi_robot_controller)
    multi_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
