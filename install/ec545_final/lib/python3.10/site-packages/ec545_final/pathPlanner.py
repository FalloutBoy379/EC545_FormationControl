import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Int32
import math

class PathPlanner(Node):
    def __init__(self):
        super().__init__('pathPlanner')
        self.all_targets = []  # List of all target points
        self.visited_targets = set()  # Set of visited targets
        self.current_positions = [None] * 6
        self.target_points = [None] * 6  # Current target for each robot
        self.path_publishers = [self.create_publisher(Twist, f'turtle{i}/cmd_vel', 10) for i in range(1, 7)]

        self.initialize_targets()

        # Subscriptions
        self.change_target_subscription = self.create_subscription(Int32, 'change_target', self.change_target_callback, 10)
        for i in range(1, 7):   
            self.create_subscription(Twist, f'apriltag/robot{i}/position', lambda msg, i=i-1: self.update_current_position(msg, i), 10)

        self.timer = self.create_timer(1.0, self.calculate_and_publish_commands)

    def initialize_targets(self):
        # Initialize all target points here
        # Example:
        self.all_targets = [Point(x=2.0, y=6.0), Point(x=4.0, y=6.0), Point(x=2.0, y=4.0), Point(x=4.0, y=4.0), Point(x=2.0, y=2.0), Point(x=4.0, y=2.0)]

    def update_current_position(self, msg, robot_id):
        self.current_positions[robot_id] = msg
        self.assign_new_target_if_needed(robot_id)

    def assign_new_target_if_needed(self, robot_id):
        current_position = self.current_positions[robot_id]
        if current_position is None:
            return

        current_target = self.target_points[robot_id]
        if current_target is None or self.is_target_reached(current_position, current_target):
            self.visited_targets.add(current_target)
            nearest_target = self.find_nearest_unvisited_target(current_position)
            self.target_points[robot_id] = nearest_target

    def is_target_reached(self, current_position, target):
        dx = target.x - current_position.linear.x
        dy = target.y - current_position.linear.y
        distance = math.sqrt(dx**2 + dy**2)
        return distance < 0.1  # Threshold for reaching the target

    def find_nearest_unvisited_target(self, current_position):
        unvisited_targets = [t for t in self.all_targets if t not in self.visited_targets]
        if not unvisited_targets:
            return None

        min_distance = float('inf')
        nearest_target = None
        for target in unvisited_targets:
            dx = target.x - current_position.linear.x
            dy = target.y - current_position.linear.y
            distance = math.sqrt(dx**2 + dy**2)
            if distance < min_distance:
                min_distance = distance
                nearest_target = target

        return nearest_target

    def calculate_and_publish_commands(self):
        for i in range(6):
            current_position = self.current_positions[i]
            target_position = self.target_points[i]

            if current_position is None or target_position is None:
                continue

            dx = target_position.x - current_position.linear.x
            dy = target_position.y - current_position.linear.y
            distance = math.sqrt(dx**2 + dy**2)
            target_angle = math.atan2(dy, dx)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = min(distance, 1.0)  # Replace with your max speed if needed
            cmd_vel_msg.angular.z = self.calculate_angular_velocity(current_position.angular.z, target_angle)
            self.path_publishers[i].publish(cmd_vel_msg)

    def calculate_angular_velocity(self, current_theta, target_theta):
        kp = 0.9  # Proportional gain
        angle_error = target_theta - current_theta
        return kp * angle_error

    def change_target_callback(self, msg):
        self.get_logger().info(f'Received target change request: {msg.data}')
        target_set = self.target_sets.get(msg.data)
        if target_set:
            self.get_logger().info(f'Setting new targets: {target_set}')
            self.target_points = target_set
        else:
            self.get_logger().info(f'Received invalid target set index: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
