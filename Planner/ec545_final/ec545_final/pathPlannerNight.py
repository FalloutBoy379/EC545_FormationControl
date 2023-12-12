import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
import numpy as np
from scipy.interpolate import CubicSpline
import math
from tf_transformations import euler_from_quaternion as eq
import signal

SPIN_QUEUE = []
PERIOD = 0.01


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

class RobotController(Node):
    def __init__(self, robot_id, waypoints, next_controller=None, endServoState=0):
        super().__init__('robot_controller_' + str(robot_id))
        self.robot_id = robot_id
        self.waypoints = waypoints
        self.next_controller = next_controller
        self.current_pose = None
        self.goal_reached = False
        self.endServo = endServoState
        self.prevAngleError = 0
        self.prevTheta = 0
        self.prevThetaUpdateFlag = 0
        self.pose_subscriber = self.create_subscription(Pose, f'/turtle{self.robot_id}/pose', self.pose_callback, 10)
        
        self.pose_subscriber = self.create_subscription(Odometry, f'robot{self.robot_id}/odometry', self.pose_callback2, 10)

        self.velocity_publisher = self.create_publisher(Twist, f'/turtle{self.robot_id}/cmd_vel', 10)
        
        self.velocity_publisher = self.create_publisher(Twist, f'/robot{self.robot_id}/cmd_vel', 10)

        self.debugger_pub = self.create_publisher(Twist, f'/robot{self.robot_id}/debugger', 10)
        
        self.goal_reached_publisher = self.create_publisher(Int32, f'/robot{self.robot_id}/servo', 10)


    def pose_callback(self, msg):
        self.current_pose = msg
        if not self.goal_reached:
            path = self.calculate_spline_path(self.current_pose, self.waypoints)
            cmd_vel = self.calculate_velocity_command(path)
            self.velocity_publisher.publish(cmd_vel)

            if self.is_goal_reached(self.waypoints[-1], self.current_pose):
                self.goal_reached = True
                goal_reached_msg = Int32()
                goal_reached_msg.data = self.endServo
                self.goal_reached_publisher.publish(goal_reached_msg)
                self.velocity_publisher.publish(Twist())  # Stop the turtle
                if self.next_controller:
                    self.next_controller.goal_reached = False  # Activate the next turtle
            else:
                goal_reached_msg = Int32()
                goal_reached_msg.data = 0
                self.goal_reached_publisher.publish(goal_reached_msg)
    
    def pose_callback2(self, msg):
        odometryPose = msg.pose.pose
        x = odometryPose.position.x
        y = odometryPose.position.y
        _,_,theta = eq([odometryPose.orientation.x,
                                                               odometryPose.orientation.y,
                                                               odometryPose.orientation.z,
                                                               odometryPose.orientation.w
                                                               ])
        
        theta = theta*180/math.pi
        # if self.prevThetaUpdateFlag == 0:
        #     self.prevThetaUpdateFlag = 1
        #     self.prevTheta=theta
        # self.get_logger().info('April Tag Raw: "%s"' % theta)
        # theta1 = theta + adjust_angle_difference(theta-self.prevTheta)
        # self.prevTheta = theta
        # self.get_logger().info('April Tag orientation: "%s"' % theta1)
        self.current_pose = Pose(x=x,y=y,theta=theta)
        self.get_logger().info(f'Pose: {self.current_pose.x}, {self.current_pose.y}, {self.current_pose.theta}')
        
        if not self.goal_reached:
            path = self.calculate_spline_path(self.current_pose, self.waypoints)
            cmd_vel = self.calculate_velocity_command(path)
            self.velocity_publisher.publish(cmd_vel)

            if self.is_goal_reached(self.waypoints[-1], self.current_pose):
                # Check and adjust orientation
                angle_error = adjust_angle_difference(self.waypoints[-1].theta - self.current_pose.theta)
                if abs(angle_error) > 20.0:  # Orientation not aligned, adjust it
                    cmd_vel.angular.z = -0.1 * angle_error  # Adjust this gain as necessary
                    self.velocity_publisher.publish(cmd_vel)
                else:
                    # Orientation aligned, stop the robot
                    goal_reached_msg = Int32()
                    goal_reached_msg.data = self.endServo
                    self.goal_reached_publisher.publish(goal_reached_msg)
                    self.velocity_publisher.publish(Twist())  # Stop the turtle
                    self.goal_reached = True
                    # Publish goal reached message or perform other end-of-goal tasks
            else:
                if self.next_controller:
                    self.next_controller.goal_reached = False  # Activate the next turtle

    def is_goal_reached(self, goal_pose, current_pose):
        distance = math.sqrt((goal_pose.x - current_pose.x) ** 2 + (goal_pose.y - current_pose.y) ** 2)
        if distance < 0.5:  # A threshold for reaching the position
            angle_error = adjust_angle_difference(goal_pose.theta - current_pose.theta)
            if abs(angle_error) < 5.0:  # A threshold for aligning orientation (in degrees)
                return True
        return False

    def calculate_spline_path(self, start_pose, waypoints):
        """
        Calculate a path through a series of waypoints using cubic splines.
        """
        x_points = [start_pose.x] + [wp.x for wp in waypoints]
        y_points = [start_pose.y] + [wp.y for wp in waypoints]

        # Log the waypoints to verify they are correct
        # for i, wp in enumerate(waypoints):
        #     self.get_logger().info(f'Waypoint {i}: x={wp.x}, y={wp.y}')

        t = [0]
        for i in range(1, len(x_points)):
            dx = x_points[i] - x_points[i - 1]
            dy = y_points[i] - y_points[i - 1]
            t.append(t[-1] + np.sqrt(dx**2 + dy**2))

        cs_x = CubicSpline(t, x_points)
        cs_y = CubicSpline(t, y_points)

        t_fine = np.linspace(0, t[-1], num=100)
        spline_path = [Pose() for _ in t_fine]
        for i, t_val in enumerate(t_fine):
            spline_path[i].x = cs_x(t_val)
            spline_path[i].y = cs_y(t_val)




        return spline_path

    def calculate_velocity_command(self, path):
        """
        Calculate the velocity command based on the path.
        """
        cmd_vel = Twist()

        if not path or len(path) < 2:
            return cmd_vel

        next_point = path[1]
        angle_to_next_point = math.atan2(
            next_point.y - self.current_pose.y,
            next_point.x - self.current_pose.x
        )

        angle_to_next_point  *= 180.00/math.pi

        angle_error = angle_to_next_point - self.current_pose.theta
        angle_error = adjust_angle_difference(angle_error)
        cmd_vel.angular.z = -0.1*angle_error # Adjust the gain factor as necessary



        self.get_logger().info(f'Angle to next point: {angle_to_next_point}')
        self.get_logger().info(f'Difference: {cmd_vel.angular.z}')
        if abs(angle_error) < 40.0:  # Adjust the tolerance as necessary
            cmd_vel.linear.x = -40.0  # Adjust the speed as necessary

        return cmd_vel
    
    def stop_robot(self):
        self.get_logger().info('Stopping robot...')
        stop_msg = Twist()  # Zero velocity message
        self.velocity_publisher.publish(stop_msg)
        self.goal_reached = True

def stop_robots(controllers):
    for controller in controllers:
        controller.stop_robot()


def main(args=None):
    rclpy.init(args=args)
    
    # Define waypoints for each turtle
    waypoints_turtle1 = [Pose(x=2.0, y=-3.0, theta=0.0)]
    waypoints_turtle2 = [Pose(x=2.0, y=3.0, theta=0.0)]
    controller_turtle2 = RobotController(6, waypoints_turtle2, endServoState=1)
    controller_turtle1 = RobotController(2, waypoints_turtle1, next_controller=controller_turtle2, endServoState=0)
    
    controller_turtle2.goal_reached = True  

    # ... Initialize other turtle controllers
    controllers = [controller_turtle1, controller_turtle2]
    def signal_handler(sig, frame):
        stop_robots(controllers)
        rclpy.shutdown()
    
    SPIN_QUEUE.append(controller_turtle1)
    SPIN_QUEUE.append(controller_turtle2)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while rclpy.ok():
                for node in controllers:
                    rclpy.spin_once(node, timeout_sec=(PERIOD / len(controllers)))
    finally:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
