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
    def __init__(self, robot_id, waypoints, next_controller=None, endServoState=0, flipped = 0, multiplier = 0):
        super().__init__('robot_controller_' + str(robot_id))
        self.robot_id = robot_id
        self.waypoints = waypoints
        self.final_waypoint = waypoints[-1] if waypoints else None
        self.next_controller = next_controller
        self.current_pose = None
        self.goal_reached = False
        self.endServo = endServoState
        self.prevAngleError = 0
        self.prevTheta = 0
        self.multiplier = multiplier
        self.prevThetaUpdateFlag = 0
        self.flipped = flipped
        # self.pose_subscriber = self.create_subscription(Pose, f'/turtle{self.robot_id}/pose', self.pose_callback, 10)
        
        self.pose_subscriber = self.create_subscription(Odometry, f'robot{self.robot_id}/odometry', self.pose_callback2, 10)

        # self.velocity_publisher = self.create_publisher(Twist, f'/turtle{self.robot_id}/cmd_vel', 10)
        
        self.velocity_publisher = self.create_publisher(Twist, f'/robot{self.robot_id}/cmd_vel', 10)

        self.final_waypoint_publisher = self.create_publisher(Odometry, f'/robot{self.robot_id}/final_waypoint', 10)

        self.debugger_pub = self.create_publisher(Twist, f'/robot{self.robot_id}/debugger', 10)

        self.goal_reached_publisher = self.create_publisher(Int32, f'/robot{self.robot_id}/servo', 10)
    

        self.path = self.calculate_spline_path(self.current_pose, self.waypoints)

        self.iterator = 1

    def publish_angle(self, angle):
        cmd_vel = Twist()
        cmd_vel.angular.z = adjust_angle_difference(angle - self.current_pose.theta)
        self.velocity_publisher.publish(cmd_vel)

    def pose_callback(self, msg):
        self.current_pose = msg
        if not self.goal_reached:
            # path = self.calculate_spline_path(self.current_pose, self.waypoints)
            cmd_vel = self.calculate_velocity_command(self.path)
            self.velocity_publisher.publish(cmd_vel)
            self.final_waypoint_publisher.publish(self.final_waypoint)

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
            cmd_vel = self.calculate_velocity_command(self.path)
            self.velocity_publisher.publish(cmd_vel)

            if self.is_goal_reached(self.waypoints[-1], self.current_pose):
                self.publish_angle(self.waypoints[-1].theta)
                if abs(self.current_pose.theta)-self.waypoints[-1].theta <  10:
                    # self.goal_reached_publisher.publish(goal_reached_msg)
                    self.goal_reached = True
                    goal_reached_msg = Int32()
                    goal_reached_msg.data = self.endServo
                    self.goal_reached_publisher.publish(goal_reached_msg)
                    self.velocity_publisher.publish(Twist())  # Stop the turtle
                    if self.next_controller:
                        self.next_controller.goal_reached = False  # Activate the next turtle

    def is_goal_reached(self, goal_pose, current_pose):
        distance = math.sqrt((goal_pose.x - current_pose.x) ** 2 + (goal_pose.y - current_pose.y) ** 2)
        return distance < 0.6  # A threshold for reaching the goal

    def calculate_spline_path(self, start_pose, waypoints):
        """
        Calculate a path through a series of waypoints using cubic splines.
        """
        path = [start_pose]

        for wp in waypoints:
            path.append(wp)
        
        return path

    def calculate_velocity_command(self, path):
        """
        Calculate the velocity command based on the path.
        """
        cmd_vel = Twist()
        # self.get_logger().info(f'Path point is: {path}')    
        if not path or len(path) < 2 or path[self.iterator]==None:
            return cmd_vel
    
        if (self.is_goal_reached(path[self.iterator], self.current_pose)) and (self.iterator+1 <= len(self.waypoints)):
            self.iterator = self.iterator + 1
        next_point = path[self.iterator]
        self.get_logger().info(f'Path point is: {path[self.iterator].x}, {path[self.iterator].y}')
        angle_to_next_point = math.atan2(
            next_point.y - self.current_pose.y,
            next_point.x - self.current_pose.x
        )

        angle_to_next_point  *= 180.00/math.pi

        angle_error = angle_to_next_point - self.current_pose.theta
        angle_error = adjust_angle_difference(angle_error)
        cmd_vel.angular.z = -self.multiplier *angle_error # Adjust the gain factor as necessary



        self.get_logger().info(f'Angle to next point: {angle_to_next_point}')
        self.get_logger().info(f'Difference: {cmd_vel.angular.z}')
        if abs(angle_error) < 25.0:  # Adjust the tolerance as necessary
            cmd_vel.linear.x = self.flipped * 40.0  # Adjust the speed as necessary

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
    top_left = Pose(x=3.8, y=-2.00, theta=0.00)
    top_right = Pose(x=3.8, y= 2.00, theta=0.00)
    bottom_left = Pose(x=-3.8, y=-2.00, theta=90.00)
    bottom_right = Pose(x=-3.8, y=2.00, theta=0.00)
    middle_left = Pose(x=0, y=-2.00, theta=0.00)
    middle_right = Pose(x=0, y=2.00, theta=0.00)

    # Define waypoints for each turtle
    waypoints_turtle1 = [top_left, middle_left, bottom_left, bottom_right, middle_right, top_right]
    waypoints_turtle2 = [top_left, middle_left, bottom_left, bottom_right, middle_right]
    waypoints_turtle3 = [top_left, middle_left, bottom_left, bottom_right]
    waypoints_turtle4 = [top_left, middle_left, bottom_left]
    waypoints_turtle5 = [top_left, middle_left]
    waypoints_turtle6 = [top_left]

    waypoints_turtle1_f = [top_right]
    waypoints_turtle2_f = [middle_right]
    waypoints_turtle3_f = [bottom_right]
    waypoints_turtle4_f = [bottom_left]
    waypoints_turtle5_f = [middle_left]
    waypoints_turtle6_f = [top_left]

    controller_turtle6_f = RobotController(6, waypoints_turtle6_f, endServoState=1, flipped=-1, multiplier=0.18)
    controller_turtle5_f = RobotController(5, waypoints_turtle5_f, next_controller=controller_turtle6_f, endServoState=1, flipped=-1, multiplier=0.3)
    controller_turtle4_f = RobotController(4, waypoints_turtle4_f, next_controller=controller_turtle5_f, endServoState=0, flipped=-1, multiplier= 0.3)
    controller_turtle3_f = RobotController(3, waypoints_turtle3_f, next_controller=controller_turtle4_f, endServoState=2, flipped=-1, multiplier= 0.3)
    controller_turtle2_f = RobotController(2, waypoints_turtle2_f, next_controller=controller_turtle3_f, endServoState=3, flipped=-1, multiplier=0.3)
    controller_turtle1_f = RobotController(1, waypoints_turtle1_f, next_controller=controller_turtle2_f,endServoState=3, flipped=-1, multiplier=0.1)

    controller_turtle6 = RobotController(6, waypoints_turtle6, next_controller=controller_turtle1_f, endServoState=0, flipped=-1, multiplier=0.18)
    controller_turtle5 = RobotController(5, waypoints_turtle5, next_controller=controller_turtle6, endServoState=0, flipped=-1, multiplier=0.3)
    controller_turtle4 = RobotController(4, waypoints_turtle4, next_controller=controller_turtle5, endServoState=0, flipped=-1, multiplier= 0.3)
    controller_turtle3 = RobotController(3, waypoints_turtle3, next_controller=controller_turtle4, endServoState=0, flipped=-1, multiplier= 0.3)
    controller_turtle2 = RobotController(2, waypoints_turtle2, next_controller=controller_turtle3, endServoState=0, flipped=-1, multiplier=0.3)
    controller_turtle1 = RobotController(1, waypoints_turtle1, next_controller=controller_turtle2,endServoState=0, flipped=-1, multiplier=0.15)




    controller_turtle1_f.goal_reached = True  
    controller_turtle2_f.goal_reached = True  
    controller_turtle3_f.goal_reached = True  
    controller_turtle4_f.goal_reached = True  
    controller_turtle5_f.goal_reached = True  

    controller_turtle2.goal_reached = True  
    controller_turtle3.goal_reached = True  
    controller_turtle4.goal_reached = True  
    controller_turtle5.goal_reached = True  
    # ... Initialize other turtle controllers
    controllers = [controller_turtle1, controller_turtle2, controller_turtle3, controller_turtle4, controller_turtle5, controller_turtle6,
                   controller_turtle1_f, controller_turtle2_f, controller_turtle3_f, controller_turtle4_f, controller_turtle5_f, controller_turtle6_f]
    def signal_handler(sig, frame):
        stop_robots(controllers)
        rclpy.shutdown()
    
    SPIN_QUEUE.append(controller_turtle1)
    SPIN_QUEUE.append(controller_turtle2)
    SPIN_QUEUE.append(controller_turtle3)
    SPIN_QUEUE.append(controller_turtle4)
    SPIN_QUEUE.append(controller_turtle5)
    SPIN_QUEUE.append(controller_turtle6)
    SPIN_QUEUE.append(controller_turtle1_f)
    SPIN_QUEUE.append(controller_turtle2_f)
    SPIN_QUEUE.append(controller_turtle3_f)
    SPIN_QUEUE.append(controller_turtle4_f)
    SPIN_QUEUE.append(controller_turtle5_f)
    SPIN_QUEUE.append(controller_turtle6_f)

    signal.signal(signal.SIGINT, signal_handler)

    try:
        while rclpy.ok():
                for node in controllers:
                    rclpy.spin_once(node, timeout_sec=(PERIOD / len(controllers)))
    finally:
            rclpy.shutdown()

if __name__ == '__main__':
    main()
