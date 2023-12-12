import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64MultiArray
from turtlesim.srv import Kill
import math

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.spawn_turtles()
        self.turtle_velocity_publishers = [
            self.create_publisher(Twist, f'/turtle{i}/cmd_vel', 10) for i in range(1, 7)
        ]
        self.turtle_pose_subscribers = [
            self.create_subscription(Pose, f'/turtle{i}/pose', lambda msg, i=i: self.pose_callback(msg, i), 10) for i in range(1, 7)
        ]
        self.target_pose_subscriber = self.create_subscription(
            Float64MultiArray, 'target_poses', self.target_pose_callback, 10)
        self.multiplier = 10
        self.target_poses = [(0.1 * self.multiplier, 0.1 * self.multiplier), (1 * self.multiplier, 0.1 * self.multiplier), (1 * self.multiplier, 1 * self.multiplier), (0.1 * self.multiplier, 0.5 * self.multiplier), (0.5 * self.multiplier, 1 * self.multiplier), (0.1 * self.multiplier, 1 * self.multiplier)] # Initial target positions for all turtles
        self.current_poses = [(0.1 * self.multiplier, 0.1 * self.multiplier), (1 * self.multiplier, 0.1 * self.multiplier), (1 * self.multiplier, 1 * self.multiplier), (0.1 * self.multiplier, 0.5 * self.multiplier), (0.5 * self.multiplier, 1 * self.multiplier), (0.1 * self.multiplier, 1 * self.multiplier)] # Current psitions and orientations

        # PID parameters
        self.Kp = 1.0
        self.Ki = 0.01
        self.Kd = 0.2
        self.prev_errors = [0] * 6
        self.integral_errors = [0] * 6
    
    def remove_default_turtle(self):
        self.get_logger().info("Waiting for 'kill' service...")
        kill_client = self.create_client(Kill, 'kill')
        while not kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Kill service not available, waiting again...')
        kill_request = Kill.Request()
        kill_request.name = 'turtle1'
        future = kill_client.call_async(kill_request)
        rclpy.spin_until_future_complete(self, future)


    def spawn_request(self, x, y, theta, n):
        request = Spawn.Request()
        request.x = float(x)  # Set initial x position
        request.y = float(y)  # Set initial y position
        request.theta = float(theta)  # Set initial orientation
        request.name = f"turtle{n}"
        self.future = self.client.call_async(request)


    def spawn_turtles(self):
        # Assuming the turtlesim window is a square of 11x11 units
        self.spawn_request(1, 1, 0, 1)  # Bottom left corner, facing right
        self.spawn_request(10, 1, math.pi/2, 2)  # Bottom right corner, facing up
        self.spawn_request(10, 10, math.pi, 3)  # Top right corner, facing left
        self.spawn_request(1, 5, -math.pi/2, 4)  # Top left corner, facing down
        self.spawn_request(5, 1, -math.pi/2, 5)  # Top left corner, facing down
        self.spawn_request(1, 10, -math.pi/2, 6)  # Top left corner, facing down
        # Add more turtles as needed, adjusting their positions and orientations


    def target_pose_callback(self, msg):
        poses = msg.data
        for i in range(6):
            self.target_poses[i] = (poses[i * 2], poses[i * 2 + 1])

    def pose_callback(self, msg, turtle_index):
        self.current_poses[turtle_index - 1] = (msg.x, msg.y, msg.theta)
        self.control_turtle(turtle_index)

    def control_turtle(self, turtle_index):
        idx = turtle_index - 1
        x, y, theta = self.current_poses[idx]
        x_target, y_target = self.target_poses[idx]

        # PID Control
        error_x = x_target - x
        error_y = y_target - y
        distance = math.sqrt(error_x**2 + error_y**2)
        angle_to_target = math.atan2(error_y, error_x)

        # Angular control
        angle_error = angle_to_target - theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # PID calculations
        self.integral_errors[idx] += angle_error
        derivative_error = angle_error - self.prev_errors[idx]
        self.prev_errors[idx] = angle_error

        angular_velocity = (self.Kp * angle_error +
                            self.Ki * self.integral_errors[idx] +
                            self.Kd * derivative_error)

        # Linear velocity control
        linear_velocity = min(1.5, distance) if abs(angle_error) < math.pi / 4 else 0

        # Publish velocity
        vel_msg = Twist()
        vel_msg.linear.x = float(linear_velocity)
        vel_msg.angular.z = float(angular_velocity)
        self.turtle_velocity_publishers[idx].publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    turtle_controller.remove_default_turtle()
    turtle_controller.spawn_turtles()
    rclpy.spin(turtle_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
