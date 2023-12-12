import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import math


class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')  # Corrected super function call
        self.client = self.create_client(Spawn, 'spawn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.remove_default_turtle()
        self.spawn_turtles()
        

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
        self.spawn_request(1, 5, math.pi, 3)  # Top right corner, facing left
        self.spawn_request(1, 10, -math.pi/2, 4)  # Top left corner, facing down
        self.spawn_request(10, 10, -math.pi/2, 5)  # Top left corner, facing down
        self.spawn_request(10, 5, -math.pi/2, 6)  # Top left corner, facing down
        # Add more turtles as needed, adjusting their positions and orientations

def main(args=None):
    rclpy.init(args=args)
    spawner = Spawner()
    rclpy.shutdown()

if __name__ == '__main__':
    main()