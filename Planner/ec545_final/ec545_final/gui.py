import sys
import threading
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtCore import QTimer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose  # or the appropriate message type

class RobotSubscriber(Node):
    def __init__(self, robot_id):
        super().__init__('robot_subscriber_' + str(robot_id))
        self.robot_id = robot_id
        self.current_pose = None
        self.final_waypoint = None

        self.pose_subscriber = self.create_subscription(
            Pose, f'/robot{self.robot_id}/current_pose', self.current_pose_callback, 10)
        self.waypoint_subscriber = self.create_subscription(
            Pose, f'/robot{self.robot_id}/final_waypoint', self.final_waypoint_callback, 10)

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def final_waypoint_callback(self, msg):
        self.final_waypoint = msg

# PyQt5 UI Class
class RobotUI(QMainWindow):
    def __init__(self, robot_controllers):
        super().__init__()
        self.robot_controllers = robot_controllers
        self.initUI()

    def initUI(self):
        self.setGeometry(300, 300, 300, 200)
        self.setWindowTitle('Robot Position Display')

        # Creating a label for each controller
        self.labels = []
        for i, controller in enumerate(self.robot_controllers):
            label = QLabel(f"Robot {controller.robot_id}: Waiting for data...", self)
            label.setGeometry(10, 10 + i*30, 280, 20)
            self.labels.append(label)
            self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(100)  # Update every 100 ms
        self.show()

    def update_status(self):
        for i, controller in enumerate(self.robot_controllers):
            if controller.current_pose:
                x = controller.current_pose.position.x
                y = controller.current_pose.position.y
                waypoint = controller.final_waypoint
                wp_text = f"WP X: {waypoint.x:.2f}, Y: {waypoint.y:.2f}" if waypoint else "No WP"
                self.labels[i].setText(f"Robot {controller.robot_id} Pos: X: {x:.2f}, Y: {y:.2f} | {wp_text}")


# Function to start ROS node
def start_ros_node(node):
    rclpy.spin(node)

# Main function
def main(args=None):
    rclpy.init(args=args)

    # Define the number of robots
    num_robots = 2  # Adjust this number as needed

    # Create a subscriber for each robot
    subscribers = [RobotSubscriber(i) for i in range(1, num_robots + 1)]

    # Start each subscriber node in a separate thread
    threads = [threading.Thread(target=start_ros_node, args=(sub,)) for sub in subscribers]
    for thread in threads:
        thread.start()

    app = QApplication(sys.argv)
    ex = RobotUI(subscribers)

    exit_code = app.exec_()
    rclpy.shutdown()
    for thread in threads:
        thread.join()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()
