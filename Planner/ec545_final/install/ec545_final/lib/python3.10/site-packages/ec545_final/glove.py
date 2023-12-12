import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import readchar

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher = self.create_publisher(Int32, 'change_target', 10)
        self.timer = self.create_timer(0.5, self.read_and_publish)

    def read_and_publish(self):
        print("Press a number key to change target (0-9), or 'q' to quit:")
        key = readchar.readkey()
        if key.isdigit():
            val = int(key)
            msg = Int32()
            msg.data = val
            self.publisher.publish(msg)
            print(f"Published: {val}")
        elif key == 'q':
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    rclpy.spin(keyboard_publisher)
    keyboard_publisher.destroy_node()

if __name__ == '__main__':
    main()
