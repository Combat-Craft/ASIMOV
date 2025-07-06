import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import keyboard
import time

JOINT_KEY_MAP = [
    ("q", "a"),  # Joint 1: Q (inc), A (dec)
    ("w", "s"),  # Joint 2: W (inc), S (dec)
    ("e", "d"),  # Joint 3: E (inc), D (dec)
    ("r", "f"),  # Joint 4: R (inc), F (dec)
    ("t", "g"),  # Joint 5: T (inc), G (dec)
    ("y", "h"),  # Joint 6: Y (inc), H (dec)
]

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        self.publisher_ = self.create_publisher(String, 'arm_joint_commands', 10)
        self.get_logger().info("Keyboard node started. Press ESC to quit.")

    def run(self):
        try:
            while True:
                for idx, (inc_key, dec_key) in enumerate(JOINT_KEY_MAP):
                    if keyboard.is_pressed(inc_key):
                        msg = String()
                        msg.data = f"{idx},{1}"
                        self.publisher_.publish(msg)
                        time.sleep(0.15)
                    elif keyboard.is_pressed(dec_key):
                        msg = String()
                        msg.data = f"{idx},{-1}"
                        self.publisher_.publish(msg)
                        time.sleep(0.15)
                if keyboard.is_pressed("esc"):
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
