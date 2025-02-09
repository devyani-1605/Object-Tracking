import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tabulate import tabulate

class Subscriber(Node):
    def __init__(self):
        super().__init__("Subscriber")
        self.subscription = self.create_subscription(
            String, '/camera_info', self.listener_callback, 10)
        self.data = []  # Store received messages

    def listener_callback(self, msg):
        print(f"\n📩 Received message: {msg.data}")  # Debugging print

        info = msg.data.split(',')
        if len(info) == 6:
            obj_name, x1, y1, x2, y2, depth = info
            self.data.append([obj_name, x1, y1, x2, y2, depth])

            self.display_table()

    def display_table(self):
        headers = ["Object", "X1", "Y1", "X2", "Y2", "Depth (m)"]
        print("\n" + tabulate(self.data, headers, tablefmt="fancy_grid"))

def main(args=None):
    rclpy.init(args=args)
    node = Subscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
