import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class HandLandmarkPrinter(Node):
    """Subscribe to /hand_landmarks (Point) and print received landmarks with a counter.

    The original tracker publishes each landmark as a separate Point. This node
    will print them as they arrive and include timestamps and a simple index so
    you can inspect the stream.
    """

    def __init__(self):
        super().__init__('hand_landmark_printer')
        self.subscription = self.create_subscription(
            Point, '/hand_landmarks', self.callback, 10)
        self.counter = 0
        self.get_logger().info('Hand landmark printer started.')

    def callback(self, msg: Point):
        self.counter += 1
        # Print a concise line for each received point
        self.get_logger().info(
            f"[{self.counter}] x={msg.x:.6f} y={msg.y:.6f} z={msg.z:.6f}")


def main(args=None):
    rclpy.init(args=args)
    node = HandLandmarkPrinter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
