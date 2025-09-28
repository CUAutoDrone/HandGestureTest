import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import mediapipe as mp
import cv2

class HandTracker(Node):
    def __init__(self):
        super().__init__('hand_tracker')
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)
        self.publisher = self.create_publisher(Point, '/hand_landmarks', 10)
        self.bridge = CvBridge()
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=2)
        self.get_logger().info("Hand tracker has been started.")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                for lm in hand_landmarks.landmark:
                    point = Point(x=lm.x, y=lm.y, z=lm.z)
                    self.publisher.publish(point)

def main(args=None):
    rclpy.init(args=args)
    node = HandTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
