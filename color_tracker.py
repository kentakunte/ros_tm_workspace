import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

        # Define range for the target color (e.g., red color)
        lower_color = np.array([0, 120, 70])
        upper_color = np.array([10, 255, 255])

        # Create a mask for the target color
        mask = cv2.inRange(hsv_image, lower_color, upper_color)

        M = cv2.moments(mask)
        if M["m00"] > 0:
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            self.get_logger().info(f"Detected color at: ({cX}, {cY})")

            cv2.circle(current_frame, (cX, cY), 10, (0, 255, 0), -1)

        cv2.imshow("Color Tracking", current_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    color_tracker = ColorTracker()
    rclpy.spin(color_tracker)
    color_tracker.destroy_node()
    rclpy.init()

if __name__ == '__main__':
    main()