import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

THRESHOLD = 100  # replace with the slider value that worked for you


class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image, 'camera_image', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.count = 0

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        self.count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        _, binary = cv2.threshold(gray, THRESHOLD, 255, cv2.THRESH_BINARY_INV)

        y = int(h * 0.8)
        cols = np.where(binary[y] > 0)[0]

        if len(cols) == 0:
            if self.count % 10 == 0:
                self.get_logger().warn('No track found')
            return

        left, right = int(cols[0]), int(cols[-1])
        centre = (left + right) // 2
        error = centre - w // 2

        if self.count % 10 == 0:  # print about once per second
            self.get_logger().info(
                f'left={left} right={right} centre={centre} error={error}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()