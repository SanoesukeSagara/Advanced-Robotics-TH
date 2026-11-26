import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class RectangleTrackerNode(Node):
    def __init__(self):
        super().__init__('rectangle_tracker_node')
        # Subscribe to the camera topic
        self.subscription = self.create_subscription(
            Image,
            'camera_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.count = 0

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV BGR image format
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        self.count += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Binarize image to separate the white rectangle from the dark background
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        output = frame.copy()

        if contours:
            # Find the largest contour (the white rectangle paper)
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500:
                # Get minimum area bounding box to handle rotation angles
                rect = cv2.minAreaRect(largest_contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)

                (cx, cy), (width, height), angle = rect

                # Determine the long axis of the rectangle
                edge1 = np.linalg.norm(box[0] - box[1])
                edge2 = np.linalg.norm(box[1] - box[2])

                if edge1 > edge2:
                    pt1 = ((box[0][0] + box[3][0]) // 2, (box[0][1] + box[3][1]) // 2)
                    pt2 = ((box[1][0] + box[2][0]) // 2, (box[1][1] + box[2][1]) // 2)
                else:
                    pt1 = ((box[0][0] + box[1][0]) // 2, (box[0][1] + box[1][1]) // 2)
                    pt2 = ((box[2][0] + box[3][0]) // 2, (box[2][1] + box[3][1]) // 2)

                # 1. Draw central point (blue dot)
                cv2.circle(output, (int(cx), int(cy)), 5, (255, 0, 0), -1)

                # 2. Draw long axis line across the rotating rectangle (red line)
                cv2.line(output, pt1, pt2, (0, 0, 255), 2)
                
                # 3. Draw end triangle markers along the line
                cv2.drawMarker(output, pt1, (0, 0, 255), cv2.MARKER_TRIANGLE_UP, 12, 2)
                cv2.drawMarker(output, pt2, (0, 0, 255), cv2.MARKER_TRIANGLE_DOWN, 12, 2)

        # 4. LIVE STREAM DISPLAY: Runs on every frame for smooth video on your monitor
        cv2.imshow("Arrow Direction", output)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = RectangleTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # Cleanly close OpenCV windows on exit

if __name__ == '__main__':
    main()