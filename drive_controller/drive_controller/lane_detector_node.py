import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class LaneDetectorNode(Node):
    def __init__(self):
        super().__init__('lane_detector_node')
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
        h, w, _ = frame.shape

        # 1. Define fixed car center position (lower middle of the screen)
        car_center_x = w // 2
        car_center_y = int(h * 0.85)

        # 2. Inspect horizontal row across the lower region of interest (ROI)
        roi_y = int(h * 0.70)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Threshold to isolate white lane stripes on dark background
        _, binary = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
        
        row_pixels = binary[roi_y, :]
        white_indices = np.where(row_pixels > 0)[0]

        output = frame.copy()

        # 3. Calculate lane center and draw steering vector
        if len(white_indices) > 20:
            left_lane_x = int(white_indices[0])
            right_lane_x = int(white_indices[-1])
            
            # Check if both left and right lane boundaries are detected
            if (right_lane_x - left_lane_x) > (w * 0.25):
                lane_center_x = (left_lane_x + right_lane_x) // 2
                lane_center_y = roi_y

                # Draw Blue Box at Car Center
                box_size = 12
                cv2.rectangle(output, 
                              (car_center_x - box_size, car_center_y - box_size),
                              (car_center_x + box_size, car_center_y + box_size),
                              (255, 0, 0), 2)

                # Draw Green Steering Arrow towards the lane center
                cv2.arrowedLine(output, 
                               (car_center_x, car_center_y), 
                               (lane_center_x, lane_center_y), 
                               (0, 255, 0), 3, tipLength=0.15)
            else:
                self.draw_warning(output, car_center_x, car_center_y)
        else:
            self.draw_warning(output, car_center_x, car_center_y)

        # 4. LIVE STREAM DISPLAY: Runs on every single frame for smooth real-time video
        cv2.imshow("Midlane Street Detection", output)
        cv2.waitKey(1)

    def draw_warning(self, img, cx, cy):
        # Draw Red Box & WARNING Text when lane detection is lost
        box_size = 12
        cv2.rectangle(img, 
                      (cx - box_size, cy - box_size), 
                      (cx + box_size, cy + box_size), 
                      (0, 0, 255), 2)
        cv2.putText(img, "WARNING", (cx - 70, cy + 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectorNode()
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