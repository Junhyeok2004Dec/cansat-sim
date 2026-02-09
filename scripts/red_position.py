import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedObjectDetector(Node):
    def __init__(self):
        super().__init__('red_object_detector')
        
        self.subscription = self.create_subscription(
            Image,
            '/cansat/camera_down/image_raw',
            self.image_callback,
            10)
        self.subscription  
        
        self.bridge = CvBridge()
        self.get_logger().info('Red Object Detector Node Started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            blurred = cv2.GaussianBlur(cv_image, (11, 11), 0) #gaussian blur
            
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            
            
            lower_red1 = np.array([0, 70, 50])
            upper_red1 = np.array([10, 255, 255])
            
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 | mask2 
            
            
            #reduction noise            
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            
            # Contours
            contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                c = max(contours, key=cv2.contourArea)
                
                
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                M = cv2.moments(c)
                if M["m00"] > 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                    
                    self.get_logger().info(f'Detected Red Object at: (X: {center_x}, Y: {center_y})')
                    print(f'Red Obj at (X: {center_x}, Y: {center_y})')
                    
                    cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                    cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
                    
            cv2.imshow("Camera View", cv_image)
            cv2.imshow("Red Mask", mask)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    detector = RedObjectDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()