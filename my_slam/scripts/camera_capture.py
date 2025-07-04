#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        
        self.bridge = CvBridge()
        
        # Subscribe to YOUR remapped topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Your remapped topic
            self.image_callback,
            10
        )
        
        # Subscribe to camera info if you add it
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.info_callback,
            10
        )
        
        self.get_logger().info('Camera test started. Waiting for images on /camera/image_raw...')
        self.first_image = True
        
    def image_callback(self, msg):
        try:
            # Convert to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            if self.first_image:
                self.get_logger().info(f'Receiving images! Shape: {cv_image.shape}')
                self.first_image = False
            
            # Add some text overlay
            cv2.putText(cv_image, 'Camera Test', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show the image
            cv2.imshow('My Robot Camera', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
    
    def info_callback(self, msg):
        self.get_logger().info(f'Camera info: {msg.width}x{msg.height}', once=True)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()