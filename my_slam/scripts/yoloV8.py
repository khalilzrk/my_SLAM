#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YOLOv8Detector(Node):
    def __init__(self):
        super().__init__('yolov8_detector')
        
        # Load YOLO model
        self.model = YOLO('yolov8n.pt')  # nano model for speed
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.detect, 10)
        
        # Publish annotated image
        self.image_pub = self.create_publisher(
            Image, '/yolo/image_annotated', 10)
        
        self.get_logger().info('YOLOv8 detector started!')
        
    def detect(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Run YOLO
        results = self.model(cv_image, conf=0.5)
        
        # Draw results on image
        annotated = results[0].plot()
        
        # Add detection count
        num_detections = len(results[0].boxes) if results[0].boxes else 0
        cv2.putText(annotated, f'Objects: {num_detections}', 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Publish annotated image
        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, 'bgr8'))

def main():
    rclpy.init()
    rclpy.spin(YOLOv8Detector())

if __name__ == '__main__':
    main()