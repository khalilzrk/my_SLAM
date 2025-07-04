#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanFrameFixer(Node):
    def __init__(self):
        super().__init__('scan_frame_fixer')
        input_topic = self.declare_parameter('scan_in', '/scan').get_parameter_value().string_value
        output_topic = self.declare_parameter('scan_out', '/scan_fixed').get_parameter_value().string_value
        self.sub = self.create_subscription(
            LaserScan, input_topic, self.callback, 10)
        self.pub = self.create_publisher(
            LaserScan, output_topic, 10)
        self.get_logger().info(f'Scan frame fixer started: {input_topic} → {output_topic}')
        
    def callback(self, msg):
        msg.header.frame_id = 'lidar_link'  # Fix the frame_id
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ScanFrameFixer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()