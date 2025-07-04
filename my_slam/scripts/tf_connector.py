#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped

class TFConnector(Node):
    def __init__(self):
        super().__init__('tf_connector')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.create_timer(0.1, self.publish_transform)
        
    def publish_transform(self):
        try:
            # Get transform from my_robot/odom to my_robot/base_footprint
            trans = self.tf_buffer.lookup_transform(
                'my_robot/odom', 'my_robot/base_footprint', 
                rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Republish as odom to base_footprint
            new_trans = TransformStamped()
            new_trans.header = trans.header
            new_trans.header.frame_id = 'odom'
            new_trans.child_frame_id = 'base_footprint'
            new_trans.transform = trans.transform
            
            self.tf_broadcaster.sendTransform(new_trans)
            
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = TFConnector()
    rclpy.spin(node)

if __name__ == '__main__':
    main()