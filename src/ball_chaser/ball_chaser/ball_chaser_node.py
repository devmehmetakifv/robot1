#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class BallChaserNode(Node):
    def __init__(self):
        super().__init__('ball_chaser')
        
        # Create publisher for cmd_vel commands (per assignment spec)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create subscriber for camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # CV Bridge for converting ROS images to OpenCV
        self.bridge = CvBridge()
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.white_threshold = 200  # Threshold for detecting white color
        
        self.get_logger().info('Ball Chaser Node Started')
    
    def image_callback(self, msg):
        """Process camera image and publish velocity commands"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Threshold to detect white pixels
            _, thresh = cv2.threshold(gray, self.white_threshold, 255, cv2.THRESH_BINARY)
            
            # Find contours of white objects
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create Twist message
            twist = Twist()
            
            if len(contours) > 0:
                # Find the largest contour (assuming it's the ball)
                largest_contour = max(contours, key=cv2.contourArea)
                
                # Get the bounding box of the largest contour
                x, y, w, h = cv2.boundingRect(largest_contour)
                
                # Calculate center of the detected object
                object_center_x = x + w // 2
                
                # Image dimensions
                image_height, image_width = cv_image.shape[:2]
                image_center_x = image_width // 2
                
                # Define center region (10% tolerance on each side)
                center_tolerance = image_width * 0.1
                
                # Determine robot action based on ball position
                if object_center_x < image_center_x - center_tolerance:
                    # Ball is on the left - turn left
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                    self.get_logger().info('Ball detected on LEFT - Turning left')
                    
                elif object_center_x > image_center_x + center_tolerance:
                    # Ball is on the right - turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                    self.get_logger().info('Ball detected on RIGHT - Turning right')
                    
                else:
                    # Ball is in the center - move forward
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                    self.get_logger().info('Ball detected in CENTER - Moving forward')
            
            else:
                # No ball detected - stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('No ball detected - Stopping', throttle_duration_sec=2.0)
            
            # Publish velocity command
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = BallChaserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
