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
        self.search_angular_speed = 0.3
        self.min_ball_area = 200        # Minimum contour area to consider as the ball
        self.max_frame_coverage = 0.25  # Ignore blobs covering more than 25% of the frame
        self.min_circle_radius = 4
        self.max_circle_radius_ratio = 0.45  # Allow the ball to take up nearly half the minimum image dimension
        self.center_tolerance_ratio = 0.08
        self.kernel = np.ones((5, 5), dtype=np.uint8)
        self.search_direction = 1.0
        self.missed_frames = 0
        self.search_activation_frames = 4
    
        self.get_logger().info('Ball Chaser Node Started')
    
    def image_callback(self, msg):
        """Process camera image and publish velocity commands"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            image_height, image_width = cv_image.shape[:2]
            
            # Convert to HSV and build a mask for bright/low-saturation pixels (white ball)
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            lower_white = np.array([0, 0, 205])
            upper_white = np.array([179, 45, 255])
            mask = cv2.inRange(hsv, lower_white, upper_white)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel, iterations=1)
            masked_gray = cv2.bitwise_and(gray, gray, mask=mask)
            blurred = cv2.GaussianBlur(masked_gray, (7, 7), 1.5)
            
            # Find contours of white objects
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Create Twist message
            twist = Twist()
            
            ball_detected = False
            object_center_x = None
            detected_radius = None
            detection_source = ''

            min_radius = self.min_circle_radius
            max_radius = int(max(min(image_width, image_height) * self.max_circle_radius_ratio, min_radius + 1))
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1.0,
                minDist=35,
                param1=60,
                param2=12,
                minRadius=min_radius,
                maxRadius=max_radius
            )

            if circles is not None and len(circles[0]) > 0:
                circles = np.round(circles[0]).astype(int)
                best_circle = max(circles, key=lambda c: c[2])
                cx, cy, radius = best_circle
                if radius >= min_radius:
                    object_center_x = int(np.clip(cx, 0, image_width - 1))
                    detected_radius = radius
                    ball_detected = True
                    detection_source = 'hough'

            if not ball_detected and len(contours) > 0:
                largest_contour = max(contours, key=cv2.contourArea)
                contour_area = cv2.contourArea(largest_contour)
                image_area = float(image_width * image_height)
                coverage = contour_area / image_area if image_area > 0 else 0.0
                x, y, w, h = cv2.boundingRect(largest_contour)
                width_ratio = w / float(image_width)
                height_ratio = h / float(image_height)
                center_y = y + h // 2

                if (
                    contour_area >= self.min_ball_area and
                    coverage <= self.max_frame_coverage and
                    width_ratio <= 0.35 and
                    height_ratio <= 0.35 and
                    center_y <= int(image_height * 0.85)
                ):
                    object_center_x = x + w // 2
                    detected_radius = int(max(w, h) / 2)
                    ball_detected = True
                    detection_source = 'contour'

            if ball_detected and object_center_x is not None:
                image_center_x = image_width // 2
                base_tolerance = image_width * self.center_tolerance_ratio
                dynamic_tolerance = detected_radius * 0.4 if detected_radius is not None else 0.0
                center_tolerance = max(base_tolerance, dynamic_tolerance)

                if object_center_x < image_center_x - center_tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                    self.search_direction = 1.0
                    self.get_logger().info(
                        f'Ball detected on LEFT via {detection_source} (x={object_center_x}) - Rotating left'
                    )
                elif object_center_x > image_center_x + center_tolerance:
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                    self.search_direction = -1.0
                    self.get_logger().info(
                        f'Ball detected on RIGHT via {detection_source} (x={object_center_x}) - Rotating right'
                    )
                else:
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                    self.get_logger().info(
                        f'Ball detected in CENTER via {detection_source} (x={object_center_x}/{image_center_x}) - Moving forward'
                    )
                self.missed_frames = 0
            else:
                twist.linear.x = 0.0
                self.missed_frames += 1
                if self.missed_frames >= self.search_activation_frames:
                    twist.angular.z = self.search_direction * self.search_angular_speed
                    self.get_logger().info('No ball detected - Searching', throttle_duration_sec=2.0)
                else:
                    twist.angular.z = 0.0
                    self.get_logger().info('No ball detected - Pausing', throttle_duration_sec=2.0)
            
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
