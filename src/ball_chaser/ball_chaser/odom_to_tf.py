# src/ball_chaser/ball_chaser/odom_to_tf.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_converter')
        
        # Create a TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to the odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.get_logger().info('Odom to TF converter node started.')

    def odom_callback(self, msg):
        # Create a TransformStamped message
        t = TransformStamped()

        # Read message content and assign it to corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id  # This should be 'odom'
        t.child_frame_id = msg.child_frame_id    # This should be 'base_footprint'

        # Pose from the odometry message
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()