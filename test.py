#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

class OdomAutoReset(Node):
    def __init__(self):
        super().__init__('odom_auto_reset')
        self.subscription = self.create_subscription(
            Odometry, '/odom_icp_odometry', self.odom_callback, 10)
        self.reset_client = self.create_client(Empty, '/reset_odom')
        self.failure_count = 0
        self.max_failures = 3  # Reset after 3 consecutive lost messages
        
        self.get_logger().info(f'🔍 Auto-reset active (resets after {self.max_failures} failures)')
    
    def odom_callback(self, msg):
        # Odometry is lost when covariance is very high (9999)
        is_lost = msg.pose.covariance[0] > 1000.0
        
        if is_lost:
            self.failure_count += 1
            self.get_logger().warn(f'⚠️  Odometry LOST! ({self.failure_count}/{self.max_failures})')
            
            if self.failure_count >= self.max_failures:
                self.reset_odometry()
                self.failure_count = 0
        else:
            if self.failure_count > 0:
                self.get_logger().info('✅ Odometry recovered')
            self.failure_count = 0
    
    def reset_odometry(self):
        self.get_logger().info('🔄 AUTO-RESETTING ODOMETRY...')
        
        if not self.reset_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().error('❌ Reset service unavailable')
            return
        
        request = Empty.Request()
        future = self.reset_client.call_async(request)
        future.add_done_callback(lambda f: self.get_logger().info('✅ Reset complete!'))

def main():
    rclpy.init()
    node = OdomAutoReset()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



############################3
        # Node(
        #     package='rtabmap_odom',
        #     executable='icp_odometry',
        #     name='icp_odometry',
        #     output='screen',
        #     parameters=[{
        #         'frame_id': 'base_footprint_sync',  # Match odometry child_frame_id
        #         'odom_frame_id': 'odom_sync',
        #         'publish_tf': False,
        #         'scan_cloud_is_2d': True,
        #         'scan_range_min': 0.05,
        #         'scan_range_max': 20.0,
        #     }],
        #     remappings=[
        #         ('scan', '/scan_sync'),
        #         ('odom', '/odom_icp_odometry'),  # เพิ่มบรรทัดนี้
        #     ]
        # ),
        ############################3