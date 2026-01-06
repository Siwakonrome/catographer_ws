#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu, JointState






class SyncRewriteNode(Node):

    def __init__(self):
        super().__init__('sync_rewrite_sensors_and_tf_node')

        # ===============================
        # ROS pubs Qos static
        # ===============================
        qos_static = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        # ===============================
        # Internal buffers (latest)
        # ===============================
        self.latest_scan = None
        self.latest_imu = None
        self.latest_joint = None

        # ===============================
        # Subscribers (STORE ONLY)
        # ===============================
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw_self', self.imu_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # ===============================
        # Publishers (SYNC OUTPUT)
        # ===============================
        self.scan_pub = self.create_publisher(LaserScan, '/scan_sync', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu_sync', 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states_sync', 10)
        self.tf_static_pub = self.create_publisher(TFMessage, '/tf_static', qos_static)

        # ===============================
        # Publishers STATIC TF
        # ===============================
        self.publish_static_tf()

        # ===============================
        # Single Timer Loop (30 Hz)
        # ===============================
        self.timer = self.create_timer(1.0 / 30.0, self.timer_cb)
        self.get_logger().info('Sync rewrite TF + TF_STATIC node started')




    # ==================================================
    # CALLBACKS (STORE ONLY)
    # ==================================================
    def scan_cb(self, msg): self.latest_scan = msg
    def imu_cb(self, msg): self.latest_imu = msg
    def joint_cb(self, msg): self.latest_joint = msg

    

    def publish_static_tf(self):
        tf_list = []
        now = self.get_clock().now().to_msg()

        # --------------------------------------------------
        # base_footprint -> base_link
        # --------------------------------------------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_footprint_sync"
        tf.child_frame_id = "base_link_sync"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.01
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        tf_list.append(tf)

        # --------------------------------------------------
        # base_link -> laser_frame
        # --------------------------------------------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_link_sync"
        tf.child_frame_id = "laser_frame_sync"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.18
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        tf_list.append(tf)

        # --------------------------------------------------
        # base_link -> imu_link
        # --------------------------------------------------
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = "base_link_sync"
        tf.child_frame_id = "imu_link_sync"
        tf.transform.translation.x = 0.085
        tf.transform.translation.y = 4.4164e-05
        tf.transform.translation.z = 0.07
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 1.0
        tf_list.append(tf)

        # --------------------------------------------------
        # Publish once (static)
        # --------------------------------------------------
        self.tf_static_pub.publish(TFMessage(transforms=tf_list))



    # ==================================================
    # SINGLE TIMER LOOP
    # ==================================================
    def timer_cb(self):

        if (self.latest_scan   is None or
            self.latest_imu    is None or
            self.latest_joint  is None 
        ): 
            return
        

        stamp = self.get_clock().now().to_msg()

        # ------------------------------ scan ------------------------------ #
        scan : LaserScan = self.latest_scan
        scan.header.stamp = stamp
        scan.header.frame_id = "laser_frame_sync"
        self.scan_pub.publish(scan)
        # ------------------------------ scan ------------------------------ #


        # ------------------------------ imu ------------------------------ #
        imu : Imu = self.latest_imu
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_frame_sync"
        self.imu_pub.publish(imu)
        # ------------------------------ imu ------------------------------ #



        

        
        


def main(args=None):
    rclpy.init(args=args)
    node = SyncRewriteNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()