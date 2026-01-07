#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>




class SyncRewriteNode : public rclcpp::Node
{
public:
    SyncRewriteNode() : Node("sync_rewrite_sensors_and_tf_node")
    {
        // ===============================
        // ROS pubs Qos static
        // ===============================
        // auto qos_static = rclcpp::QoS(1).transient_local();

        // ===============================
        // Subscribers (STORE ONLY)
        // ===============================
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SyncRewriteNode::scan_callback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw_self", 10,
            std::bind(&SyncRewriteNode::imu_callback, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SyncRewriteNode::joint_callback, this, std::placeholders::_1));        

        // ===============================
        // Publishers (SYNC OUTPUT)
        // ===============================
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_sync", 10);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_raw_sync", 10);
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states_sync", 10);
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // tf_static_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos_static);

        // ===============================
        // Publishers STATIC TF
        // ===============================
        publish_static_tf();

        RCLCPP_INFO(this->get_logger(), "Sync rewrite TF + TF_STATIC node started");
    }

private:
    // ==================================================
    // CALLBACKS (STORE ONLY)
    // ==================================================
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // ------------------------------ scan ------------------------------ //
        auto scan = *msg;
        scan.header.stamp = this->get_clock()->now();
        scan.header.frame_id = "laser_frame_sync";
        scan_pub_->publish(scan);
        // ------------------------------ scan ------------------------------ //
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // ------------------------------ imu ------------------------------ //
        auto imu = *msg;
        imu.header.stamp = this->get_clock()->now();
        imu.header.frame_id = "imu_frame_sync";

        imu_pub_->publish(imu);
        // ------------------------------ imu ------------------------------ //
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Empty callback - not implemented
        (void)msg;
    }

    void publish_static_tf()
    {
        // tf2_msgs::msg::TFMessage tf_message;
        std::vector<geometry_msgs::msg::TransformStamped> tfs;
        auto now = this->get_clock()->now();

        // --------------------------------------------------
        // base_footprint -> base_link
        // --------------------------------------------------
        geometry_msgs::msg::TransformStamped tf1;
        tf1.header.stamp = now;
        tf1.header.frame_id = "base_footprint_sync";
        tf1.child_frame_id = "base_link_sync";
        tf1.transform.translation.x = 0.0;
        tf1.transform.translation.y = 0.0;
        tf1.transform.translation.z = 0.01;
        tf1.transform.rotation.x = 0.0;
        tf1.transform.rotation.y = 0.0;
        tf1.transform.rotation.z = 0.0;
        tf1.transform.rotation.w = 1.0;
        tfs.push_back(tf1);

        // --------------------------------------------------
        // base_link -> laser_frame
        // --------------------------------------------------
        geometry_msgs::msg::TransformStamped tf2;
        tf2.header.stamp = now;
        tf2.header.frame_id = "base_link_sync";
        tf2.child_frame_id = "laser_frame_sync";
        tf2.transform.translation.x = 0.0;
        tf2.transform.translation.y = 0.0;
        tf2.transform.translation.z = 0.18;
        tf2.transform.rotation.x = 0.0;
        tf2.transform.rotation.y = 0.0;
        tf2.transform.rotation.z = 0.0;
        tf2.transform.rotation.w = 1.0;
        tfs.push_back(tf2);

        // --------------------------------------------------
        // base_link -> imu_link
        // --------------------------------------------------
        geometry_msgs::msg::TransformStamped tf3;
        tf3.header.stamp = now;
        tf3.header.frame_id = "base_link_sync";
        tf3.child_frame_id = "imu_frame_sync";
        tf3.transform.translation.x = 0.085;
        tf3.transform.translation.y = 4.4164e-05;
        tf3.transform.translation.z = 0.07;
        tf3.transform.rotation.x = 0.0;
        tf3.transform.rotation.y = 0.0;
        tf3.transform.rotation.z = 0.0;
        tf3.transform.rotation.w = 1.0;
        tfs.push_back(tf3);

        // --------------------------------------------------
        // Publish once (static)
        // --------------------------------------------------
        static_broadcaster_->sendTransform(tfs);
        // tf_static_pub_->publish(tf_message);
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
    // rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_pub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SyncRewriteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
