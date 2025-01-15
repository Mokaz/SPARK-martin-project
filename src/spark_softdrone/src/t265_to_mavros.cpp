#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

class T265ToMavrosNode : public rclcpp::Node
{
public:
    T265ToMavrosNode()
    : Node("t265_to_mavros_node")
    {
        // Subscribe to T265 Odometry
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/camera/pose/sample",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            std::bind(&T265ToMavrosNode::odomCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos_profile(10);  // "history depth" of 10
        qos_profile
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);

        // Publishers for PoseWithCovarianceStamped and TwistWithCovarianceStamped
        pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mavros/vision_pose/pose_cov",
            qos_profile
        );
        pub_twist_cov_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/mavros/vision_speed/speed_twist_cov",
            qos_profile
        );

        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera/pose/sample");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header = odom_msg->header;
        pose_cov_msg.pose = odom_msg->pose;

        geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
        twist_cov_msg.header = odom_msg->header;
        twist_cov_msg.twist = odom_msg->twist;

        pub_pose_cov_->publish(pose_cov_msg);
        pub_twist_cov_->publish(twist_cov_msg);
    }

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<T265ToMavrosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
