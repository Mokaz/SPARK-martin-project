#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class T265ToMavrosNode : public rclcpp::Node
{
public:
    T265ToMavrosNode()
    : Node("t265_to_mavros_node"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
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
        // Lookup transform from nav_cam_link to fcu_link
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform(
                "fcu_link",         // target
                "nav_cam_link",     // source
                tf2::TimePointZero
            );
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // Transform pose
        geometry_msgs::msg::PoseStamped pose_in, pose_out;
        pose_in.header = odom_msg->header;
        pose_in.pose   = odom_msg->pose.pose;
        tf2::doTransform(pose_in, pose_out, transform_stamped);

        // Transform twist (only rotates vectors)
        geometry_msgs::msg::Vector3Stamped lin_in, lin_out;
        geometry_msgs::msg::Vector3Stamped ang_in, ang_out;
        lin_in.header = odom_msg->header;
        lin_in.vector = odom_msg->twist.twist.linear;
        ang_in.header = odom_msg->header;
        ang_in.vector = odom_msg->twist.twist.angular;
        tf2::doTransform(lin_in, lin_out, transform_stamped);
        tf2::doTransform(ang_in, ang_out, transform_stamped);

        // Fill transformed messages
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header           = pose_out.header;
        pose_cov_msg.pose.pose        = pose_out.pose;
        pose_cov_msg.pose.covariance  = odom_msg->pose.covariance;

        geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
        twist_cov_msg.header                  = pose_out.header;
        twist_cov_msg.twist.twist.linear      = lin_out.vector;
        twist_cov_msg.twist.twist.angular     = ang_out.vector;
        twist_cov_msg.twist.covariance        = odom_msg->twist.covariance;

        pub_pose_cov_->publish(pose_cov_msg);
        pub_twist_cov_->publish(twist_cov_msg);
    }

    // Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;

    // TF2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<T265ToMavrosNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
