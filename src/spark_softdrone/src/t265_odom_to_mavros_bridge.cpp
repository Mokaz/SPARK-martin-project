#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class T265OdomToMavrosBridge : public rclcpp::Node
{
public:
    T265OdomToMavrosBridge()
    : Node("t265_odom_to_mavros_bridge"),
      tf_buffer_(this->get_clock()),
      tf_listener_(tf_buffer_)
    {
        // Retrieve static transforms once during initialization
        getStaticTransforms();

        // Subscriber and publishers remain the same as before
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/camera/pose/sample",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            std::bind(&T265OdomToMavrosBridge::odomCallback, this, std::placeholders::_1)
        );

        rclcpp::QoS qos_profile(10);
        qos_profile
            .reliability(rclcpp::ReliabilityPolicy::Reliable)
            .durability(rclcpp::DurabilityPolicy::Volatile);

        pub_pose_cov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mavros/vision_pose/pose_cov",
            qos_profile
        );
        pub_twist_cov_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/mavros/vision_speed/speed_twist_cov",
            qos_profile
        );
    }

private:
    void getStaticTransforms() {
        bool transforms_retrieved = false;
        while (rclcpp::ok() && !transforms_retrieved) {
            try {
                // Lookup static transforms (block until they are available)
                map_to_odom_ = tf_buffer_.lookupTransform("map", "odom_frame", tf2::TimePointZero);
                cam_to_fcu_ = tf_buffer_.lookupTransform("nav_cam_link_tilted", "fcu_link", tf2::TimePointZero);
                
                // Convert to tf2::Transform for efficient reuse
                tf2::fromMsg(map_to_odom_.transform, map_T_odom_);
                tf2::fromMsg(cam_to_fcu_.transform, cam_T_fcu_);
                
                // Extract rotation components for twist transformation
                map_T_odom_rot_.setRotation(map_T_odom_.getRotation());
                cam_T_fcu_rot_.setRotation(cam_T_fcu_.getRotation());
                
                transforms_retrieved = true;
                RCLCPP_INFO(this->get_logger(), "Static transforms retrieved.");
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Waiting for static transforms: %s", ex.what());
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        // Convert odometry pose to a transform (odom_T_cam)
        tf2::Transform odom_T_cam;
        tf2::fromMsg(odom_msg->pose.pose, odom_T_cam);

        // Compute map_T_fcu = map_T_odom * odom_T_cam * cam_T_fcu
        tf2::Transform map_T_fcu = map_T_odom_ * odom_T_cam * cam_T_fcu_;

        // Convert to PoseStamped
        geometry_msgs::msg::PoseStamped pose_out;
        pose_out.header.stamp = odom_msg->header.stamp;
        pose_out.header.frame_id = "map";
        tf2::toMsg(map_T_fcu, pose_out.pose);

        // Transform twist (rotate linear and angular velocities)
        tf2::Vector3 lin_vel, ang_vel;
        tf2::fromMsg(odom_msg->twist.twist.linear, lin_vel);
        tf2::fromMsg(odom_msg->twist.twist.angular, ang_vel);

        lin_vel = map_T_odom_rot_ * lin_vel; // Rotate by map to odom
        ang_vel = map_T_odom_rot_ * ang_vel;
        lin_vel = cam_T_fcu_rot_ * lin_vel;  // Rotate by cam to fcu
        ang_vel = cam_T_fcu_rot_ * ang_vel;

        // Publish messages (same as before)
        geometry_msgs::msg::PoseWithCovarianceStamped pose_cov_msg;
        pose_cov_msg.header = pose_out.header;
        pose_cov_msg.pose.pose = pose_out.pose;
        pose_cov_msg.pose.covariance = odom_msg->pose.covariance;

        geometry_msgs::msg::TwistWithCovarianceStamped twist_cov_msg;
        twist_cov_msg.header.stamp = odom_msg->header.stamp;
        twist_cov_msg.header.frame_id = "map";
        twist_cov_msg.twist.twist.linear = tf2::toMsg(lin_vel);
        twist_cov_msg.twist.twist.angular = tf2::toMsg(ang_vel);
        twist_cov_msg.twist.covariance = odom_msg->twist.covariance;

        pub_pose_cov_->publish(pose_cov_msg);
        pub_twist_cov_->publish(twist_cov_msg);
    }

    // Subscriber and publishers (same as before)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_cov_;
    rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr pub_twist_cov_;

    // TF2 objects
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Static transforms (retrieved once)
    geometry_msgs::msg::TransformStamped map_to_odom_, cam_to_fcu_;
    tf2::Transform map_T_odom_, cam_T_fcu_;
    tf2::Transform map_T_odom_rot_, cam_T_fcu_rot_; // Rotation components only
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<T265OdomToMavrosBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}