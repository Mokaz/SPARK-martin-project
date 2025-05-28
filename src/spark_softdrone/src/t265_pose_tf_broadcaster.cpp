#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class T265PoseTFBroadcaster : public rclcpp::Node
{
public:
  T265PoseTFBroadcaster()
  : Node("t265_pose_tf_broadcaster"),
    tf_broadcaster_(this)
  {
    // QoS profile mirroring the publisher settings
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/mavros/vision_pose/pose_cov",
      qos_profile,
      std::bind(&T265PoseTFBroadcaster::poseCallback, this, std::placeholders::_1));

      RCLCPP_INFO(
        this->get_logger(),
        "T265PoseTFBroadcaster node started"
      );
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped transform;

    // Populate the transform message
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "map";      // Parent frame
    transform.child_frame_id = "base_link"; // Child frame

    transform.transform.translation.x = msg->pose.pose.position.x;
    transform.transform.translation.y = msg->pose.pose.position.y;
    transform.transform.translation.z = msg->pose.pose.position.z;
    transform.transform.rotation = msg->pose.pose.orientation;

    // Broadcast the transform
    tf_broadcaster_.sendTransform(transform);
  }

  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265PoseTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
