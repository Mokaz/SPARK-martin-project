#include <memory>
#include <chrono>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h" 
#include "tf2_ros/buffer.h"             
#include "tf2/exceptions.h"              

using namespace std::chrono_literals;

class T265ToMapStaticTransformPublisher : public rclcpp::Node
{
public:
  explicit T265ToMapStaticTransformPublisher()
  : Node("t265_to_map_static_transform_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    max_retries_(10),
    retry_count_(0)
  {
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    timer_ = this->create_wall_timer(
      1s,  // Timer interval
      std::bind(&T265ToMapStaticTransformPublisher::timer_callback, this)
    );

    RCLCPP_INFO(this->get_logger(), "T265ToMapStaticTransformPublisher node has been started.");
  }

private:
  void timer_callback()
  {
    const std::string target_frame_ref = "base_link";
    const std::string source_frame_ref = "nav_cam_link";

    const std::string target_frame = "map";
    const std::string source_frame = "odom_frame";

    try {
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
        target_frame_ref,
        source_frame_ref,
        tf2::TimePointZero
      );

      transform_stamped.header.stamp = this->get_clock()->now();
      transform_stamped.header.frame_id = target_frame;
      transform_stamped.child_frame_id = source_frame;

      tf_static_broadcaster_->sendTransform(transform_stamped);

      RCLCPP_INFO(this->get_logger(),
        "Published static transform from '%s' to '%s'.",
        target_frame.c_str(),
        source_frame.c_str()
      );

      // Stop the timer when the transform has been successfully published
      timer_->cancel();

    } catch (const tf2::TransformException &ex) {
      // Handle exceptions (e.g., transform not available yet)
      RCLCPP_WARN(this->get_logger(),
        "Transform lookup failed: %s. Attempt %d of %d.",
        ex.what(),
        retry_count_ + 1,
        max_retries_
      );

      retry_count_++;
      if (retry_count_ >= max_retries_) {
        RCLCPP_ERROR(this->get_logger(),
          "Exceeded maximum number of retries (%d). Unable to publish static transform.",
          max_retries_
        );
        rclcpp::shutdown();
      }
    }
  }

  // Member variables
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  const int max_retries_;
  int retry_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265ToMapStaticTransformPublisher>());
  rclcpp::shutdown();
  return 0;
}
