#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PX4LocalPositionTfBroadcaster : public rclcpp::Node
{
public:
  PX4LocalPositionTfBroadcaster() : Node("px4_local_position_tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.best_effort();        
    qos.durability_volatile();

    sub_local_pos_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/mavros/local_position/pose",
      qos,
      std::bind(&PX4LocalPositionTfBroadcaster::localPositionCallback, this, std::placeholders::_1)
    );

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();        
    transform.header.frame_id = "map";           
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
      this->get_logger(),
      "PX4 to TF node started"
    );
  }

private:
  void localPositionCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    double x_enu = msg->pose.position.x;  // North
    double y_enu = msg->pose.position.y;  // East
    double z_enu = msg->pose.position.z;  // Down

    tf2::Quaternion q_enu;
    tf2::fromMsg(msg->pose.orientation, q_enu);

    double roll_enu, pitch_enu, yaw_enu;
    tf2::Matrix3x3(q_enu).getRPY(roll_enu, pitch_enu, yaw_enu);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();        // or msg->header.stamp if synchronized
    transform.header.frame_id = "map";           // ENU world frame
    transform.child_frame_id = "base_link";      // must match your URDF base_link

    transform.transform.translation.x = x_enu;
    transform.transform.translation.y = y_enu;
    transform.transform.translation.z = z_enu;

    transform.transform.rotation = tf2::toMsg(q_enu);

    tf_broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
      this->get_logger(),
      "ENU: x=%.5f, y=%.5f, z=%.5f, yaw=%.5f",
      x_enu, y_enu, z_enu, yaw_enu
    );
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_local_pos_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4LocalPositionTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
