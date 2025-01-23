#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp> // Adjust if your .msg differs
#include <tf2/LinearMath/Quaternion.h>

class NEDToENUTransformNode : public rclcpp::Node
{
public:
  NEDToENUTransformNode() : Node("ned_to_enu_transform_node")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    sub_local_pos_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position",
      10,
      std::bind(&NEDToENUTransformNode::localPositionCallback, this, std::placeholders::_1)
    );
  }

private:
  void localPositionCallback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    // 1) Extract NED position
    double xN = msg->x;   // North
    double yE = msg->y;   // East
    double zD = msg->z;   // Down

    // 2) Convert position from NED -> ENU
    double x_enu = yE;
    double y_enu = xN;
    double z_enu = -zD;  // because NED z is "down"

    // 3) Convert heading (NED) -> yaw (ENU)
    //
    //    heading (NED) is measured from north toward east
    //    yaw (ENU) is measured from +x (east) toward +y (north)
    //    relation: yaw_enu = pi/2 - heading_ned
    double heading_ned = msg->heading; // in radians, -PI..+PI
    double yaw_enu = M_PI/2.0 - heading_ned;

    // 4) Create an orientation quaternion: (roll=0, pitch=0, yaw=yaw_enu)
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_enu);
    q.normalize();

    // 5) Build the transform message
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();        // or msg->timestamp if synched
    transform.header.frame_id = "map";           // ENU world frame
    transform.child_frame_id = "base_link";      // must match your URDF base_link

    transform.transform.translation.x = x_enu;
    transform.transform.translation.y = y_enu;
    transform.transform.translation.z = z_enu;

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    // 6) Broadcast the transform
    tf_broadcaster_->sendTransform(transform);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr sub_local_pos_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NEDToENUTransformNode>());
  rclcpp::shutdown();
  return 0;
}
