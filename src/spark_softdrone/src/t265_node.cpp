#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

class T265Node : public rclcpp::Node
{
public:
  T265Node()
  : Node("t265_node"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("imu", 50);

    rs2::config cfg;
    cfg.enable_device(T265_SERIAL);                   // set via a param
    cfg.enable_stream(RS2_STREAM_POSE);
    pipe_.start(cfg, std::bind(&T265Node::cb, this, std::placeholders::_1));  // async
  }

private:
  void cb(const rs2::frame &f)
  {
    auto pose_frame = f.as<rs2::pose_frame>();
    if (!pose_frame) return;
    auto data = pose_frame.get_pose_data();           // 200 Hz pose :contentReference[oaicite:1]{index=1}

    auto t = pose_frame.get_timestamp();
    rclcpp::Time stamp{static_cast<int64_t>(t*1e6)};  // µs→ns

    // Odometry ----------------------------------------------------------------
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";
    odom.pose.pose.position.x =  data.translation.x;
    odom.pose.pose.position.y =  data.translation.y;
    odom.pose.pose.position.z =  data.translation.z;
    odom.pose.pose.orientation.x = data.rotation.x;
    odom.pose.pose.orientation.y = data.rotation.y;
    odom.pose.pose.orientation.z = data.rotation.z;
    odom.pose.pose.orientation.w = data.rotation.w;
    odom.twist.twist.linear.x  = data.velocity.x;
    odom.twist.twist.angular.x = data.angular_velocity.x;
    odom_pub_->publish(odom);                               // basic Odometry publisher pattern :contentReference[oaicite:2]{index=2}

    // IMU ---------------------------------------------------------------------
    sensor_msgs::msg::Imu imu;
    imu.header = odom.header;
    imu.angular_velocity.x = data.angular_velocity.x;
    imu.linear_acceleration.x = data.acceleration.x;
    imu_pub_->publish(imu);

    // TF ----------------------------------------------------------------------
    geometry_msgs::msg::TransformStamped tf;
    tf.header = odom.header;
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rs2::pipeline pipe_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string T265_SERIAL = declare_parameter("serial", "121222110352");
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}