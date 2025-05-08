#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

#include <librealsense2/rs.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace std::chrono_literals;

class T265Node : public rclcpp::Node
{
public:
  T265Node()
  : Node("t265_node"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  {
    // Configure T265 --------------------------------------------------------
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL);
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_POSE);
    pipe_.start(cfg);

    // Publishers ------------------------------------------------------------
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("rs_t265/odom", 10);
    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>  ("rs_t265/imu",  10);

    // 100 Hz timer ----------------------------------------------------------
    timer_ = create_wall_timer(10ms,
              std::bind(&T265Node::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "Started T265 node - publishing rs_t265/odom and rs_t265/imu");
  }

private:
  // -------------------------  periodic work  ------------------------------
  void timerCallback()
  {
    rs2::frameset fs = pipe_.wait_for_frames();
    publishImu(fs);
    publishOdomAndTf(fs);
  }

  void publishImu(const rs2::frameset &fs)
  {
    if (auto gyro = fs.first_or_default(RS2_STREAM_GYRO))
    {
      auto g = gyro.as<rs2::motion_frame>().get_motion_data();
      imu_msg_.angular_velocity.x = g.x;
      imu_msg_.angular_velocity.y = g.y;
      imu_msg_.angular_velocity.z = g.z;
    }

    if (auto accel = fs.first_or_default(RS2_STREAM_ACCEL))
    {
      auto a = accel.as<rs2::motion_frame>().get_motion_data();

      imu_msg_.header.stamp          = now();
      imu_msg_.header.frame_id       = "t265_frame";
      imu_msg_.linear_acceleration.x = a.x;
      imu_msg_.linear_acceleration.y = a.y;
      imu_msg_.linear_acceleration.z = a.z;

      imu_pub_->publish(imu_msg_);
    }
  }

  void publishOdomAndTf(const rs2::frameset &fs)
  {
    auto pose_f = fs.first_or_default(RS2_STREAM_POSE);
    if (!pose_f) return;

    const rs2_pose p = pose_f.as<rs2::pose_frame>().get_pose_data();

    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now();
    odom.header.frame_id = "odom_frame";
    odom.child_frame_id  = "t265_frame";

    odom.pose.pose.position.x = -p.translation.z;
    odom.pose.pose.position.y = -p.translation.x;
    odom.pose.pose.position.z =  p.translation.y;

    odom.pose.pose.orientation.x = -p.rotation.z;
    odom.pose.pose.orientation.y = -p.rotation.x;
    odom.pose.pose.orientation.z =  p.rotation.y;
    odom.pose.pose.orientation.w =  p.rotation.w;

    // Rotate velocity from camera into odom frame
    double r, pitch, yaw;
    tf2::Quaternion q(odom.pose.pose.orientation.x,
                      odom.pose.pose.orientation.y,
                      odom.pose.pose.orientation.z,
                      odom.pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(r, pitch, yaw);

    const double vx_cam = -p.velocity.z;
    const double vy_cam = -p.velocity.x;

    odom.twist.twist.linear.x =  vx_cam * std::cos(yaw) + vy_cam * std::sin(yaw);
    odom.twist.twist.linear.y = -vx_cam * std::sin(yaw) + vy_cam * std::cos(yaw);
    odom.twist.twist.linear.z =  p.velocity.y;

    odom.twist.twist.angular.x = -p.angular_velocity.z;
    odom.twist.twist.angular.y = -p.angular_velocity.x;
    odom.twist.twist.angular.z =  p.angular_velocity.y;

    odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header         = odom.header;
    tf.child_frame_id = odom.child_frame_id;
    tf.transform.translation.x = odom.pose.pose.position.x;
    tf.transform.translation.y = odom.pose.pose.position.y;
    tf.transform.translation.z = odom.pose.pose.position.z;
    tf.transform.rotation      = odom.pose.pose.orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  // ------------------------------  members  -------------------------------
  rs2::pipeline pipe_;

  sensor_msgs::msg::Imu                         imu_msg_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr   imu_pub_;
  rclcpp::TimerBase::SharedPtr                          timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}
