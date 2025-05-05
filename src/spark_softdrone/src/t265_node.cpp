#include <cmath>                        // M_PI_2
#include <librealsense2/rs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

class T265Node : public rclcpp::Node
{
public:
  T265Node()
  : Node("t265_node"),
    tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(*this))
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/camera/pose/sample", 10);
    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/camera/imu", 50);

    rs2::config cfg;
    cfg.enable_device(declare_parameter("serial", "121222110352"));
    cfg.enable_stream(RS2_STREAM_POSE);
    pipe_.start(cfg, std::bind(&T265Node::cb, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
                "Started T265 node, publishing /camera/pose/sample and /camera/imu");
  }

private:
  /* RealSense (X‑right, Y‑up, Z‑back)  →  ROS‑ENU (X‑fwd, Y‑left, Z‑up)
   *   roll +90° about X  followed by  yaw −90° about Z
   *   (matrix  [[ 0  0 −1], [−1 0 0], [0 1 0]])
   */
  static const tf2::Quaternion& rs2ros_q()
  {
    static const tf2::Quaternion q = []{
      tf2::Quaternion tmp;
      tmp.setRPY( -M_PI_2, 0.0, -M_PI_2);   // roll, pitch, yaw
      tmp.normalize();
      return tmp;
    }();
    return q;
  }

  void cb(const rs2::frame &f)
  {
    auto pose_frame = f.as<rs2::pose_frame>();
    if (!pose_frame) return;
    const auto d = pose_frame.get_pose_data();

    const auto& C = rs2ros_q();             // device → ROS rotation

    /* pose ---------------------------------------------------------------- */
    tf2::Quaternion q_rs(d.rotation.x, d.rotation.y, d.rotation.z, d.rotation.w);
    tf2::Quaternion q_ros = C * q_rs;       q_ros.normalize();

    tf2::Vector3    p_rs(d.translation.x, d.translation.y, d.translation.z);
    tf2::Vector3    p_ros = tf2::quatRotate(C, p_rs);

    /* twist & acceleration ------------------------------------------------ */
    tf2::Vector3 v_ros = tf2::quatRotate(C, {d.velocity.x,
                                             d.velocity.y,
                                             d.velocity.z});
    tf2::Vector3 w_ros = tf2::quatRotate(C, {d.angular_velocity.x,
                                             d.angular_velocity.y,
                                             d.angular_velocity.z});
    tf2::Vector3 a_ros = tf2::quatRotate(C, {d.acceleration.x,
                                             d.acceleration.y,
                                             d.acceleration.z});

    rclcpp::Time stamp = get_clock()->now();

    /* Odometry ------------------------------------------------------------ */
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom_frame";
    odom.child_frame_id  = "camera_pose_frame";

    odom.pose.pose.position.x = p_ros.x();
    odom.pose.pose.position.y = p_ros.y();
    odom.pose.pose.position.z = p_ros.z();
    odom.pose.pose.orientation = tf2::toMsg(q_ros);

    odom.twist.twist.linear.x  = v_ros.x();
    odom.twist.twist.linear.y  = v_ros.y();
    odom.twist.twist.linear.z  = v_ros.z();

    odom.twist.twist.angular.x = w_ros.x();
    odom.twist.twist.angular.y = w_ros.y();
    odom.twist.twist.angular.z = w_ros.z();

    odom_pub_->publish(odom);

    /* IMU ----------------------------------------------------------------- */
    sensor_msgs::msg::Imu imu;
    imu.header            = odom.header;
    imu.orientation       = odom.pose.pose.orientation;

    imu.angular_velocity.x    = w_ros.x();
    imu.angular_velocity.y    = w_ros.y();
    imu.angular_velocity.z    = w_ros.z();

    imu.linear_acceleration.x = a_ros.x();
    imu.linear_acceleration.y = a_ros.y();
    imu.linear_acceleration.z = a_ros.z();

    imu_pub_->publish(imu);

    /* TF ------------------------------------------------------------------ */
    geometry_msgs::msg::TransformStamped tf;
    tf.header         = odom.header;
    tf.child_frame_id = "camera_pose_frame";
    tf.transform.translation.x = p_ros.x();
    tf.transform.translation.y = p_ros.y();
    tf.transform.translation.z = p_ros.z();
    tf.transform.rotation      = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rs2::pipeline pipe_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr    imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>         tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<T265Node>());
  rclcpp::shutdown();
  return 0;
}
