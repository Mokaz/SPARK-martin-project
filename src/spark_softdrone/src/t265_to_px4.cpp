#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>
#include <Eigen/Dense>
#include <memory>

class T265ToPX4Node : public rclcpp::Node
{
public:
    T265ToPX4Node()
    : Node("t265_to_px4_node")
    {
        // Initialize the Local Position Measurement Interface with LocalNED frame
        px4_ros2::PoseFrame pose_frame = px4_ros2::PoseFrame::LocalNED;
        px4_ros2::VelocityFrame velocity_frame = px4_ros2::VelocityFrame::LocalNED;

        try {
            local_pos_interface_ = std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
                *this, pose_frame, velocity_frame);
            RCLCPP_INFO(this->get_logger(), "LocalPositionMeasurementInterface initialized.");
        } catch (const std::exception &e) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize LocalPositionMeasurementInterface: %s", e.what());
            throw;
        }

        // Subscribe to T265 Odometry
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/camera/pose/sample",
            rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)).best_effort(),
            std::bind(&T265ToPX4Node::odomCallback, this, std::placeholders::_1)
        );


        RCLCPP_INFO(this->get_logger(), "Subscribed to /camera/pose/sample");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Extract pose
        double x_cam = msg->pose.pose.position.x;
        double y_cam = msg->pose.pose.position.y;
        double z_cam = msg->pose.pose.position.z;

        double qx_cam = msg->pose.pose.orientation.x;
        double qy_cam = msg->pose.pose.orientation.y;
        double qz_cam = msg->pose.pose.orientation.z;
        double qw_cam = msg->pose.pose.orientation.w;

        // Extract velocity
        double vx_cam = msg->twist.twist.linear.x;
        double vy_cam = msg->twist.twist.linear.y;
        double vz_cam = msg->twist.twist.linear.z;

        // Transform pose from T265 frame (Camera-Centric) to PX4 LocalNED
        Eigen::Vector3f pos_ned = transformCamToNED(x_cam, y_cam, z_cam);
        Eigen::Quaternionf q_ned = transformOrientationCamToNED(qx_cam, qy_cam, qz_cam, qw_cam);

        // Transform velocity
        Eigen::Vector3f vel_ned = transformVelocityCamToNED(vx_cam, vy_cam, vz_cam);

        // Populate LocalPositionMeasurement
        px4_ros2::LocalPositionMeasurement measurement{};
        measurement.timestamp_sample = rclcpp::Time(msg->header.stamp);

        // Position
        // --- Extract position variances from pose.covariance (diagonal) ---
        float var_x = msg->pose.covariance[0];   // C_{xx}
        float var_y = msg->pose.covariance[7];   // C_{yy}
        float var_z = msg->pose.covariance[14];  // C_{zz}

        measurement.position_xy = Eigen::Vector2f(pos_ned.x(), pos_ned.y());
        measurement.position_z = pos_ned.z();
        measurement.position_xy_variance = Eigen::Vector2f(var_x, var_y);
        measurement.position_z_variance  = var_z;

        // Velocity
        // --- Extract velocity variances from twist.covariance (diagonal) ---
        float var_vx = msg->twist.covariance[0];   // C_{vxvx}
        float var_vy = msg->twist.covariance[7];   // C_{vyvy}
        float var_vz = msg->twist.covariance[14];  // C_{vzvz}

        measurement.velocity_xy = Eigen::Vector2f(vel_ned.x(), vel_ned.y());
        measurement.velocity_xy_variance = Eigen::Vector2f(var_vx, var_vy);
        measurement.velocity_z = vel_ned.z();
        measurement.velocity_z_variance = var_vz;

        // Optional: Attitude
        // --- Extract orientation variance (roll, pitch, yaw) from pose.covariance ---
        // For a full quaternion, you might approximate or just fill yaw variance
        float var_roll = msg->pose.covariance[21];   // C_{φφ}
        float var_pitch = msg->pose.covariance[28];  // C_{θθ}
        float var_yaw = msg->pose.covariance[35];    // C_{ψψ}

        measurement.attitude_quaternion = q_ned;
        measurement.attitude_variance = Eigen::Vector3f(var_roll, var_pitch, var_yaw);

        // Send measurement to PX4
        try {
            local_pos_interface_->update(measurement);
            RCLCPP_DEBUG(this->get_logger(), "Sent LocalPositionMeasurement to PX4.");
        } catch (const px4_ros2::NavigationInterfaceInvalidArgument &e) {
            RCLCPP_ERROR(this->get_logger(), "Invalid measurement: %s", e.what());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Exception while sending measurement: %s", e.what());
        }
    }

    // Transformation functions from T265 Camera Frame to PX4 LocalNED

    // Example transformation: Camera (X forward, Y left, Z up) to NED (X north, Y east, Z down)
    Eigen::Vector3f transformCamToNED(double x, double y, double z)
    {
        // Define rotation: 180 degrees around Y-axis, then 90 degrees around Z-axis
        // TODO: Adjust based on your actual mounting orientation

        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

        Eigen::Vector3f cam_vec(x, y, z);
        Eigen::Vector3f ned_vec = rotation * cam_vec;

        return ned_vec;
    }

    Eigen::Quaternionf transformOrientationCamToNED(double qx, double qy, double qz, double qw)
    {
        // Convert quaternion to Eigen
        Eigen::Quaternionf q_cam(qw, qx, qy, qz);

        // Define the same rotation as in position
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

        Eigen::Quaternionf q_rotation(rotation);

        // Apply rotation to orientation
        Eigen::Quaternionf q_ned = q_rotation * q_cam;

        return q_ned;
    }

    Eigen::Vector3f transformVelocityCamToNED(double vx, double vy, double vz)
    {
        // Use the same rotation as position
        Eigen::Matrix3f rotation;
        rotation = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) *
                   Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitZ());

        Eigen::Vector3f cam_vel(vx, vy, vz);
        Eigen::Vector3f ned_vel = rotation * cam_vel;

        return ned_vel;
    }

    // ROS 2 subscription and PX4 interface
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> local_pos_interface_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<T265ToPX4Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
