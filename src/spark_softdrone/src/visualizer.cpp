#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>

class OdometryVisualizer : public rclcpp::Node
{
public:
    OdometryVisualizer()
    : Node("odometry_visualizer")
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&OdometryVisualizer::odomCallback, this, std::placeholders::_1));

        setpoint_sub_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos,
            std::bind(&OdometryVisualizer::setpointCallback, this, std::placeholders::_1));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("drone/odom", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("drone/setpoint_marker", 10);

        // Initialize marker for setpoint visualization
        setpoint_marker_.header.frame_id = "map";  // Frame of reference for visualization
        setpoint_marker_.ns = "setpoint";
        setpoint_marker_.id = 0;
        setpoint_marker_.type = visualization_msgs::msg::Marker::SPHERE;
        setpoint_marker_.action = visualization_msgs::msg::Marker::ADD;
        setpoint_marker_.scale.x = 0.3;
        setpoint_marker_.scale.y = 0.3;
        setpoint_marker_.scale.z = 0.3;
        setpoint_marker_.color.a = 1.0;
        setpoint_marker_.color.r = 1.0; // Red sphere for setpoint
        setpoint_marker_.color.g = 0.0;
        setpoint_marker_.color.b = 0.0;
    }

private:
    void odomCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        // Convert VehicleOdometry to nav_msgs/Odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "map"; // Fixed frame, can be "map" or "world"
        odom_msg.child_frame_id = "base_link"; // Drone's body frame

        // Position
        // NOTE: In PX4 NED frame, x=North, y=East, z=Down
        // For simplicity, assume "map" also uses NED (not standard ROS)
        // A proper conversion NED->ENU would require:
        // ENU: x_e = y_n, y_e = x_n, z_e = -z_n (if needed)
        odom_msg.pose.pose.position.x = msg->position[0];
        odom_msg.pose.pose.position.y = msg->position[1];
        odom_msg.pose.pose.position.z = msg->position[2];

        // Orientation
        // The quaternion provided by VehicleOdometry is from FRD to reference frame.
        // Make sure you understand the frame conversions for correct visualization.
        odom_msg.pose.pose.orientation.x = msg->q[1];
        odom_msg.pose.pose.orientation.y = msg->q[2];
        odom_msg.pose.pose.orientation.z = msg->q[3];
        odom_msg.pose.pose.orientation.w = msg->q[0];

        // Velocity
        // If velocity is in NED, and "map" is NED, just assign directly
        odom_msg.twist.twist.linear.x = msg->velocity[0];
        odom_msg.twist.twist.linear.y = msg->velocity[1];
        odom_msg.twist.twist.linear.z = msg->velocity[2];

        // Publish Odometry
        odom_pub_->publish(odom_msg);
    }

    void setpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
    {
        setpoint_marker_.header.stamp = this->get_clock()->now();
        setpoint_marker_.pose.position.x = msg->position[0];
        setpoint_marker_.pose.position.y = msg->position[1];
        setpoint_marker_.pose.position.z = msg->position[2];

        marker_pub_->publish(setpoint_marker_);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    visualization_msgs::msg::Marker setpoint_marker_;
};

int main(int argc, char *argv[])
{
    std::cout << "Starting odometry visualizer node..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometryVisualizer>());
    rclcpp::shutdown();
    return 0;
}
