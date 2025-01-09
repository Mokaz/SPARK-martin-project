/****************************************************************************
 *
 * Copyright 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example with dynamic setpoint updates
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author ...
 */

// COMMAND TO PUBLISH SETPOINT:
// ros2 topic pub /fmu/my_new_setpoints px4_msgs/msg/TrajectorySetpoint "{position: [1.0, 2.0, -3.0], yaw: 1.57}"

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <string>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	explicit OffboardControl(std::string px4_namespace) :
		Node("offboard_control_srv"),
		state_{State::init},
		service_result_{0},
		service_done_{false},
		offboard_control_mode_publisher_{
			this->create_publisher<OffboardControlMode>(px4_namespace + "in/offboard_control_mode", 10)},
		trajectory_setpoint_publisher_{
			this->create_publisher<TrajectorySetpoint>(px4_namespace + "in/trajectory_setpoint", 10)},
		vehicle_command_client_{
			this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace + "vehicle_command")}
	{
		RCLCPP_INFO(this->get_logger(), "Starting Offboard Control example with PX4 services");
		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
		}

		// Subscriber for updated setpoints
		setpoint_subscriber_ = this->create_subscription<TrajectorySetpoint>(
			px4_namespace + "my_new_setpoints", 10,
			std::bind(&OffboardControl::setpoint_callback, this, std::placeholders::_1)
		);

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));
	}

	void switch_to_offboard_mode();
	void arm();
	void disarm();

private:
	enum class State {
		init,
		offboard_requested,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed
	} state_;

	uint8_t service_result_;
	bool service_done_;
	rclcpp::TimerBase::SharedPtr timer_;

	// Publishers
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;

	// Client
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	// Subscriber for dynamic setpoints
	rclcpp::Subscription<TrajectorySetpoint>::SharedPtr setpoint_subscriber_;

	// Variables to hold the current desired position/yaw
	float desired_x_{0.0};
	float desired_y_{0.0};
	float desired_z_{-5.0};
	float desired_yaw_{-3.14};

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void request_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void timer_callback();

	// Callback to update the desired setpoint
	void setpoint_callback(const TrajectorySetpoint::SharedPtr msg);
};

/**
 * @brief Send a command to switch to offboard mode
 */
void OffboardControl::switch_to_offboard_mode(){
	RCLCPP_INFO(this->get_logger(), "Requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "Requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "Requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        Uses the current desired_x_, desired_y_, desired_z_, and desired_yaw_.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	TrajectorySetpoint msg{};
	msg.position = {desired_x_, desired_y_, desired_z_};
	msg.yaw = desired_yaw_; // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(
		request, std::bind(&OffboardControl::response_callback, this, std::placeholders::_1)
	);
	RCLCPP_INFO(this->get_logger(), "Command sent");
}

/**
 * @brief Timer callback. Publishes offboard mode, setpoint, and checks if we
 *        can switch to offboard/arm.
 */
void OffboardControl::timer_callback()
{
	static uint8_t num_of_steps = 0;

	// OffboardControlMode and TrajectorySetpoint must be published at the same rate
	publish_offboard_control_mode();
	publish_trajectory_setpoint();

	switch (state_) {
	case State::init :
		switch_to_offboard_mode();
		state_ = State::offboard_requested;
		break;

	case State::offboard_requested :
		if (service_done_) {
			if (service_result_ == 0) {
				RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
				state_ = State::wait_for_stable_offboard_mode;				
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;

	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps > 10) {
			arm();
			state_ = State::arm_requested;
		}
		break;

	case State::arm_requested :
		if (service_done_) {
			if (service_result_ == 0) {
				RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
				state_ = State::armed;
			} else {
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;

	case State::armed:
		// Once armed, you can stay in offboard and update setpoints in real-time!
		break;
	}
}

/**
 * @brief Callback for VehicleCommand service response
 */
void OffboardControl::response_callback(
	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future)
{
	auto status = future.wait_for(1s);
	if (status == std::future_status::ready) {
		auto reply = future.get()->reply;
		service_result_ = reply.result;
		switch (service_result_) {
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "Command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "Command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "Command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "Command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "Command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "Command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "Command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "Command reply unknown");
			break;
		}
		service_done_ = true;
	} else {
		RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
	}
}

/**
 * @brief Receive new setpoints from an external node/topic
 */
void OffboardControl::setpoint_callback(const TrajectorySetpoint::SharedPtr msg)
{
	RCLCPP_INFO(this->get_logger(), "Received new setpoint: [x=%.2f, y=%.2f, z=%.2f], yaw=%.2f",
	            msg->position[0], msg->position[1], msg->position[2], msg->yaw);

	// Update internal desired setpoints
	desired_x_ = msg->position[0];
	desired_y_ = msg->position[1];
	desired_z_ = msg->position[2];
	desired_yaw_ = msg->yaw;
}

/**
 * @brief Main
 */
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);

	// Create the OffboardControl node. Prefix "/fmu/" is used as example.
	auto node = std::make_shared<OffboardControl>("/fmu/");

	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
