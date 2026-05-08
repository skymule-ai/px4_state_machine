#include "px4_state_machine/px4_interface.hpp"
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace px4_msgs::msg;

Px4Interface::Px4Interface(rclcpp::Node * node)
    : node_(node)
{
    vehicle_command_publisher_ = node_->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
}

void Px4Interface::arm()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
    RCLCPP_INFO(node_->get_logger(), "Arming the drone");
}

void Px4Interface::takeOff(float altitude)
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, altitude);
    RCLCPP_INFO(node_->get_logger(), "Taking off to altitude: %f", altitude);
}

void Px4Interface::switchToHoldMode()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
    RCLCPP_INFO(node_->get_logger(), "Switching to hold mode");
}

void Px4Interface::land()
{
    publishVehicleCommand(VehicleCommand::VEHICLE_CMD_NAV_LAND);
    RCLCPP_INFO(node_->get_logger(), "Landing the drone");
}

void Px4Interface::publishVehicleCommand(uint16_t command, float param1, float param2)
{
    VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = node_->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}
