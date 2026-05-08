#ifndef PX4_INTERFACE_HPP
#define PX4_INTERFACE_HPP

#include <cstdint>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

class Px4Interface
{
public:
    explicit Px4Interface(rclcpp::Node * node);

    void arm();
    void takeOff(float altitude);
    void switchToHoldMode();
    void land();

private:
    void publishVehicleCommand(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Node * node_;
};

#endif // PX4_INTERFACE_HPP
