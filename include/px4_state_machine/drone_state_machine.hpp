#ifndef DRONE_STATE_MACHINE_HPP
#define DRONE_STATE_MACHINE_HPP

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "px4_state_machine/action/flight_command.hpp"
#include "px4_state_machine/msg/state.hpp"

class DroneStateMachine : public rclcpp::Node
{
public:
    DroneStateMachine();

    void shutdown();

private:
    using StateMsg = px4_state_machine::msg::State;
    using FlightCommand = px4_state_machine::action::FlightCommand;
    using GoalHandleFlightCommand = rclcpp_action::ServerGoalHandle<FlightCommand>;

    void stateCallback(const StateMsg::SharedPtr msg);
    void executeState();

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const FlightCommand::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFlightCommand> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandleFlightCommand> goal_handle);
    void executeGoal(const std::shared_ptr<GoalHandleFlightCommand> goal_handle);

    bool handleCommand(uint8_t command, float takeoff_altitude, std::string & message);
    bool commandAllowed(uint8_t command, std::string & reason) const;

    bool takeOff(float altitude, std::string & message);

    void setState(uint8_t new_state);
    uint8_t getState() const;
    void publishState();

    int drone_id_;
    float default_takeoff_altitude_;

    mutable std::mutex state_mutex_;
    std::mutex command_mutex_;
    uint8_t current_state_;
    bool command_in_progress_;

    rclcpp::Publisher<StateMsg>::SharedPtr state_publisher_;
    rclcpp::Subscription<StateMsg>::SharedPtr state_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_action::Server<FlightCommand>::SharedPtr action_server_;
};

#endif // DRONE_STATE_MACHINE_HPP
