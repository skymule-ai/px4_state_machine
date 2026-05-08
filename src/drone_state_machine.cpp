#include "px4_state_machine/drone_state_machine.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

DroneStateMachine::DroneStateMachine()
    : Node("drone_state_machine"), current_state_(StateMsg::IDLE)
{
    this->declare_parameter<int>("drone_id", 1);
    this->get_parameter("drone_id", drone_id_);


    // Initialize publishers and subscribers
    state_subscriber_ = this->create_subscription<StateMsg>(
        "/vehicle_state", 10,
        std::bind(&DroneStateMachine::stateCallback, this, std::placeholders::_1));

    // Timer for state machine execution
    timer_ = this->create_wall_timer(100ms, std::bind(&DroneStateMachine::executeState, this));
}

void DroneStateMachine::stateCallback(const StateMsg::SharedPtr msg)
{
    transitionTo(msg->state);
}

void DroneStateMachine::executeState()
{
    switch (current_state_)
    {
    case StateMsg::IDLE:
        // Wait for command to take off
        break;
    case StateMsg::TAKING_OFF:
        takeOff();
        break;
    case StateMsg::HOLDING:
        hold();
        break;
    case StateMsg::LANDING:
        land();
        break;
    case StateMsg::MISSION:
        // Mission-specific behavior can be added here.
        break;
    default:
        break;
    }
}

void DroneStateMachine::takeOff()
{
    // px4_interface_->takeoff();
    transitionTo(StateMsg::HOLDING);
}

void DroneStateMachine::hold()
{
    // Logic to maintain position
}

void DroneStateMachine::land()
{
    // px4_interface_->land();
    transitionTo(StateMsg::IDLE);
}

void DroneStateMachine::shutdown()
{
    land(); // Ensure the drone lands before shutdown
}

void DroneStateMachine::transitionTo(uint8_t new_state)
{
    current_state_ = new_state;
}