# PX4 State Machine Workspace

This workspace contains a ROS 2 package that implements a state machine for controlling a PX4 drone. The state machine is designed to manage the drone's flight states, including taking off, holding position, and landing safely upon shutdown.

## Project Structure

```
px4_state_machine_ws
├── src
│   └── px4_state_machine
│       ├── include
│       │   └── px4_state_machine
│       │       ├── drone_state_machine.hpp
│       │       └── px4_interface.hpp
│       ├── src
│       │   ├── drone_state_machine.cpp
│       │   ├── px4_interface.cpp
│       │   └── main.cpp
│       ├── launch
│       │   └── px4_state_machine.launch.py
│       ├── config
│       │   └── params.yaml
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
└── README.md
```

## Setup Instructions

1. **Clone the Repository**: Clone this repository to your local machine.

2. **Install Dependencies**: Make sure you have ROS 2 installed along with any necessary dependencies for the PX4 autopilot.

3. **Build the Package**:
   Navigate to the workspace directory and run the following commands:
   ```bash
   colcon build
   ```

4. **Source the Setup File**:
   After building, source the setup file to overlay this workspace on top of your current environment:
   ```bash
   source install/setup.bash
   ```

## Usage

To run the state machine, use the provided launch file:
```bash
ros2 launch px4_state_machine px4_state_machine.launch.py
```

For the Python offboard VTOL node details (topics, commands, parameters, examples), see `README_offboard.md`.

## Features

- **Takeoff**: The drone can take off to a specified altitude.
- **Hold Mode**: The drone can maintain its position and altitude.
- **Landing**: The drone will land safely when commanded or upon shutdown.

## Additional Information

Refer to the individual README files in the `src/px4_state_machine` directory for more detailed documentation on the implementation and usage of the classes and methods within the package.
