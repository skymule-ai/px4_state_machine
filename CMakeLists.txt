cmake_minimum_required(VERSION 3.5)
project(px4_state_machine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(px4_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/State.msg"
    "action/FlightCommand.action"
)

# C++ things -------------------------------
# Include directories
include_directories(include)

# Source files
set(SOURCES
    # src/drone_state_machine.cpp
    # src/px4_interface.cpp
    src/drone_start.cpp
)

# Create executable
add_executable(${PROJECT_NAME}_node ${SOURCES})

# Link libraries
ament_target_dependencies(${PROJECT_NAME}_node
    rclcpp
    rclcpp_action
    px4_msgs
)

# Link generated message interfaces so headers are discoverable
rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(${PROJECT_NAME}_node ${cpp_typesupport_target})

# Install
install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)


# Python things -------------------------------
# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/vtol_offboard.py
  scripts/lawnmower_offboard.py
  scripts/quad_offboard.py
  scripts/fleet_tf_broadcaster_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
