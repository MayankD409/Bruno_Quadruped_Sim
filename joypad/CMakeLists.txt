cmake_minimum_required(VERSION 3.5)
project(joypad)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# Create an executable
# add_executable(joy_controller src/ramped_joypad.py)

# # Link the executable to the required libraries
# ament_target_dependencies(joy_controller rclpy std_msgs sensor_msgs)

# # Install the executable
# install(TARGETS joy_controller
#   DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
    )

install(
    PROGRAMS
    src/ramped_joypad.py
    DESTINATION lib/${PROJECT_NAME}
    )

ament_package()
