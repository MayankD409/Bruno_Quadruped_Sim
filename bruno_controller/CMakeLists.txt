cmake_minimum_required(VERSION 3.5)
project(bruno_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

## Find other packages/libraries
find_package(Eigen3 REQUIRED)

include_directories(include)

set(dependencies
rclcpp
std_msgs
sensor_msgs
tf2
Eigen3
)

add_executable(robot_controller_gazebo 
    src/main.cpp 
    src/InverseKinematics/InverseKinematics.cpp 
    src/RoboticsUtilities/Transformations.cpp
    src/RobotController/RobotController.cpp
    src/RobotController/RestController.cpp
    src/RobotController/TrotGaitController.cpp
    src/RobotController/GaitController.cpp
    src/RobotController/CrawlGaitController.cpp
    src/RobotController/PIDController.cpp
    src/RobotController/StandController.cpp
    )

ament_target_dependencies(robot_controller_gazebo ${dependencies})

install(TARGETS robot_controller_gazebo
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

install(
  DIRECTORY 
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_package()