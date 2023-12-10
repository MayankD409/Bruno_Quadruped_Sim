/*
 *  main.cpp
 *  Author: lbrunol
 */

#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Dense>
#include "bruno_controller/InverseKinematics.hpp"
#include "bruno_controller/Transformations.hpp"
#include "bruno_controller/RobotController.hpp"
#include <vector>
#include <string>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>

#define RATE 60
int main(int argc, char * argv[])
{
  // robot body dimensions - body length, body width
  const float body_dimensions[] = {1.0, 0.50};

  // robot leg dimensions - l1, l2, l3, l4
  const float leg_dimensions[] = {0.0, 0.255, 0.520, 0.550};

  // ROS node initialization
  rclcpp::init(argc, argv);

  auto robot_controller_node = std::make_shared<rclcpp::Node>("robot_conbtroller_node");

  // RobotController
  RobotController bruno(body_dimensions, leg_dimensions);

  auto dsd = robot_controller_node->create_subscription<sensor_msgs::msg::Joy>(
    "spot_joy/joy_ramped",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotController::joystick_command, &bruno, std::placeholders::_1));

  auto dsa = robot_controller_node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/absolute",
    rclcpp::SensorDataQoS(),
    std::bind(&RobotController::imu_orientation, &bruno, std::placeholders::_1));

  // Inverse Kinematics
  InverseKinematics bruno_IK(body_dimensions, leg_dimensions);

  // Gazebo command publisher
  auto command_pub = robot_controller_node->create_publisher<std_msgs::msg::Float64MultiArray>(
    "position_controller/commands",
    rclcpp::SensorDataQoS());

  // main while loop rate
  rclcpp::Rate loop_rate(RATE);

  while (rclcpp::ok()) {
    // new leg positions
    Eigen::Matrix<float, 3, 4> leg_positions = bruno.run();
    bruno.change_controller();

    // body local position
    float dx = bruno.state.body_local_position[0];
    float dy = bruno.state.body_local_position[1];
    float dz = bruno.state.body_local_position[2];

    // body local orientation
    float roll = bruno.state.body_local_orientation[0];
    float pitch = bruno.state.body_local_orientation[1];
    float yaw = bruno.state.body_local_orientation[2];

    // inverse kinematics -> joint angles
    std::vector<double> angles = bruno_IK.inverse_kinematics(
      leg_positions,
      dx, dy, dz, roll, pitch, yaw);

    std_msgs::msg::Float64MultiArray command_message;
    command_message.layout.data_offset = 1;

    // publish joint angle commands
    for (int i = 0; i < 12; i++) {
      if (!std::isnan(angles[i])) {
        command_message.data.push_back(angles[i]);
      }
    }
    command_pub->publish(command_message);

    // spin
    rclcpp::spin_some(robot_controller_node);

    // sleep
    loop_rate.sleep();
  }

  return 0;
}
