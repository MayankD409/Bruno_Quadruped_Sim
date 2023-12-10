/*
 *  RestController.hpp
 *  Author: lbrunol
 */

#pragma once
#include <eigen3/Eigen/Core>
#include <sensor_msgs/msg/joy.hpp>

#include "bruno_controller/StateCommand.hpp"
#include "bruno_controller/PIDController.hpp"

class RestController
{
public:
  // RestController class constructor - set default stance
  RestController(Eigen::Matrix<float, 3, 4> def_stance);

  // ROS joystick callback - update state and other variables
  void updateStateCommand(
    const sensor_msgs::msg::Joy::ConstSharedPtr msg,
    State & state, Command & command);

  void reset_pid_controller();

  // Main run function, return leg positions in the base_link_world frame
  Eigen::Matrix<float, 3, 4> run(State & state, Command & command);

private:
  // default stance
  Eigen::Matrix<float, 3, 4> def_stance;

  // return default_stance
  Eigen::Matrix<float, 3, 4> default_stance();

  // Controller step - return new leg positions
  Eigen::Matrix<float, 3, 4> step(State & state, Command & command);

  // pid controller
  PIDController pid_controller;
  float max_tilt;

  bool use_imu;
  bool use_button;
};
