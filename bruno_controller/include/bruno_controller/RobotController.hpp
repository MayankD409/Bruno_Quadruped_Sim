/*
 *  RobotController.hpp
 *  Author: lbrunol
 */

#pragma once
#include <eigen3/Eigen/Core>
#include <sensor_msgs/msg/imu.hpp>

#include "bruno_controller/StateCommand.hpp"
#include "bruno_controller/RestController.hpp"
#include "bruno_controller/TrotGaitController.hpp"
#include "bruno_controller/CrawlGaitController.hpp"
#include "bruno_controller/StandController.hpp"

class RobotController
{
public:
  // CrawlGaitController class constructor - set body and leg dimensions
  RobotController(const float body[], const float legs[]);

  // Main run function, return leg positions in the base_link_world frame
  Eigen::Matrix<float, 3, 4> run();

  // ROS joystick callback
  void joystick_command(const sensor_msgs::msg::Joy::ConstSharedPtr msg);

  // ROS imu callback
  void imu_orientation(const sensor_msgs::msg::Imu::ConstSharedPtr msg);

  // change current controller if requested
  void change_controller();

  // robot's state
  State state;

private:
  // variables
  float body[2];
  float legs[4];

  float delta_x;
  float delta_y;
  float x_shift_front;
  float x_shift_back;

  // rest controller
  RestController restController;

  // trot gait controller
  TrotGaitController trotGaitController;

  // crawl gait controller
  CrawlGaitController crawlGaitController;

  // stand controller
  StandController standController;

  Command command;

  // return default_stance
  Eigen::Matrix<float, 3, 4> default_stance();
};
