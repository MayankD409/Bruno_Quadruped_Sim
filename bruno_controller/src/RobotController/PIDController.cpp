/*
 *  PIDController.cpp
 *  Author: lbrunol
 */

#include <eigen3/Eigen/Dense>
#include "bruno_controller/PIDController.hpp"

#define MAX_I 0.1

///////////////////////////////////////////////////////////////////////////////
PIDController::PIDController(float p, float i, float d)
{

  // set initial p, i and d values
  kp = p;
  ki = i;
  kd = d;

  // set all values to zero
  desired_roll_pitch.setZero();
  I_term.setZero();
  last_error.setZero();

  // anti-windup
  max_I = MAX_I;
}

///////////////////////////////////////////////////////////////////////////////
void PIDController::reset()
{
  desired_roll_pitch.setZero();
  I_term.setZero();
  last_error.setZero();
  last_send_time_ = rclcpp::Clock().now();
}

///////////////////////////////////////////////////////////////////////////////
void PIDController::set_desired_RP_angles(float des_roll, float des_pitch)
{
  desired_roll_pitch[0] = des_roll;
  desired_roll_pitch[1] = des_pitch;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Vector2f PIDController::run(float roll, float pitch)
{
  Eigen::Vector2f error;
  error[0] = desired_roll_pitch[0] - roll;
  error[1] = desired_roll_pitch[1] - pitch;

  rclcpp::Time t_now = rclcpp::Clock().now();
  double step = (t_now - last_send_time_).seconds();

  I_term += error * step;

  for (int i = 0; i < 2; i++) {
    if (I_term[i] < -max_I) {
      I_term[i] = -max_I;
    } else if (I_term[i] > max_I) {
      I_term[i] = max_I;
    }
  }

  Eigen::Vector2f D_term;
  if (step == 0) {
    D_term.setZero();
  } else {
    D_term = (error - last_error) / step;
  }
  last_send_time_ = t_now;
  last_error = error;

  Eigen::Vector2f ret;
  ret = error * kp;
  ret += I_term * ki;
  ret += D_term * kd;

  return ret;
}
