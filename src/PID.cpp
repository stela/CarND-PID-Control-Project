#include "PID.h"


/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(double cte) {
  const auto previous_cte = p_error;
  p_error = cte;
  // {i,d}_error adjustments should ideally be multiplied by delta-time, especially if updates not received with predictable intervals
  i_error += cte;
  d_error = cte - previous_cte;
}

double PID::TotalError() {
  // assuming the K-factors are positive, sum of products is the error
  return Kp * p_error + Ki * i_error + Kd + d_error;
}

double PID::steer_value() {
  // Steer the opposite of the error
  double steer = TotalError();
  if (steer < -1.0) {
    return -1.0;
  }
  if (steer > 1.0) {
    return 1.0;
  }
  return steer;
}
