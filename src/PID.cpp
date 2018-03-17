#include "PID.h"

PID::PID() = default;

PID::~PID() = default;

void PID::Init(const double Kp, const double Ki, const double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
}

void PID::UpdateError(const double cte) {
  const auto previous_cte = p_error;
  p_error = cte;
  // {i,d}_error adjustments should ideally be multiplied by delta-time,
  // especially if updates not received in consistent intervals
  i_error += cte;
  d_error = cte - previous_cte;
}

double PID::TotalError() const {
  // assuming the K-factors are positive, sum of products is the error
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::SteerValue() const {
  // Steer the opposite of the error
  double steer = -TotalError();
  if (steer < -1.0) {
    return -1.0;
  }
  if (steer > 1.0) {
    return 1.0;
  }
  return steer;
}
