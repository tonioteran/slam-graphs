/**
 * @file DubinsCar.cpp
 * @brief Implementation of the Dubins Car.
 * @date September 7, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include "slam-graphs/DubinsCar.h"

#include <math.h>

namespace mrg {

DubinsCar::DubinsCar(double initial_x, double initial_y, double initial_theta,
                     double car_speed)
    : kCarSpeed(car_speed) {
  // Setup initial conditions.
  state_.x_position = initial_x;
  state_.y_position = initial_y;
  state_.theta_angle = initial_theta;
  // Keep track of initial values in trajectory.
  trajectory_.emplace_back(state_);
}

DubinsCar::~DubinsCar() {}

void DubinsCar::MoveCar(double u) {
  // TODO(tonioteran) check control values and enforce bounds.
  state_.theta_angle += u * kDeltaT;
  state_.x_position += kCarSpeed * cos(state_.theta_angle) * kDeltaT;
  state_.y_position += kCarSpeed * sin(state_.theta_angle) * kDeltaT;

  // Add new state to trajectory.
  trajectory_.emplace_back(state_);
}

std::vector<double> DubinsCar::GetParamHistory(const std::string &param) const {
  std::vector<double> history;
  history.reserve(trajectory_.size());

  for (auto &state : trajectory_) {
    if (param == "x") {
      history.emplace_back(state.x_position);
    } else if (param == "y") {
      history.emplace_back(state.y_position);
    } else if (param == "theta") {
      history.emplace_back(state.theta_angle);
    }
  }

  return history;
}

// Operator overload.

std::ostream &operator<<(std::ostream &os, const DubinsCar &c) {
  os << "t = " << c.GetTimeStep() << ", (" << c.GetXPosition() << ", "
     << c.GetYPosition() << ", " << c.GetThetaAngle() << ").";
  return os;
}

} // namespace mrg
