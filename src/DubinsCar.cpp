/**
 * @file DubinsCar.cpp
 * @brief Implementation of the Dubins Car.
 * @date September 7, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include "slam-graphs/DubinsCar.h"

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

std::vector<double> DubinsCar::GetParamHistory(const std::string &param) {
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

} // namespace mrg
