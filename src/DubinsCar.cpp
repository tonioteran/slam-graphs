/**
 * @file DubinsCar.cpp
 * @brief Implementation of the Dubins Car.
 * @date September 7, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include "slam-graphs/DubinsCar.cpp"

namespace mrg {

DubinsCar::DubinsCar(double initial_x = 0.0, double initial_y = 0.0,
                     double initial_theta = 0.0, double car_speed = 1.0)
    : state_.x_position(initial_x),
    state_.y_position(initial_y), state_.theta_angle(initial_theta),
    kCarSpeed(car_speed) {
  // Add initial values to trajectory.
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
