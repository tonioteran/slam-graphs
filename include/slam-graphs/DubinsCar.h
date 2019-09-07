/**
 * @file DubinsCar.h
 * @brief Implementation of the Dubins Car.
 * @date September 7, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#ifndef SLAM_GRAPHS_DUBINS_CAR_H_
#define SLAM_GRAPHS_DUBINS_CAR_H_

#include <iostream>
#include <memory>
#include <vector>

namespace mrg {

//! Class for discrete implementation of Dubins Car.
/*!
  Based on the equations for the Dubins path:
    x_dot = V * cos(theta)
    y_dot = V * cos(theta)
    theta_dot = u,
  where `u` represents the control input, and `V` the speed of the car, the
  implementation makes use of the following discretized model:
  TODO(teran) add discretized set of equations
 */
class DubinsCar {
public:
  DubinsCar(double initial_x = 0.0, double initial_y = 0.0,
            double initial_theta = 0.0, double car_speed = 1.0);
  virtual ~DubinsCar();

  struct CarState {
    double x_position;  // [meters]
    double y_position;  // [meters]
    double theta_angle; // [radians]
  };

  // TODO(teran) Add description and implement!
  void MoveCar(double u, double delta_t);

  //! Get the car's current `CarState`.
  inline CarState GetState() { return state_; };
  //! Get the car's hitherto trajectory.
  inline std::vector<CarState> GetTrajectory() { return trajectory_; };

  //! Get the car's desired parameter hitherto history, where `param` can be
  //! "x", "y", or "theta".
  std::vector<double> GetParamHistory(const std::string &param);

private:
  //! Current navigation state of the car.
  CarState state_;
  //! Vector for storing the car's navigation history.
  std::vector<CarState> trajectory_;

  // Car properties and constants:
  const double kMaxDeltaThetaPerStep = 0.1; // [radians]
  const double kSteeringAngleBound = 0.5;   // [radians]
  const double kDeltaT = 0.1;               // [seconds]
  const double kCarSpeed = 1.0;             // [meters/second]
};

} // namespace mrg

#endif // SLAM_GRAPHS_DUBINS_CAR_H_
