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

  // TODO(teran) Add description.
  void MoveCar(double u, double delta_t);

  //! Get the car's current `CarState`.
  inline std::unique_ptr<CarState> GetState() {
    return std::make_unique<CarState>(state_);
  };

private:
  //! Current navigation state of the car.
  CarState state_;
  //! Vector for storing the car's navigation history.
  std::vector<CarState> trajectory_;

  //! Current car's state variable for x-position, in meters.
  double current_x_;
  //! Current car's state variable for y-position, in meters.
  double current_y_;
  //! Current car's state variable for orientation angle, in radians.
  double current_theta_;

  //! Vector for storing the x-position sequence of the car's trajectory.
  std::vector<double> x_history_;
  //! Vector for storing the y-position sequence of the car's trajectory.
  std::vector<double> y_history_;
  //! Vector for storing the orientation sequence of the car's trajectory.
  std::vector<double> theta_history_;

  // Car properties and constants:
  double kMaxTurningAngle = 0.5; // [radians]
  double kDeltaT = 0.1;          // [seconds]
};

#endif // SLAM_GRAPHS_DUBINS_CAR_H_
