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
    x_t+1 = x_t + V * cos(theta_t+1) * deltaT
    y_t+1 = y_t + V * sin(theta_t+1) * deltaT
    theta_t+1 = theta_t + u * deltaT
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

  //! TODO(teran) Add description and implement!
  void MoveCar(double u);

  //! Overload for printing the current state of the car.
  friend std::ostream &operator<<(std::ostream &os, const DubinsCar &c);

  //! Get the car's current `CarState`.
  inline CarState GetState() const { return state_; };
  //! Get the car's current time step number.
  inline int GetTimeStep() const { return trajectory_.size() - 1; };
  //! Get the car's current x-position.
  inline double GetXPosition() const { return state_.x_position; };
  //! Get the car's current y-position.
  inline double GetYPosition() const { return state_.y_position; };
  //! Get the car's current heading angle.
  inline double GetThetaAngle() const { return state_.theta_angle; };
  //! Get the car's hitherto trajectory.
  inline std::vector<CarState> GetTrajectory() const { return trajectory_; };

  //! Get the car's desired parameter hitherto history, where `param` can be
  //! "x", "y", or "theta".
  std::vector<double> GetParamHistory(const std::string &param) const;

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
