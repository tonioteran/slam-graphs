/**
 * @file PointLandmark.h
 * @brief Data structure for a 2D point landmark.
 * @date September 8, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#ifndef SLAM_GRAPHS_POINT_LANDMARK_H_
#define SLAM_GRAPHS_POINT_LANDMARK_H_

#include <iostream>

namespace mrg {

//! Class for storing a Gaussian 2D point landmark.
/*!
  //TODO(tonioteran) Change to matrix representation and add uncertainty.
 */
class PointLandmark {
public:
  PointLandmark(double x_pos, double y_pos);
  virtual ~PointLandmark();

  //! Overload for printing the landmark's information.
  friend std::ostream &operator<<(std::ostream &os, const PointLandmark &l);

  //! Get the landmark's x-position.
  inline double GetXPosition() const { return x_pos_; };
  //! Get the landmark's y-position.
  inline double GetYPosition() const { return y_pos_; };

private:
  double x_pos_; // [meters]
  double y_pos_; // [meters]
};

} // namespace mrg

#endif // SLAM_GRAPHS_POINT_LANDMARK_H_
