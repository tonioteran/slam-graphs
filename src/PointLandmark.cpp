/**
 * @file PointLandmark.cpp
 * @brief Data structure for a 2D point landmark.
 * @date September 8, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include "slam-graphs/PointLandmark.h"

namespace mrg {

PointLandmark::PointLandmark(double x_pos, double y_pos)
    : x_pos_(x_pos), y_pos_(y_pos) {}

PointLandmark::~PointLandmark() {}

// Overloaded operators.

std::ostream &operator<<(std::ostream &os, const PointLandmark &l) {
  os << "Landmark: (" << l.GetXPosition() << ", " << l.GetYPosition() << ").";
  return os;
}

} // namespace mrg
