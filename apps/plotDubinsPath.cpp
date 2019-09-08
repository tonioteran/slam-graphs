/**
 * @file plotDubinsPath.cpp
 * @brief Test for plotting within cpp.
 * @date September 8, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include "/home/tonio/repos/matplotlib-cpp/matplotlibcpp.h"
#include "slam-graphs/DubinsCar.h"

namespace plt = matplotlibcpp;

int main(int argc, char **argv) {

  std::cout << "hello DubinsCar!" << std::endl;

  // Create an instance of the car.
  mrg::DubinsCar car{};
  std::cout << car << std::endl;

  int num_steps = 20;
  for (int i = 0; i < num_steps; i++) {
    car.MoveCar(0.01);
    std::cout << car << std::endl;
  }

  auto x_hist = car.GetParamHistory("x");
  auto y_hist = car.GetParamHistory("y");
  plt::named_plot("Car", x_hist, y_hist, "o");
  plt::show();

  return 0;
}
