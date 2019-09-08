#include "slam-graphs/DubinsCar.h"

int main(int argc, char **argv) {

  std::cout << "hello DubinsCar!" << std::endl;

  // Create an instance of the car.
  mrg::DubinsCar car{};
  std::cout << car << std::endl;

  int num_steps = 1000;
  for (int i = 0; i < num_steps; i++) {
    car.MoveCar(0.01);
    std::cout << car << std::endl;
  }

  return 0;
}
