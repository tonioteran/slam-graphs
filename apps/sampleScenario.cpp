/**
 * @file sampleScenario.cpp
 * @brief Scenario with car and landmarks.
 * @date September 8, 2019
 * @author Antonio Teran (teran@mit.edu)
 */

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <random>

#include "slam-graphs/DubinsCar.h"
#include "slam-graphs/PointLandmark.h"

#include "/home/tonio/repos/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char **argv) {

  std::cout << "Sample 2D navigation scenario." << std::endl;

  // Create an instance of the car.
  mrg::DubinsCar car{};
  std::cout << car << std::endl;

  // Navigate the car around.
  int num_steps = 20;
  for (int i = 0; i < num_steps; i++) {
    car.MoveCar(0.01);
    std::cout << car << std::endl;
  }

  // Plot dubins path.
  auto x_hist = car.GetParamHistory("x");
  auto y_hist = car.GetParamHistory("y");
  plt::named_plot("Car", x_hist, y_hist, "o");
  plt::named_plot("Car", x_hist, y_hist, "k");

  // Setup uniform distribution for landmarks.
  std::random_device rd;
  std::mt19937 mt(rd());
  std::uniform_real_distribution<double> x_dist(
      0.0, *std::max_element(std::begin(x_hist), std::end(x_hist)));
  std::uniform_real_distribution<double> y_dist(
      0.0, *std::max_element(std::begin(y_hist), std::end(y_hist)));

  // Generate random landmarks.
  int num_landmarks = 10;
  std::vector<mrg::PointLandmark> landmarks;
  std::vector<double> landmarks_x_coords, landmarks_y_coords;
  for (int i = 0; i < num_landmarks; i++) {
    mrg::PointLandmark l{x_dist(mt), y_dist(mt)};
    std::cout << l << std::endl;
    landmarks.emplace_back(l);
    landmarks_x_coords.emplace_back(l.GetXPosition());
    landmarks_y_coords.emplace_back(l.GetYPosition());
  }

  // Plot landmarks.
  for (auto &l : landmarks) {
    plt::plot(std::vector<double>{l.GetXPosition()},
              std::vector<double>{l.GetYPosition()}, "ro");
  }

  // Create adjacency matrix.
  int matrix_size = x_hist.size() + landmarks.size();
  Eigen::MatrixXd adjacency_mat =
      Eigen::MatrixXd::Zero(matrix_size, matrix_size);

  // Add pose graph edges:
  for (int i = 0; i < x_hist.size() - 1; i++) {
    adjacency_mat(i, i + 1) = 1; // Undirected symmetric.
    adjacency_mat(i + 1, i) = 1;
  }

  // Create degrees matrix.
  Eigen::MatrixXd degree_mat = Eigen::MatrixXd::Zero(matrix_size, matrix_size);

  // Add pose graph only degrees:
  degree_mat(0, 0) = 1;
  degree_mat(x_hist.size() - 1, x_hist.size() - 1) = 1;
  for (int i = 1; i < x_hist.size() - 1; i++) {
    degree_mat(i, i) = 2;
  }

  // Add landmark-related degrees:
  double min_distance = 0.3;
  std::vector<std::vector<int>> pose_landmark_pairs;
  for (int l_idx = 0; l_idx < num_landmarks; l_idx++) {
    std::cout << "l_idx: " << l_idx << std::endl;
    for (int pose_idx = 0; pose_idx < x_hist.size(); pose_idx++) {
      std::cout << "  pose_idx: " << pose_idx << std::endl;
      // Check for min distance and add edge if below.
      double distance =
          sqrt(pow(x_hist[pose_idx] - landmarks[l_idx].GetXPosition(), 2) +
               pow(y_hist[pose_idx] - landmarks[l_idx].GetYPosition(), 2));
      std::cout << "   distance: " << distance << std::endl;
      if (distance <= min_distance) {
        // Record pose/landmark pair.
        std::vector<int> edge{pose_idx, l_idx};
        pose_landmark_pairs.push_back(edge);

        // Sum degree to both nodes.
        degree_mat(pose_idx, pose_idx) = degree_mat(pose_idx, pose_idx) + 1;
        degree_mat(x_hist.size() + l_idx, x_hist.size() + l_idx) =
            degree_mat(x_hist.size() + l_idx, x_hist.size() + l_idx) + 1;

        // Add edge to adjacency matrix.
        adjacency_mat(pose_idx, x_hist.size() + l_idx) = 1;
        adjacency_mat(x_hist.size() + l_idx, pose_idx) = 1;

        std::cout << "   ++++++++++++Add edge!" << std::endl;
      }
    }
  }

  // Plot edges between poses and landmarks.
  for (auto &edge : pose_landmark_pairs) {
    plt::plot(
        std::vector<double>{x_hist[edge[0]], landmarks[edge[1]].GetXPosition()},
        std::vector<double>{y_hist[edge[0]], landmarks[edge[1]].GetYPosition()},
        "k");
  }

  // Create the Laplacian matrix:
  auto laplacian_mat = degree_mat - adjacency_mat;

  std::cout << "Degree matrix:\n" << degree_mat << std::endl << std::endl;
  std::cout << "Adjacency matrix:\n" << adjacency_mat << std::endl << std::endl;
  std::cout << "Laplacian matrix:\n" << laplacian_mat << std::endl << std::endl;
  plt::show();

  // Obtain eigenvalues and eigenvectors.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(laplacian_mat);
  std::cout << es.eigenvalues() << std::endl << std::endl;
  std::cout << laplacian_mat * es.eigenvectors().col(0) << std::endl;
  auto v1 = es.eigenvectors().col(1);
  auto v2 = es.eigenvectors().col(2);
  auto v3 = es.eigenvectors().col(3);
  auto v4 = es.eigenvectors().col(4).real();
  std::vector<double> v1_vec, v2_vec, v3_vec, v4_vec;
  for (int i = 0; i < es.eigenvectors().col(1).real().size(); i++) {
    v1_vec.push_back(v1[i]);
    v2_vec.push_back(v2[i]);
    v3_vec.push_back(v3[i]);
    v4_vec.push_back(v4[i]);
  }

  plt::plot(v1_vec, v2_vec);
  plt::plot(v1_vec, v2_vec, "ro");
  plt::show();

  plt::plot(v2_vec, v3_vec);
  plt::plot(v2_vec, v3_vec, "ro");
  plt::show();

  plt::plot(v3_vec, v4_vec);
  plt::plot(v3_vec, v4_vec, "ro");
  plt::show();

  return 0;
}
