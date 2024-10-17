#pragma once

#include <Eigen/Dense>
#include <vector>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;

enum ObjectStatus
{
  UNDEFINED,
  STATIC,
  DYNAMIC
};

struct Object
{
  int id = -1;
  Vector10d state;  // [x, y, z, heading, length, width, height, vx, vy, vz]
  int num_points;
  float density;             // points / m^2
  float avg_residuum = 0.0;  // m, average
  ObjectStatus status = UNDEFINED;

  std::vector<int> indices;
};
