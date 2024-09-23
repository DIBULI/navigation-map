#ifndef GRID_HPP
#define GRID_HPP

#include <cmath>
#include <Eigen/Core>

enum GridState {
  UNKNOWN,
  OCCUPIED,
  FREE
};

class Grid {
public:
  Grid();
  ~Grid();

  float occupancyProbabilityLog = 1.0f;

  float lmax = 100.0f;
  float lmin = -10.0f;

  float hitUpdateConstant = 8;
  float missUpdateConstant = -0.5;

  void hit();
  void miss();

  bool isOccupied();

  Eigen::Vector3f point;
  Eigen::Vector3f normal;
  bool isSurfaceVoxel = false;
  GridState state;
  bool reachable = true;
  float observed_score = 0;

};

#endif /* GRID_HPP */
