#ifndef GRID_HPP
#define GRID_HPP

#include <cmath>

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
};

#endif /* GRID_HPP */
