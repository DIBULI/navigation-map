#ifndef GRID_HPP
#define GRID_HPP

#include <cmath>
#include <Eigen/Core>

#include "occupancy_grid_map/structs/surface_cluster.hpp"

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
  bool isFree();

  GridState state = UNKNOWN;

  /**
   * Only store the point inside the occupied grid
   */
  Eigen::Vector3f* point;

  /**
   * Only store the normal inside the surface grid/edge
   */
  Eigen::Vector3f* normal;

  /**
   * If this grid is a surface cluster, assign which surface cluster it belongs to
   */
  SurfaceCluster* surface_cluster;
  
  float observed_score = 0;
  bool isSurfaceVoxel = false;
  bool isSurfaceEdge = false;
  bool visited = false;
  bool reachable = true;
};

#endif /* GRID_HPP */
