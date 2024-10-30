#ifndef GRID_HPP
#define GRID_HPP

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <numeric>
#include <random>


#include "occupancy_grid_map/structs/surface_cluster.hpp"
#include "occupancy_grid_map/structs/voxel_octree.hpp"


enum GridState {
  UNKNOWN,
  OCCUPIED,
  FREE
};

class Grid {
public:
  Grid();
  ~Grid();

  const int maxPoints = 250000;
  const int maxNormals = 250000;
  const int maxOctreeResolution = 20;

  float occupancyProbabilityLog = 1.0f;

  float lmax = 100.0f;
  float lmin = -10.0f;

  float hitUpdateConstant = 8;
  float missUpdateConstant = -0.5;

  void hit();
  void miss();

  bool isOccupied();
  bool isFree();
  void computeNormals(int slidingWindowSize);
  void buildOctree(float gridSize, float originX, float originY, float originZ);
  void addPoint(const Eigen::Vector3f& point);


  GridState state = UNKNOWN;

  uint8_t depth = 1;
  bool voxelCollpased = false;
  std::vector<Grid *> subgrids;

  std::vector<Eigen::Vector3f> normals;
  VoxelOctree* octree;


  /**
   * Only store the point inside the occupied grid
   */
  std::vector<Eigen::Vector3f> points;

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
  bool reachable = true; // only used by surface edge voxels

  int xIndex;
  int yIndex;
  int zIndex;
};

#endif /* GRID_HPP */
