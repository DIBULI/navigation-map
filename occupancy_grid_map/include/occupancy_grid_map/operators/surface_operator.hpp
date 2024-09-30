#ifndef SURFACE_OPERATOR_HPP
#define SURFACE_OPERATOR_HPP

#include <vector>
#include <map>

#include "occupancy_grid_map/operators/grid_operator.hpp"
#include "occupancy_grid_map/structs/surface_cluster.hpp"

class SurfaceOperator: public GridOperator {
public:
  SurfaceOperator();
  ~SurfaceOperator();

  void grid_operator(Grid* grid, int x, int y, int z, OccupancyGridMap* gridMap) override;

  bool is_surface(int unknownNums, int occupiedNums, int surfaceNums, int surfaceEdgeNums);
  
  bool is_edge(int unknownNums, int occupiedNums, int surfaceNums, int surfaceEdgeNums);

  SurfaceCluster* create_new_surface_cluster(Grid* &grid);

  void calNorm(int x, int y, int z, Grid* grid, OccupancyGridMap* gridMap);

  std::atomic<int> surface_cluster_global_id;

  std::vector<std::tuple<int, int, int>> connectedNeighbours;
  std::vector<std::tuple<int, int, int>> surrNeighbours;

  std::map<int, SurfaceCluster *> surface_clusters;
};

#endif /* SURFACE_OPERATOR_HPP */
