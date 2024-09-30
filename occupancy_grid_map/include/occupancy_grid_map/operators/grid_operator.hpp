#ifndef GRID_OPERATOR_HPP
#define GRID_OPERATOR_HPP

#include <queue>

#include "occupancy_grid_map/structs/grid.hpp"

class OccupancyGridMap;

class GridOperator {
  virtual void grid_operator(Grid* grid, int x, int y, int z, OccupancyGridMap* gridMap) = 0;
};

#endif /* GRID_OPERATOR_HPP */
