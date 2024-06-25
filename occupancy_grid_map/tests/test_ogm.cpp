#include "occupancy_grid_map/ogm.hpp"
#include <iostream>

int main () {
  OccupancyGridMap ogm(0.0f, 0.0f, 0.0f, 1.0f, -10.5, 10.5, -10.5, 10.5, -0.5, 0.5);

  std::vector<std::tuple<int, int, int>> grids;

  ogm.gridTravel(-5.4, 8.8, 0, 6.7, 1.3, 0, grids);

  if (grids.size() != 20 ||
      grids.at(0) != std::make_tuple<int, int, int>(-6, 8, 0) ||
      grids.at(1) != std::make_tuple<int, int, int>(-5, 8, 0) ||
      grids.at(2) != std::make_tuple<int, int, int>(-5, 7, 0) ||
      grids.at(3) != std::make_tuple<int, int, int>(-4, 7, 0) ||
      grids.at(4) != std::make_tuple<int, int, int>(-3, 7, 0) ||
      grids.at(5) != std::make_tuple<int, int, int>(-3, 6, 0) ||
      grids.at(6) != std::make_tuple<int, int, int>(-2, 6, 0) ||
      grids.at(7) != std::make_tuple<int, int, int>(-1, 6, 0) ||
      grids.at(8) != std::make_tuple<int, int, int>(-1, 5, 0) ||
      grids.at(9) != std::make_tuple<int, int, int>(0, 5, 0) ||
      grids.at(10) != std::make_tuple<int, int, int>(0, 4, 0) ||
      grids.at(11) != std::make_tuple<int, int, int>(1, 4, 0) ||
      grids.at(12) != std::make_tuple<int, int, int>(2, 4, 0) ||
      grids.at(13) != std::make_tuple<int, int, int>(2, 3, 0) ||
      grids.at(14) != std::make_tuple<int, int, int>(3, 3, 0) ||
      grids.at(15) != std::make_tuple<int, int, int>(3, 2, 0) ||
      grids.at(16) != std::make_tuple<int, int, int>(4, 2, 0) ||
      grids.at(17) != std::make_tuple<int, int, int>(5, 2, 0) ||
      grids.at(18) != std::make_tuple<int, int, int>(5, 1, 0) ||
      grids.at(19) != std::make_tuple<int, int, int>(6, 1, 0)) {
    return -1;
  }


  grids.clear();
  ogm.gridTravel(-1, 1, 0, 2, 2, 0, grids);
  if (grids.size() != 5 ||
      grids.at(0) != std::make_tuple<int, int, int>(-1, 1, 0) ||
      grids.at(1) != std::make_tuple<int, int, int>(0, 1, 0) ||
      grids.at(2) != std::make_tuple<int, int, int>(1, 1, 0) ||
      grids.at(3) != std::make_tuple<int, int, int>(2, 1, 0) ||
      grids.at(4) != std::make_tuple<int, int, int>(2, 2, 0)) {
    return -1;
  }

  return 0;
}