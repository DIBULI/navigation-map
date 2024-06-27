#include "occupancy_grid_map/ogm.hpp"
#include <iostream>


void printGrids(std::vector<std::tuple<int, int, int>> &grids) {
  int x, y, z;
  for (auto g : grids) {
    std::tie(x, y, z) = g;
    std::cout << x << ", " << y << ", " << z << std::endl;
  }
}
int main () {
  OccupancyGridMap ogm1(0.0f, 0.0f, 0.0f, 1.0f, -10.5, 10.5, -10.5, 10.5, -0.5, 0.5);

  std::vector<std::tuple<int, int, int>> grids;

  ogm1.gridTravel(-5.4, 8.8, 0, 6.7, 1.3, 0, grids);

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
    printGrids(grids);
    return -1;
  }


  grids.clear();
  ogm1.gridTravel(-1, 1, 0, 2, 2, 0, grids);
  if (grids.size() != 5 ||
      grids.at(0) != std::make_tuple<int, int, int>(-1, 1, 0) ||
      grids.at(1) != std::make_tuple<int, int, int>(0, 1, 0) ||
      grids.at(2) != std::make_tuple<int, int, int>(1, 1, 0) ||
      grids.at(3) != std::make_tuple<int, int, int>(2, 1, 0) ||
      grids.at(4) != std::make_tuple<int, int, int>(2, 2, 0)) {
    printGrids(grids);
    return -1;
  }

  grids.clear();
  OccupancyGridMap ogm2(0.0f, 0.0f, 0.0f, 0.5f, -30.5, 30.5, -30.5, 30.5, -5.5, 5.5);
  ogm2.gridTravel(0.0, 0.0, 0.0, 18.9578, 9.09207, 1.25988, grids);
  if (grids.size() != 58 ||
      grids.at(0) != std::make_tuple<int, int, int>(0, 0, 0) ||
      grids.at(1) != std::make_tuple<int, int, int>(1, 0, 0) ||
      grids.at(2) != std::make_tuple<int, int, int>(2, 0, 0) ||
      grids.at(3) != std::make_tuple<int, int, int>(2, 1, 0) ||
      grids.at(4) != std::make_tuple<int, int, int>(3, 1, 0) ||
      grids.at(5) != std::make_tuple<int, int, int>(4, 1, 0) ||
      grids.at(6) != std::make_tuple<int, int, int>(4, 2, 0) ||
      grids.at(7) != std::make_tuple<int, int, int>(5, 2, 0) ||
      grids.at(8) != std::make_tuple<int, int, int>(6, 2, 0) ||
      grids.at(9) != std::make_tuple<int, int, int>(6, 3, 0) ||
      grids.at(10) != std::make_tuple<int, int, int>(7, 3, 0) ||
      grids.at(11) != std::make_tuple<int, int, int>(8, 3, 0) ||
      grids.at(12) != std::make_tuple<int, int, int>(8, 4, 0) ||
      grids.at(13) != std::make_tuple<int, int, int>(9, 4, 0) ||
      grids.at(14) != std::make_tuple<int, int, int>(10, 4, 0) ||
      grids.at(15) != std::make_tuple<int, int, int>(10, 5, 0) ||
      grids.at(16) != std::make_tuple<int, int, int>(11, 5, 0) ||
      grids.at(17) != std::make_tuple<int, int, int>(12, 5, 0) ||
      grids.at(18) != std::make_tuple<int, int, int>(12, 6, 0) ||
      grids.at(19) != std::make_tuple<int, int, int>(13, 6, 0) ||
      grids.at(20) != std::make_tuple<int, int, int>(14, 6, 0) ||
      grids.at(21) != std::make_tuple<int, int, int>(14, 7, 0) ||
      grids.at(22) != std::make_tuple<int, int, int>(15, 7, 0) ||
      grids.at(23) != std::make_tuple<int, int, int>(15, 7, 1) ||
      grids.at(24) != std::make_tuple<int, int, int>(16, 7, 1) ||
      grids.at(25) != std::make_tuple<int, int, int>(16, 8, 1) ||
      grids.at(26) != std::make_tuple<int, int, int>(17, 8, 1) ||
      grids.at(27) != std::make_tuple<int, int, int>(18, 8, 1) ||
      grids.at(28) != std::make_tuple<int, int, int>(18, 9, 1) ||
      grids.at(29) != std::make_tuple<int, int, int>(19, 9, 1) ||
      grids.at(30) != std::make_tuple<int, int, int>(20, 9, 1) ||
      grids.at(31) != std::make_tuple<int, int, int>(20, 10, 1) ||
      grids.at(32) != std::make_tuple<int, int, int>(21, 10, 1) ||
      grids.at(33) != std::make_tuple<int, int, int>(22, 10, 1) ||
      grids.at(34) != std::make_tuple<int, int, int>(22, 11, 1) ||
      grids.at(35) != std::make_tuple<int, int, int>(23, 11, 1) ||
      grids.at(36) != std::make_tuple<int, int, int>(24, 11, 1) ||
      grids.at(37) != std::make_tuple<int, int, int>(25, 11, 1) ||
      grids.at(38) != std::make_tuple<int, int, int>(25, 12, 1) ||
      grids.at(39) != std::make_tuple<int, int, int>(26, 12, 1) ||
      grids.at(40) != std::make_tuple<int, int, int>(27, 12, 1) ||
      grids.at(41) != std::make_tuple<int, int, int>(27, 13, 1) ||
      grids.at(42) != std::make_tuple<int, int, int>(28, 13, 1) ||
      grids.at(43) != std::make_tuple<int, int, int>(29, 13, 1) ||
      grids.at(44) != std::make_tuple<int, int, int>(29, 14, 1) ||
      grids.at(45) != std::make_tuple<int, int, int>(30, 14, 1) ||
      grids.at(46) != std::make_tuple<int, int, int>(30, 14, 2) ||
      grids.at(47) != std::make_tuple<int, int, int>(31, 14, 2) ||
      grids.at(48) != std::make_tuple<int, int, int>(31, 15, 2) ||
      grids.at(49) != std::make_tuple<int, int, int>(32, 15, 2) ||
      grids.at(50) != std::make_tuple<int, int, int>(33, 15, 2) ||
      grids.at(51) != std::make_tuple<int, int, int>(33, 16, 2) ||
      grids.at(52) != std::make_tuple<int, int, int>(34, 16, 2) ||
      grids.at(53) != std::make_tuple<int, int, int>(35, 16, 2) ||
      grids.at(54) != std::make_tuple<int, int, int>(35, 17, 2) ||
      grids.at(55) != std::make_tuple<int, int, int>(36, 17, 2) ||
      grids.at(56) != std::make_tuple<int, int, int>(37, 17, 2) ||
      grids.at(57) != std::make_tuple<int, int, int>(37, 18, 2)) {
    printGrids(grids);
    return -1;
  }

  return 0;
}