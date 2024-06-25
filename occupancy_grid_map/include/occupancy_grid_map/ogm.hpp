#ifndef OGM_HPP
#define OGM_HPP

#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>
#include "grid.hpp"

class OccupancyGridMap {
public:
  OccupancyGridMap();
  OccupancyGridMap(float originX, float originY, float originZ, 
                  float gridSize,
                  float mapMinX, float mapMaxX, float mapMinY, float mapMaxY, float mapMinZ, float mapMaxZ);
  ~OccupancyGridMap();

  // right now, only support set squared grid
  float mapMinX, mapMaxX, mapMinY, mapMaxY, mapMinZ, mapMaxZ;
  float gridSize;

  // store the grid in order: x, y, z
  int xIndexMin, xIndexMax, yIndexMin, yIndexMax, zIndexMin, zIndexMax;
  std::vector<Grid> grids;

  // OGM origin represented with map coordinates
  float originX;
  float originY;
  float originZ;

  void getGridIndex(float x, float y, float z, int &xIndex, int &yIndex, int &zIndex);

  void gridTravel(float xStart, float yStart, float zStart, float xDes, float yDes, float zDes, std::vector<std::tuple<int, int, int>> &grids);
};

#endif /* OGM_HPP */
