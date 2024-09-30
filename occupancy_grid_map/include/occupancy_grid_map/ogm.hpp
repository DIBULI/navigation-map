#ifndef OGM_HPP
#define OGM_HPP

#include <cmath>
#include <vector>
#include <tuple>
#include <iostream>
#include <limits>
#include <fstream>
#include <iostream>
#include <filesystem>

#include "occupancy_grid_map/structs/grid.hpp"
#include "occupancy_grid_map/operators/surface_operator.hpp"

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
  unsigned int xMapLength, yMapLength, zMapLength; 
  std::vector<Grid> mapGrids;

  // OGM origin represented with map coordinates
  float originX;
  float originY;
  float originZ;

  // numbers to count different grid types
  uint32_t freeGridNum = 0;
  uint32_t occupiedGridNum = 0;
  uint32_t unknownGridNum = 0;
  uint32_t totalGridNum = 0;

  SurfaceOperator surfaceOperator;

  void getGridIndex(float x, float y, float z, int &xIndex, int &yIndex, int &zIndex);

  void getGridIndex(unsigned int gridVectorIndex, int &x, int &y, int &z);

  void gridIndexToPosition(int xIndex, int yIndex, int zIndex, float &x, float &y, float &z);

  float getInitialTD(float start, float direction);

  void gridTravel(float xStart, float yStart, float zStart, float xDes, float yDes, float zDes, std::vector<std::tuple<int, int, int>> &grids);

  /**
   * The received position and points should all be described under the map frame
   */
  void updateMap(float xPos, float yPos, float zPos, std::vector<std::tuple<float, float, float>> points);

  Grid* getGridByIndex(int x, int y, int z);

  Grid* getNeightbourGrid(int x, int y, int z, int nbX, int nbY, int nbZ);

  

  void outputAsPointCloud(std::string filepath);
};

#endif /* OGM_HPP */
