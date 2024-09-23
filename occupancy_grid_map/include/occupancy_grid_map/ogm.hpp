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
  unsigned int xMapLength, yMapLength, zMapLength; 
  std::vector<Grid> mapGrids;

  // OGM origin represented with map coordinates
  float originX;
  float originY;
  float originZ;

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

  void outputAsPointCloud(std::string filepath);
};

#endif /* OGM_HPP */
