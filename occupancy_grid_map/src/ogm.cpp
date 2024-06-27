#include "occupancy_grid_map/ogm.hpp"

OccupancyGridMap::OccupancyGridMap() {}

OccupancyGridMap::OccupancyGridMap(float originX, float originY, float originZ, float gridSize,
float mapMinX, float mapMaxX, float mapMinY, float mapMaxY, float mapMinZ, float mapMaxZ)
: originX(originX), originY(originY), originZ(originZ), gridSize(gridSize),
mapMinX(mapMinX), mapMaxX(mapMaxX), mapMinY(mapMinY), mapMaxY(mapMaxY), mapMinZ(mapMinZ), mapMaxZ(mapMaxZ) 
{
  // assign the mapGrids
  xIndexMin = floor((mapMinX - originX) / gridSize);
  yIndexMin = floor((mapMinY - originY) / gridSize);
  zIndexMin = floor((mapMinZ - originZ) / gridSize);

  xIndexMax = floor((mapMaxX - originX) / gridSize);
  yIndexMax = floor((mapMaxY - originY) / gridSize);
  zIndexMax = floor((mapMaxZ - originZ) / gridSize);
  
  xMapLength = xIndexMax - xIndexMin + 1;
  yMapLength = yIndexMax - yIndexMin + 1;
  zMapLength = zIndexMax - zIndexMin + 1;

  mapGrids.resize(xMapLength * yMapLength * zMapLength);
}

OccupancyGridMap::~OccupancyGridMap() {}


void OccupancyGridMap::getGridIndex(float x, float y, float z, int &xIndex, int &yIndex, int &zIndex) {
  xIndex = floor((x - originX) / gridSize);
  yIndex = floor((y - originY) / gridSize);
  zIndex = floor((z - originZ) / gridSize);

  xIndex = std::max(xIndexMin, std::min(xIndex, xIndexMax));
  yIndex = std::max(yIndexMin, std::min(yIndex, yIndexMax));
  zIndex = std::max(zIndexMin, std::min(zIndex, zIndexMax));
}

float OccupancyGridMap::getInitialTD(float start, float direction) {
  if (direction >= 0) {
    if (start >= 0) {
      return fabs((gridSize - fmod(start, gridSize)) / direction);
    } else {
      return fabs((-fmod(start, gridSize)) / direction);
    }
  } else {
    if (start >= 0) {
      return fabs((fmod(start, gridSize)) / -direction);
    } else {
      return fabs((gridSize - fmod(-start, gridSize)) / -direction);
    }
  }
}

void OccupancyGridMap::gridTravel(float xStart, float yStart, float zStart, float xDes, float yDes, float zDes, std::vector<std::tuple<int, int, int>> &grids) {
  float directionX, directionY, directionZ;
  directionX = xDes - xStart;
  directionY = yDes - yStart;
  directionZ = zDes - zStart;

  float length = sqrt(directionX * directionX + directionY * directionY + directionZ * directionZ);
  
  int xStep, yStep, zStep;
  xStep = directionX > 0 ? 1 : -1;
  yStep = directionY > 0 ? 1 : -1;
  zStep = directionZ > 0 ? 1 : -1;

  // delta can be inf
  float xDelta, yDelta, zDelta;
  xDelta = gridSize / fabs(directionX);
  yDelta = gridSize / fabs(directionY);
  zDelta = gridSize / fabs(directionZ);

  int x, y, z, xEnd, yEnd, zEnd;
  getGridIndex(xStart, yStart, zStart, x, y, z);
  getGridIndex(xDes, yDes, zDes, xEnd, yEnd, zEnd);

  // initialize the travel distance
  float xTD, yTD, zTD;
  xTD = getInitialTD(xStart, directionX);
  yTD = getInitialTD(yStart, directionY);
  zTD = getInitialTD(zStart, directionZ);

  if (xDelta == std::numeric_limits<float>::infinity()) {
    xTD = std::numeric_limits<float>::infinity();
  }
  if (yDelta == std::numeric_limits<float>::infinity()) {
    yTD = std::numeric_limits<float>::infinity();
  }
  if (zDelta == std::numeric_limits<float>::infinity()) {
    zTD = std::numeric_limits<float>::infinity();
  }

  grids.push_back(std::make_tuple(x, y, z));
  while (x != xEnd || y != yEnd || z != zEnd) {
    // only compare the dimension which has not travelled the whole distance
    if (xTD < yTD && xTD < zTD) {
      // xTD is the smallest
      xTD += xDelta;
      x += xStep;
    } else if (yTD < zTD) {
      // yTD is the smallest
      yTD += yDelta;
      y += yStep;
    } else {
      // zTD is the smallest
      zTD += zDelta;
      z += zStep;
    }

    grids.push_back(std::make_tuple(x, y, z));
  }
}

void OccupancyGridMap::updateMap(float xPos, float yPos, float zPos, std::vector<std::tuple<float, float, float>> points) {
  std::vector<std::tuple<int, int, int>> grids;
  float x, y, z;
  int gridX, gridY, gridZ;
  Grid *grid;

  for (auto &point : points) {
    std::tie(x, y, z) = point;
    gridTravel(xPos, yPos, zPos, x, y, z, grids);
    
    for (size_t i = 0; i < grids.size() - 1; i++) {
      std::tie(gridX, gridY, gridZ) = grids[i];
      grid = getGridByIndex(gridX, gridY, gridZ);
      grid->miss();
    }
    std::tie(gridX, gridY, gridZ) = grids[grids.size() - 1];
    grid = getGridByIndex(gridX, gridY, gridZ);
    grid->hit();
    
    grids.clear();
  }
}

Grid* OccupancyGridMap::getGridByIndex(int x, int y, int z) {
  return &mapGrids.at((xMapLength * yMapLength) * (z - zIndexMin) + xMapLength * (y - yIndexMin) + x - xIndexMin);
}

void OccupancyGridMap::getGridIndex(unsigned int gridVectorIndex, int &x, int &y, int &z) {
  z = (gridVectorIndex) / (xMapLength * yMapLength) + zIndexMin;
  y = (gridVectorIndex - (z - zIndexMin) * xMapLength * yMapLength) / yMapLength + yIndexMin;
  x = (gridVectorIndex - (z - zIndexMin) * xMapLength * yMapLength - yMapLength * (y - yIndexMin)) + xIndexMin;
}

void OccupancyGridMap::gridIndexToPosition(int xIndex, int yIndex, int zIndex, float &x, float &y, float &z) {
  x = xIndex * gridSize + originX;
  y = yIndex * gridSize + originY;
  z = zIndex * gridSize + originZ;
}