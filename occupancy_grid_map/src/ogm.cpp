#include "occupancy_grid_map/ogm.hpp"

OccupancyGridMap::OccupancyGridMap() {}

OccupancyGridMap::OccupancyGridMap(float originX, float originY, float originZ, float gridSize,
float mapMinX, float mapMaxX, float mapMinY, float mapMaxY, float mapMinZ, float mapMaxZ)
: originX(originX), originY(originY), originZ(originZ), gridSize(gridSize),
mapMinX(mapMinX), mapMaxX(mapMaxX), mapMinY(mapMinY), mapMaxY(mapMaxY), mapMinZ(mapMinZ), mapMaxZ(mapMaxZ) 
{
  float half_grid_size = gridSize / 2;
  this->mapMinX -= half_grid_size;
  this->mapMinY -= half_grid_size;
  this->mapMinZ -= half_grid_size;

  this->mapMaxX += half_grid_size;
  this->mapMaxY += half_grid_size;
  this->mapMaxZ += half_grid_size;

  // assign the mapGrids
  xIndexMin = floor((mapMinX - originX) / gridSize);
  yIndexMin = floor((mapMinY - originY) / gridSize);
  zIndexMin = floor((mapMinZ - originZ) / gridSize);

  xIndexMax = ceil((mapMaxX - originX) / gridSize);
  yIndexMax = ceil((mapMaxY - originY) / gridSize);
  zIndexMax = ceil((mapMaxZ - originZ) / gridSize);
  
  xMapLength = xIndexMax - xIndexMin + 1;
  yMapLength = yIndexMax - yIndexMin + 1;
  zMapLength = zIndexMax - zIndexMin + 1;

  unknownGridNum = xMapLength * yMapLength * zMapLength;

  totalGridNum = unknownGridNum;
  
  mapGrids.resize(totalGridNum);

  std::cout << "Map size:" << std::endl;
  std::cout << "mapMinX:" << this->mapMinX << ", mapMaxX: " << this->mapMaxX << std::endl;
  std::cout << "mapMinY:" << this->mapMinY << ", mapMaxY: " << this->mapMaxY << std::endl;
  std::cout << "mapMinZ:" << this->mapMinZ << ", mapMaxZ: " << this->mapMaxZ << std::endl;
  std::cout << "xMapLength:" << xMapLength << ", xIndexMin: " << xIndexMin << ", xIndexMax: " << xIndexMax <<  std::endl;
  std::cout << "yMapLength:" << yMapLength << ", yIndexMin: " << yIndexMin << ", yIndexMax: " << yIndexMax <<  std::endl;
  std::cout << "zMapLength:" << zMapLength << ", zIndexMin: " << zIndexMin << ", zIndexMax: " << zIndexMax <<  std::endl;
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

bool OccupancyGridMap::gridTravel(float xStart, float yStart, float zStart, float xDes, float yDes, float zDes, std::vector<std::tuple<int, int, int>> &grids) {
  bool exceedMapRegion = 
    xDes < mapMinX || xDes > mapMaxX || yDes < mapMinY || yDes > mapMaxY || zDes < mapMinZ || zDes > mapMaxZ;

  float directionX, directionY, directionZ;
  directionX = xDes - xStart;
  directionY = yDes - yStart;
  directionZ = zDes - zStart;

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

  // TODO: check when xstart or xend are outside of the map scope, this grid travel may not work.
  grids.push_back(std::make_tuple(x, y, z));
  while (x != xEnd || y != yEnd || z != zEnd) {
    // only compare the dimension which has not travelled the whole distance
    if ((xTD < yTD && xTD < zTD) && x != xEnd || (y == yEnd && z == zEnd)) {
      // xTD is the smallest
      xTD += xDelta;
      x += xStep;
    } else if (yTD < zTD && y != yEnd || z == zEnd ) {
      // yTD is the smallest
      yTD += yDelta;
      y += yStep;
    } else if (z != zEnd) {
      // zTD is the smallest
      zTD += zDelta;
      z += zStep;
    }

    grids.push_back(std::make_tuple(x, y, z));
  }
  return exceedMapRegion;
}

void OccupancyGridMap::updateMap(float xPos, float yPos, float zPos, std::vector<std::tuple<float, float, float>> points) {
  std::vector<std::tuple<int, int, int>> grids;
  float x, y, z;
  int gridX, gridY, gridZ;
  Grid *grid;

  for (auto &point : points) {
    std::tie(x, y, z) = point;

    bool exceedMapRegion = gridTravel(xPos, yPos, zPos, x, y, z, grids);
    
    for (size_t i = 0; i < grids.size() - 1; i++) {
      std::tie(gridX, gridY, gridZ) = grids[i];
      grid = getGridByIndex(gridX, gridY, gridZ);
      grid->miss();
      if (grid->state == GridState::UNKNOWN && grid->isFree()) {
        freeGridNum += 1;
        unknownGridNum -= 1;
        grid->state = GridState::FREE;
      } else if (grid->state == GridState::OCCUPIED && grid->isFree()) {
        freeGridNum += 1;
        occupiedGridNum -= 1;
        grid->state = GridState::FREE;
      }
    }
    std::tie(gridX, gridY, gridZ) = grids[grids.size() - 1];
    grid = getGridByIndex(gridX, gridY, gridZ);
    if (!exceedMapRegion) {
      grid->hit();
    } else {
      grid->miss(); 
    }

    // post process
    if (grid->state == GridState::UNKNOWN && grid->isOccupied()) {
      occupiedGridNum += 1;
      unknownGridNum -= 1;
      grid->state = GridState::OCCUPIED;
    } else if (grid->state == GridState::FREE && grid->isOccupied()) {
      occupiedGridNum += 1;
      freeGridNum -= 1;
      grid->state = GridState::OCCUPIED;

      grid->isSurfaceVoxel = false;
      grid->isSurfaceEdge = false;
    }

    // check the last free grid 
    if (grid->isOccupied()) {
      std::tie(gridX, gridY, gridZ) = grids[grids.size() - 2];
      grid = getGridByIndex(gridX, gridY, gridZ);
      if (grid->isFree()) {
        surfaceOperator.grid_operator(grid, gridX, gridY, gridZ, this);
      }
    }

    grids.clear();
  }
}

Grid* OccupancyGridMap::getGridByIndex(int x, int y, int z) {
  return &mapGrids.at((xMapLength * yMapLength) * (z - zIndexMin) + xMapLength * (y - yIndexMin) + x - xIndexMin);
}

Grid* OccupancyGridMap::getNeightbourGrid(int x, int y, int z, int nbX, int nbY, int nbZ) {
  int xIndex = std::max(xIndexMin, std::min(x + nbX, xIndexMax));
  int yIndex = std::max(yIndexMin, std::min(y + nbY, yIndexMax));
  int zIndex = std::max(zIndexMin, std::min(z + nbZ, zIndexMax));
  return getGridByIndex(xIndex, yIndex, zIndex);
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

void OccupancyGridMap::outputAsPointCloud(std::string filepath) {
  std::ofstream outFile(filepath + "/point_cloud.ply");
  if (!outFile) {
      std::cerr << "Can not open the point cloud file!" << std::endl;
      return;
  }

  uint32_t point_num = 0;
  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isOccupied()) {
      point_num++;
    }
  }

  outFile << "ply\n";
  outFile << "format ascii 1.0\n";
  outFile << "element vertex " << point_num << "\n";
  outFile << "property float x\n";
  outFile << "property float y\n";
  outFile << "property float z\n";
  outFile << "property uchar red\n";
  outFile << "property uchar green\n";
  outFile << "property uchar blue\n";
  outFile << "end_header\n";

  float x, y, z;
  int xIndex, yIndex, zIndex;
  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isOccupied()) {
      this->getGridIndex(i, xIndex, yIndex, zIndex);
      this->gridIndexToPosition(xIndex, yIndex, zIndex, x, y, z);
      outFile << x << " " << y << " " << z << " "  << 180 << " " << 180 << " " 
        << int(255 * (zIndex - this->zIndexMin) / (zIndexMax - zIndexMin))  << "\n";
    }
  }

  outFile.close();

  std::ofstream surfacePCOutFile(filepath + "/point_cloud_surface.ply");
  if (!surfacePCOutFile) {
      std::cerr << "Can not open the point cloud file!" << std::endl;
      return;
  }

  point_num = 0;
  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isSurfaceVoxel) {
      point_num++;
    }
  }

  surfacePCOutFile << "ply\n";
  surfacePCOutFile << "format ascii 1.0\n";
  surfacePCOutFile << "element vertex " << point_num << "\n";
  surfacePCOutFile << "property float x\n";
  surfacePCOutFile << "property float y\n";
  surfacePCOutFile << "property float z\n";
  surfacePCOutFile << "property uchar red\n";
  surfacePCOutFile << "property uchar green\n";
  surfacePCOutFile << "property uchar blue\n";
  surfacePCOutFile << "property float nx\n";
  surfacePCOutFile << "property float ny\n";
  surfacePCOutFile << "property float nz\n";
  surfacePCOutFile << "end_header\n";

  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isSurfaceVoxel) {
      this->getGridIndex(i, xIndex, yIndex, zIndex);
      this->gridIndexToPosition(xIndex, yIndex, zIndex, x, y, z);
      surfacePCOutFile << x << " " << y << " " << z << " " << 80 << " " << 10 << " " 
      << int(255 * (zIndex - this->zIndexMin) / (zIndexMax - zIndexMin)) << " "
      << this->mapGrids[i].normal->x() << " " << this->mapGrids[i].normal->y() << " " << this->mapGrids[i].normal->z() << "\n";
    }
  }

  surfacePCOutFile.close();

  std::ofstream surfaceEdgeFile(filepath + "/point_cloud_surface_edges.ply");
  if (!surfaceEdgeFile) {
      std::cerr << "Can not open the point cloud file!" << std::endl;
      return;
  }

  point_num = 0;
  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isSurfaceEdge && this->mapGrids[i].reachable) {
      point_num++;
    }
  }

  surfaceEdgeFile << "ply\n";
  surfaceEdgeFile << "format ascii 1.0\n";
  surfaceEdgeFile << "element vertex " << point_num << "\n";
  surfaceEdgeFile << "property float x\n";
  surfaceEdgeFile << "property float y\n";
  surfaceEdgeFile << "property float z\n";
  surfaceEdgeFile << "property uchar red\n";
  surfaceEdgeFile << "property uchar green\n";
  surfaceEdgeFile << "property uchar blue\n";
  surfaceEdgeFile << "property float nx\n";
  surfaceEdgeFile << "property float ny\n";
  surfaceEdgeFile << "property float nz\n";
  surfaceEdgeFile << "end_header\n";

  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    if (this->mapGrids[i].isSurfaceEdge && this->mapGrids[i].reachable) {
      this->getGridIndex(i, xIndex, yIndex, zIndex);
      this->gridIndexToPosition(xIndex, yIndex, zIndex, x, y, z);
      surfaceEdgeFile << x << " " << y << " " << z << " " << 10 << " " << 80 << " " 
      << int(255 * (zIndex - this->zIndexMin) / (zIndexMax - zIndexMin)) << " "
      << this->mapGrids[i].normal->x() << " " << this->mapGrids[i].normal->y() << " " << this->mapGrids[i].normal->z() << "\n";
    }
  }
  
  surfaceEdgeFile.close();


  std::ofstream mapBoundaryFile(filepath + "/point_cloud_map_boundary.ply");
  if (!mapBoundaryFile) {
      std::cerr << "Can not open the point cloud file!" << std::endl;
      return;
  }

  point_num = 0;
  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    this->getGridIndex(i, xIndex, yIndex, zIndex);
    if (xIndex == this->xIndexMin || xIndex == this->xIndexMax ||
        yIndex == this->yIndexMin || yIndex == this->yIndexMax ||
        zIndex == this->zIndexMin || zIndex == this->zIndexMax
      ) {
      point_num++;
    }
  }

  mapBoundaryFile << "ply\n";
  mapBoundaryFile << "format ascii 1.0\n";
  mapBoundaryFile << "element vertex " << point_num << "\n";
  mapBoundaryFile << "property float x\n";
  mapBoundaryFile << "property float y\n";
  mapBoundaryFile << "property float z\n";
  mapBoundaryFile << "property uchar red\n";
  mapBoundaryFile << "property uchar green\n";
  mapBoundaryFile << "property uchar blue\n";
  mapBoundaryFile << "end_header\n";

  for (size_t i = 0; i < this->mapGrids.size(); i++) {
    this->getGridIndex(i, xIndex, yIndex, zIndex);
    if (xIndex == this->xIndexMin || xIndex == this->xIndexMax ||
        yIndex == this->yIndexMin || yIndex == this->yIndexMax ||
        zIndex == this->zIndexMin || zIndex == this->zIndexMax) {
      this->gridIndexToPosition(xIndex, yIndex, zIndex, x, y, z);
      
      mapBoundaryFile << x << " " << y << " " << z << " " 
      << (this->mapGrids[i].state == GridState::UNKNOWN ? 255:0) << " " 
      << (this->mapGrids[i].state == GridState::FREE ? 255:0) << " " 
      << (this->mapGrids[i].state == GridState::OCCUPIED ? 255:0) << "\n";
    }
  }
  
  mapBoundaryFile.close();


  std::cout << "All finished." << std::endl;
}