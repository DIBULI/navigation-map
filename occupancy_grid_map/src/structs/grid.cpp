#include "occupancy_grid_map/structs/grid.hpp"

Grid::Grid(): surface_cluster(nullptr) {}

Grid::~Grid() {}

void Grid::hit() {
  occupancyProbabilityLog += hitUpdateConstant;
  occupancyProbabilityLog = std::fmin(lmax, std::fmax(lmin, occupancyProbabilityLog));
}

void Grid::miss() {
  occupancyProbabilityLog += missUpdateConstant;
  occupancyProbabilityLog = std::fmin(lmax, std::fmax(lmin, occupancyProbabilityLog));
}

bool Grid::isOccupied() {
  return occupancyProbabilityLog > 40;
}

bool Grid::isFree() {
  return occupancyProbabilityLog < -4;
}