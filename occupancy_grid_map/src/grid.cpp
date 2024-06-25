#include "occupancy_grid_map/grid.hpp"

Grid::Grid() {}

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