#include "occupancy_grid_map/structs/surface_cluster.hpp"
#include "occupancy_grid_map/structs/grid.hpp"

SurfaceCluster::SurfaceCluster() {}


SurfaceCluster::~SurfaceCluster() {}


bool SurfaceCluster::containsSurface(int x, int y, int z) {
  
}

bool SurfaceCluster::containsEdge(int x, int y, int z) {
  
}

void merge_two_surfaces_map(std::map<int, std::map<int, std::map<int, Grid*>>> &destination, 
  std::map<int, std::map<int, std::map<int, Grid*>>> &source,
  SurfaceCluster* main) {
  for (auto &outer_pair : source) {
    int outer_key = outer_pair.first;
    auto &source_mid_map = outer_pair.second;

    if (destination.find(outer_key) == destination.end()) {
      destination[outer_key] = source_mid_map;
    } else {
      auto &destination_mid_map = destination[outer_key];

      for (auto &mid_pair : source_mid_map) {
        int mid_key = mid_pair.first;
        auto &source_inner_map = mid_pair.second;

        if (destination_mid_map.find(mid_key) == destination_mid_map.end()) {
          destination_mid_map[mid_key] = source_inner_map;
        } else {
          auto &destination_inner_map = destination_mid_map[mid_key];

          for (auto &inner_pair : source_inner_map) {
            int inner_key = inner_pair.first;
            Grid* grid_ptr = inner_pair.second;

            grid_ptr->surface_cluster = main;

            destination_inner_map[inner_key] = grid_ptr;
          }
        }
      }
    }
  }
}

void SurfaceCluster::mergeTwoSurfaceClusters(SurfaceCluster* main, SurfaceCluster* to_be_merged) {
  main->surfacesNumber += to_be_merged->surfacesNumber;
  main->surfaceEdgeNumber += to_be_merged->surfaceEdgeNumber;

  merge_two_surfaces_map(main->surfaceEdges, to_be_merged->surfaceEdges, main);
  main->surfaceGrids.insert(main->surfaceGrids.end(), to_be_merged->surfaceGrids.begin(), to_be_merged->surfaceGrids.end());
  for (auto &grid : to_be_merged->surfaceGrids) {
    grid->surface_cluster = main;
  }
}

void SurfaceCluster::add_surface(int x, int y, int z, Grid *grid) {
  this->surfacesNumber++;
  surfaceGrids.push_back(grid);
}

void SurfaceCluster::add_surface_edge(int x, int y, int z, Grid *grid) {
  this->surfaceEdgeNumber++;
  this->surfaceEdges[x][y][z] = grid;
}

void SurfaceCluster::remove_surface_edge(int x, int y, int z) {
  this->surfaceEdgeNumber--;
  if (this->surfaceEdges.find(x) != this->surfaceEdges.end()) {
    auto x_map = this->surfaceEdges[x];
    if (x_map.find(y) != x_map.end()) {
      auto y_map = this->surfaceEdges[x][y];
      y_map.erase(z);
    };
  }
}