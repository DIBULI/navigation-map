#ifndef SURFACE_CLUSTER_HPP
#define SURFACE_CLUSTER_HPP

#include <map>
#include <set>
#include <vector>
#include <atomic>
#include <iostream>

class Grid;


class SurfaceCluster {
public:
  SurfaceCluster();
  ~SurfaceCluster();

  bool containsSurface(int x, int y, int z);
  bool containsEdge(int x, int y, int z);

  int id;
  uint32_t surfacesNumber = 0;
  uint32_t surfaceEdgeNumber = 0;

  std::set<Grid*> surfaceGrids;
  std::map<int, std::map<int, std::map<int, Grid*>>> surfaceEdges;

  static void mergeTwoSurfaceClusters(SurfaceCluster* main, SurfaceCluster* to_be_merged);

  void add_surface(int x, int y, int z, Grid* grid);
  void add_surface_edge(int x, int y, int z, Grid* grid);

  void remove_surface_edge(int x, int y, int z);
};


struct SurfaceClusterAscendingOrder {
  bool operator() (const SurfaceCluster *a, const SurfaceCluster *b) const {
    return a->id < b->id;  // Sort in ascending order
  }
};

#endif /* SURFACE_CLUSTER_HPP */
