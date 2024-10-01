#include "occupancy_grid_map/ogm.hpp"
#include "occupancy_grid_map/operators/surface_operator.hpp"

SurfaceOperator::SurfaceOperator(): surface_cluster_global_id(0) {
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) continue;
        if (dx == 0 || dy == 0 || dz == 0) {
          connectedNeighbours.push_back(std::make_tuple(dx, dy, dz));
        } 
        surrNeighbours.push_back(std::make_tuple(dx, dy, dz));
      }
    }
  }
}

SurfaceOperator::~SurfaceOperator() {}

void SurfaceOperator::grid_operator(Grid* grid, int x, int y, int z, OccupancyGridMap* gridMap) {
  int unknownNums = 0;
  int occupiedNums = 0;
  int surfaceNums = 0;
  int surfaceEdgeNums = 0;

  // check the surrounding connected neighbours
  int dx, dy, dz;
  std::set<SurfaceCluster *, SurfaceClusterDescendingOrder> surface_cluster_ptrs;
  for (int i = 0; i < connectedNeighbours.size(); i++) {
    std::tie(dx, dy, dz) = connectedNeighbours[i];
    Grid* neiGrid = gridMap->getNeightbourGrid(x, y, z, dx, dy, dz);
    if (neiGrid == grid) {
      continue;
    }
    unknownNums += neiGrid->state == GridState::UNKNOWN ? 1 : 0;
    occupiedNums += neiGrid->state == GridState::OCCUPIED ? 1 : 0;
    surfaceNums += neiGrid->isSurfaceVoxel ? 1 : 0;
    surfaceEdgeNums += neiGrid->isSurfaceEdge ? 1 : 0;
    if (neiGrid->surface_cluster != nullptr) {
      // find the connected surface cluster
      surface_cluster_ptrs.insert(neiGrid->surface_cluster);
    }
  }

  if (is_surface(unknownNums, occupiedNums, surfaceNums, surfaceEdgeNums)) { // is surface
    // if this voxel does not belong to any surface cluster
    // create a new surface cluster
    if (surface_cluster_ptrs.size() == 0 && grid->surface_cluster == nullptr) {
      this->create_new_surface_cluster(grid);
      grid->surface_cluster->add_surface(x, y, z, grid);
    }

    if (grid->isSurfaceEdge) { // if the voxel was surface edge
      grid->surface_cluster->remove_surface_edge(x, y, z);
    }

    grid->isSurfaceEdge = false;
    grid->isSurfaceVoxel = true;

    this->calNorm(x, y, z, grid, gridMap);
  } else if (is_edge(unknownNums, occupiedNums, surfaceNums, surfaceEdgeNums) && !grid->isSurfaceVoxel) { // is surface edge
    // assign the surface edge to the surface cluster
    if (surface_cluster_ptrs.size() == 0 && grid->surface_cluster == nullptr) {
      this->create_new_surface_cluster(grid);
      grid->surface_cluster->add_surface_edge(x, y, z, grid);
    }

    grid->isSurfaceEdge = true;
    grid->isSurfaceVoxel = false;

    if (x == gridMap->xIndexMax || x == gridMap->xIndexMin ||
        y == gridMap->yIndexMax || y == gridMap->yIndexMin ||
        z == gridMap->zIndexMax || z == gridMap->zIndexMin) {
      grid->reachable = false;
    } else {
      this->calNorm(x, y, z, grid, gridMap);
    }
  }
  
  if (surface_cluster_ptrs.size() > 0 && (grid->isSurfaceEdge || grid->isSurfaceVoxel)) { // two surfaces are connected, no matter if it's surface or edge
    // connect two surface clusters
    if (grid->surface_cluster != nullptr) { // add the current cluster and wait to be merged
      surface_cluster_ptrs.insert(grid->surface_cluster);
    }
    if (grid->isSurfaceVoxel && grid->surface_cluster == nullptr) {
      grid->surface_cluster = *(surface_cluster_ptrs.begin());
      grid->surface_cluster->add_surface(x, y, z, grid);
    }
    if (grid->isSurfaceEdge && grid->surface_cluster == nullptr) {
      grid->surface_cluster = *(surface_cluster_ptrs.begin());
      grid->surface_cluster->add_surface_edge(x, y, z, grid);
    }

    if (grid->isSurfaceEdge || grid->isSurfaceVoxel) {
      // merge all the surface clusters
      for (int j=1; j<surface_cluster_ptrs.size(); j++) {
        auto it = surface_cluster_ptrs.begin();
        std::advance(it, j);
        SurfaceCluster::mergeTwoSurfaceClusters(
          *surface_cluster_ptrs.begin(), 
          *it);
        surface_clusters.erase((*it)->id);
      }
    }
  }
}

SurfaceCluster* SurfaceOperator::create_new_surface_cluster(Grid* &grid) {
  SurfaceCluster *sc = new SurfaceCluster();
  sc->id = surface_cluster_global_id;
  surface_clusters[surface_cluster_global_id.load()] = sc;
  grid->surface_cluster = sc;
  ++surface_cluster_global_id;

  return sc;
}

bool SurfaceOperator::is_surface(int unknownNums, int occupiedNums, int surfaceNums, int surfaceEdgeNums) {
  return unknownNums == 0 && occupiedNums != 0 && occupiedNums != 6;
}

bool SurfaceOperator::is_edge(int unknownNums, int occupiedNums, int surfaceNums, int surfaceEdgeNums) {
  return unknownNums != 0 && occupiedNums != 0 && occupiedNums != 6 && surfaceNums != 0;
}

void SurfaceOperator::calNorm(int x, int y, int z, Grid* grid, OccupancyGridMap* gridMap) {
  // calculate the normal vector based on the surrounding 26 voxels
  float x_ax=0;
  float y_ax=0;
  float z_ax=0;

  int dx, dy, dz;

  for (int i=0; i<surrNeighbours.size(); i++) {
    std::tie(dx, dy, dz) = surrNeighbours[i];
    Grid* neiGrid = gridMap->getNeightbourGrid(x, y, z, dx, dy, dz);

    if (neiGrid->state == GridState::FREE) {
      x_ax += dx;
      y_ax += dy;
      z_ax += dz;
    }
  } 

  if (grid->normal == nullptr) {
    grid->normal = new Eigen::Vector3f();
  }
  *(grid->normal) << x_ax, y_ax, z_ax;
  grid->normal->normalize();
}
