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

void Grid::addPoint(const Eigen::Vector3f& point) {
    points.push_back(point);
    if (points.size() > maxPoints) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(points.begin(), points.end(), gen);
        points.resize(maxPoints);
    }
}

void Grid::computeNormals(int slidingWindowSize) {
    normals.clear();

    int nPoints = points.size();
    if (nPoints == 0) {
        std::cerr << "No points available to compute normals in grid." << std::endl;
        return;
    }

    std::vector<float> curvatures;
    std::vector<Eigen::Vector3f> computedNormals;

    for (int i = 0; i < nPoints; ++i) {
        int startIdx = std::max(0, i - slidingWindowSize / 2);
        int endIdx = std::min(nPoints - 1, i + slidingWindowSize / 2);

        std::vector<Eigen::Vector3f> windowPoints(points.begin() + startIdx, points.begin() + endIdx + 1);
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& p : windowPoints) {
            centroid += p;
        }
        centroid /= windowPoints.size();

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (const auto& p : windowPoints) {
            Eigen::Vector3f centered = p - centroid;
            covariance += centered * centered.transpose();
        }

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
        Eigen::Vector3f normal = solver.eigenvectors().col(0); // Normal is eigenvector with smallest eigenvalue
        normal.normalize();
        computedNormals.push_back(normal);

        Eigen::Vector3f eigenvalues = solver.eigenvalues();
        float lambda0 = eigenvalues(0); // Smallest eigenvalue
        float lambda1 = eigenvalues(1);
        float lambda2 = eigenvalues(2);
        float curvature = lambda0 / (lambda0 + lambda1 + lambda2);
        curvatures.push_back(curvature);
    }


    //Hessian approximation starts
    if (computedNormals.size() > maxNormals) {
        float curvatureSum = std::accumulate(curvatures.begin(), curvatures.end(), 0.0f);
        if (curvatureSum > 0) {
            for (auto& curvature : curvatures) {
                curvature /= curvatureSum;
            }
        } else {
            float equalWeight = 1.0f / curvatures.size();
            for (auto& curvature : curvatures) {
                curvature = equalWeight;
            }
        }

        std::discrete_distribution<size_t> distribution(curvatures.begin(), curvatures.end());
        
        //Sample normals based on the distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::set<size_t> selectedIndices;
        while (selectedIndices.size() < static_cast<size_t>(maxNormals)) {
            size_t index = distribution(gen);
            selectedIndices.insert(index);
        }

        normals.reserve(maxNormals);
        for (size_t index : selectedIndices) {
            normals.push_back(computedNormals[index]);
        }
    } else {
        normals = computedNormals;
    }
}

void Grid::buildOctree(float gridSize, float originX, float originY, float originZ) {
    if (octree != nullptr) {
        delete octree;
    }
    Eigen::Vector3f minBound, maxBound;
    minBound.x() = xIndex * gridSize + originX;
    minBound.y() = yIndex * gridSize + originY;
    minBound.z() = zIndex * gridSize + originZ;
    maxBound = minBound + Eigen::Vector3f(gridSize, gridSize, gridSize);
    octree = new VoxelOctree(minBound, maxBound);
    octree->build(points, normals, maxOctreeResolution);
    float normalEpsilon = 0.1f;
    for(int i = 0; i < 5; i++){
        octree->mergeNodes(normalEpsilon);
    }
    
}
