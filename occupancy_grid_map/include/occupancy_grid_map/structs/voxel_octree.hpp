#ifndef VOXEL_OCTREE_HPP
#define VOXEL_OCTREE_HPP

#include <vector>
#include <Eigen/Core>

class VoxelOctreeNode {
public:
    Eigen::Vector3f minBound;
    Eigen::Vector3f maxBound;
    Eigen::Vector3f point;   // Single point per leaf node
    Eigen::Vector3f normal;  // Single normal per leaf node
    VoxelOctreeNode* children[8]; // Octree has 8 children
    bool isLeaf;

    VoxelOctreeNode(const Eigen::Vector3f& minBound, const Eigen::Vector3f& maxBound);
    ~VoxelOctreeNode();

    void subdivide(int maxResolution, int currentResolution, const std::vector<Eigen::Vector3f>& points,
                   const std::vector<Eigen::Vector3f>& normals);

    void averagePointsAndNormals(const std::vector<Eigen::Vector3f>& points,
                                 const std::vector<Eigen::Vector3f>& normals);

    bool mergeNodes(float normalEpsilon);

    void collectLeafNodes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals);
    uint32_t countLeafNodes() const;
};

class VoxelOctree {
public:
    VoxelOctreeNode* root;

    VoxelOctree(const Eigen::Vector3f& minBound, const Eigen::Vector3f& maxBound);
    ~VoxelOctree();

    void build(const std::vector<Eigen::Vector3f>& points, const std::vector<Eigen::Vector3f>& normals, int maxResolution);
    void mergeNodes(float normalEpsilon);

    void collectLeafNodes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals);
    uint32_t countLeafNodes() const;
};

#endif // VOXEL_OCTREE_HPP
