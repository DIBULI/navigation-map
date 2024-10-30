#include "occupancy_grid_map/structs/voxel_octree.hpp"
#include <algorithm>
#include <iostream>
#include <cmath> // For std::cos

VoxelOctreeNode::VoxelOctreeNode(const Eigen::Vector3f& minBound, const Eigen::Vector3f& maxBound)
    : minBound(minBound), maxBound(maxBound), isLeaf(true) {
    std::fill(std::begin(children), std::end(children), nullptr);
}

VoxelOctreeNode::~VoxelOctreeNode() {
    for (auto& child : children) {
        if (child != nullptr) {
            delete child;
            child = nullptr;
        }
    }
}

void VoxelOctreeNode::subdivide(int maxResolution, int currentResolution,
                                const std::vector<Eigen::Vector3f>& points,
                                const std::vector<Eigen::Vector3f>& normals) {
    if (points.size() <= 1) {
        isLeaf = true;
        if (!points.empty()) {
            Eigen::Vector3f p = points.front();

            // Clamp p
            for (int axis = 0; axis < 3; ++axis) {
                p[axis] = std::max(minBound[axis], std::min(p[axis], maxBound[axis]));
            }

            point = p;
            normal = normals.front();
        }
        return;
    }

    if (currentResolution >= maxResolution) {
        // Maximum resolution reached, average points and normals
        isLeaf = true;
        averagePointsAndNormals(points, normals);
        return;
    }
    isLeaf = false;
    Eigen::Vector3f center = (minBound + maxBound) / 2.0f;
    std::vector<Eigen::Vector3f> childPoints[8];
    std::vector<Eigen::Vector3f> childNormals[8];
    for (size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3f& point = points[i];
        const Eigen::Vector3f& normal = normals[i];
        int index = 0;
        if (point.x() >= center.x()) index |= 1;
        if (point.y() >= center.y()) index |= 2;
        if (point.z() >= center.z()) index |= 4;
        childPoints[index].push_back(point);
        childNormals[index].push_back(normal);
    }

    for (int i = 0; i < 8; ++i) {
        if (!childPoints[i].empty()) {
            Eigen::Vector3f childMin = minBound;
            Eigen::Vector3f childMax = maxBound;

            for (int axis = 0; axis < 3; ++axis) {
                int axis_bit = 1 << axis;
                if ((i & axis_bit) == 0) {
                    // Lower half along this axis
                    childMax[axis] = center[axis];
                } else {
                    // Upper half along this axis
                    childMin[axis] = center[axis];
                }
            }

            children[i] = new VoxelOctreeNode(childMin, childMax);
            children[i]->subdivide(maxResolution, currentResolution + 1, childPoints[i], childNormals[i]);
        }
    }
}

void VoxelOctreeNode::averagePointsAndNormals(const std::vector<Eigen::Vector3f>& points,
                                              const std::vector<Eigen::Vector3f>& normals) {
    Eigen::Vector3f avgPoint = Eigen::Vector3f::Zero();
    for (const auto& p : points) {
        avgPoint += p;
    }
    avgPoint /= static_cast<float>(points.size());

    for (int axis = 0; axis < 3; ++axis) {
        avgPoint[axis] = std::max(minBound[axis], std::min(avgPoint[axis], maxBound[axis]));
    }

    point = avgPoint;
    Eigen::Vector3f avgNormal = Eigen::Vector3f::Zero();
    for (const auto& n : normals) {
        avgNormal += n;
    }
    if (avgNormal.norm() > 0.0f) {
        avgNormal.normalize();
        normal = avgNormal;
    }
}


bool VoxelOctreeNode::mergeNodes(float normalEpsilon) {
    if (isLeaf) {
        return true;
    }

    bool canAllChildrenBeMerged = true;
    for (int i = 0; i < 8; ++i) {
        if (children[i] != nullptr) {
            bool childIsLeafAfterMerge = children[i]->mergeNodes(normalEpsilon);
            if (!childIsLeafAfterMerge) {
                canAllChildrenBeMerged = false;
            }
        }
    }

    if (!canAllChildrenBeMerged) {
        return false;
    }

    Eigen::Vector3f avgNormal = Eigen::Vector3f::Zero();
    int numChildrenWithNormals = 0;

    Eigen::Vector3f referenceNormal;
    bool referenceSet = false;
    for (int i = 0; i < 8; ++i) {
        if (children[i] != nullptr && children[i]->normal != Eigen::Vector3f::Zero()) {
            if (!referenceSet) {
                referenceNormal = children[i]->normal;
                referenceSet = true;
            } else {
                if (referenceNormal.dot(children[i]->normal) < 0) {
                    children[i]->normal = -children[i]->normal;
                }
            }
        }
    }

    for (int i = 0; i < 8; ++i) {
        if (children[i] != nullptr && children[i]->normal != Eigen::Vector3f::Zero()) {
            avgNormal += children[i]->normal;
            numChildrenWithNormals++;
        }
    }

    if (numChildrenWithNormals == 0) {
        return false;
    }

    avgNormal.normalize();

    float cosEpsilon = std::cos(normalEpsilon);

    bool canMerge = true;
    for (int i = 0; i < 8; ++i) {
        if (children[i] != nullptr && children[i]->normal != Eigen::Vector3f::Zero()) {
            float dotProduct = avgNormal.dot(children[i]->normal);
            dotProduct = std::max(-1.0f, std::min(1.0f, dotProduct));
            if (dotProduct < cosEpsilon) {
                canMerge = false;
                break;
            }
        }
    }

    if (canMerge) {
        Eigen::Vector3f avgPoint = Eigen::Vector3f::Zero();
        int numPoints = 0;
        for (int i = 0; i < 8; ++i) {
            if (children[i] != nullptr) {
                avgPoint += children[i]->point;
                numPoints++;
            }
        }
        avgPoint /= static_cast<float>(numPoints);
        point = avgPoint;
        normal = avgNormal;

        for (int i = 0; i < 8; ++i) {
            if (children[i] != nullptr) {
                delete children[i];
                children[i] = nullptr;
            }
        }
        isLeaf = true;
        return true; // Node is now a leaf
    } else {
        return false; // Cannot merge this node
    }
}

VoxelOctree::VoxelOctree(const Eigen::Vector3f& minBound, const Eigen::Vector3f& maxBound) {
    root = new VoxelOctreeNode(minBound, maxBound);
}

VoxelOctree::~VoxelOctree() {
    delete root;
}

void VoxelOctree::build(const std::vector<Eigen::Vector3f>& points,
                        const std::vector<Eigen::Vector3f>& normals, int maxResolution) {
    if (points.size() != normals.size()) {
        std::cout << "numPoints: " << points.size() << std::endl;
        std::cout << "numNormals: " << normals.size() << std::endl;
        std::cerr << "Points and normals size mismatch in octree build." << std::endl;
        return;
    }
    root->subdivide(maxResolution, 0, points, normals);
}

void VoxelOctree::mergeNodes(float normalEpsilon) {
    if (root != nullptr) {
        root->mergeNodes(normalEpsilon);
    }
}

void VoxelOctreeNode::collectLeafNodes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) {
    if (isLeaf) {
        points.push_back(point);
        normals.push_back(normal);
    } else {
        for (auto* child : children) {
            if (child != nullptr) {
                child->collectLeafNodes(points, normals);
            }
        }
    }
}

uint32_t VoxelOctreeNode::countLeafNodes() const {
    if (isLeaf) {
        return 1;
    } else {
        uint32_t count = 0;
        for (const auto* child : children) {
            if (child != nullptr) {
                count += child->countLeafNodes();
            }
        }
        return count;
    }
}

void VoxelOctree::collectLeafNodes(std::vector<Eigen::Vector3f>& points, std::vector<Eigen::Vector3f>& normals) {
    if (root != nullptr) {
        root->collectLeafNodes(points, normals);
    }
}

uint32_t VoxelOctree::countLeafNodes() const {
    if (root != nullptr) {
        return root->countLeafNodes();
    }
    return 0;
}
