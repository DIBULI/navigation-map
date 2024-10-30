// #include <string>
// #include <fstream>
// #include <iostream>
// #include <filesystem>
// #include <thread>
// #include <mutex>

// #include <Eigen/Geometry>
// #include <pcl/io/ply_io.h>

// #include "occupancy_grid_map/ogm.hpp"

// std::vector<std::string> getFilesInDirectory(const std::string& directory) {
//   std::vector<std::string> files;
//   for (const auto& entry : std::filesystem::directory_iterator(directory)) {
//     if (entry.is_regular_file() && entry.path().extension() == ".ply") {
//       files.push_back(entry.path().string());
//     }
//   }
//   std::sort(files.begin(), files.end());
//   return files;
// }

// void loadPosesFromCSV(const std::string& filepath, std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> &poses) {
//   std::ifstream file(filepath);
//   if (!file.is_open()) {
//     std::cerr << "Unable to open pose file: " << filepath << std::endl;
//     return;
//   }

//   std::string line;
//   bool first_line = true;
//   while (std::getline(file, line)) {
//     if (first_line) {
//       first_line = false;  // Skip the first line
//       continue;
//     }
//     std::istringstream ss(line);
//     std::string item;
//     std::vector<float> numbers;

//     while (std::getline(ss, item, ',')) {
//       std::istringstream itemStream(item);
//       float number;
//       if (itemStream >> number) {
//         numbers.push_back(number);
//       } else {
//         std::cerr << "Error parsing number: " << item << std::endl;
//       }
//     }
//     Eigen::Quaternionf pose(numbers[7], numbers[4], numbers[5], numbers[6]);
//     pose.normalize();
//     poses.push_back(std::make_pair(pose, Eigen::Vector3f(numbers[1], numbers[2], numbers[3])));
//   }
// }

// std::mutex load_mutex;
// void loadPLYFile(int pc_files_index, std::string pc_filepath, std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> poses, 
//   std::vector<std::vector<std::tuple<float, float, float>>> &points,
//   bool is_under_camera_frame_pose) {
//   {
//     std::lock_guard<std::mutex> guard(load_mutex);
//     points.push_back(std::vector<std::tuple<float, float, float>>());
//   }
  
//   std::cout << "Loading point cloud files, index: " << pc_files_index << ", " << pc_filepath << std::endl;
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::io::loadPLYFile(pc_filepath, *cloud);
//   Eigen::Quaternionf q_T = poses[pc_files_index].first;
//   Eigen::Vector3f t = poses[pc_files_index].second;
//   for (size_t i = 0; i < cloud->points.size(); ++i) {
//     if (is_under_camera_frame_pose == 0) {
//       float x = cloud->points[i].x;
//       float y = cloud->points[i].y;
//       float z = cloud->points[i].z;
//       points[pc_files_index].push_back(std::make_tuple(x, y, z));
//     } else {
//       Eigen::Vector3f point;
//       point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
//       point = q_T * point + t;
//       points[pc_files_index].push_back(std::make_tuple(point.x(), point.y(), point.z()));
//     }
//   }
// }

// int main(int argc, char **argv) {
//   if (argc != 7) {
//     std::cerr << "Usage: " << argv[0] << "<pose file path> <point cloud ply files> <ogm_output_path> <is_under_camera_frame_pose> <resolution> <resize>" << std::endl;
//     return -1;
//   }

//   std::string posefile_path = argv[1];
//   std::string pc_path = argv[2];
//   std::string ogm_output_path = argv[3];
//   bool is_under_camera_frame_pose = std::stoi(argv[4]);
//   float resolution = std::stof(argv[5]);
//   int resize_num = std::stoi(argv[6]);

//   std::vector<std::string> pc_files = getFilesInDirectory(pc_path);
//   std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> poses;
//   loadPosesFromCSV(posefile_path, poses);

//   pc_files.resize(resize_num);
//   poses.resize(resize_num);

//   std::cout << "Number of pointcloud files: " << pc_files.size() << std::endl;
//   std::cout << "Number of poses: " << poses.size() << std::endl;

//   if (pc_files.size() != poses.size() ) {
//     std::cerr << "The number of pointcloud files does not match the number of poses!" << std::endl;
//     return -1;
//   }

//   OccupancyGridMap ogm(0.0f, 0.0f, 0.0f, resolution, -10, 10, -10, 10, -1.0f, 6.0f);
//   std::vector<std::thread> threads;

//   int pc_files_index = 0;
//   std::vector<std::vector<std::tuple<float, float, float>>> points(pc_files.size());
//   for (std::string pc_filepath : pc_files) {
//     threads.emplace_back(std::thread(loadPLYFile, pc_files_index, pc_filepath, poses, std::ref(points), is_under_camera_frame_pose));
//     pc_files_index++;
//   }

//   for (auto& t : threads) {
//     if (t.joinable()) {
//         t.join();
//     }
//   }

//   pc_files_index = 0;
//   for (auto& pc_filepath : pc_files) {
//     std::cout << "MapLoading point cloud files, index: " << pc_files_index << ", " << pc_filepath << std::endl;
//     Eigen::Quaternionf q_T = poses[pc_files_index].first;
//     Eigen::Vector3f t = poses[pc_files_index].second;
//     if (is_under_camera_frame_pose == 0) {
//       ogm.updateMap(0, 0, 0, points[pc_files_index]);
//     } else {
//       ogm.updateMap(t.x(), t.y(), t.z(), points[pc_files_index]);
//     }

//     std::cout << "Drone position:" << t.x() << " " << t.y() << " " << t.z() << std::endl; 
//     std::cout << "Drone pose:" << q_T.w() << " " << q_T.x() << " " << q_T.y() << " " << q_T.z() << std::endl; 

//     std::cout << "OGM - free: " << ogm.freeGridNum << ", occupied: " << ogm.occupiedGridNum << ", unknown: " << ogm.unknownGridNum << std::endl; 
//     std::cout << "OGM - surfaces: " << ogm.surfaceOperator.surface_clusters.size() << std::endl; 

//     pc_files_index++;
//   }
  
//   ogm.outputAsPointCloud(ogm_output_path);
// }


// visualize_octree.cpp
#include <iostream>
#include <string>
#include <vector>
#include <tuple>
#include <thread>
#include <chrono>

#include <Eigen/Core>

#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "occupancy_grid_map/ogm.hpp"

void visualizeOctreeNode(VoxelOctreeNode* node, pcl::visualization::PCLVisualizer::Ptr viewer, int& id_counter) {
    if (node == nullptr) return;

    std::string box_id = "octree_node_" + std::to_string(id_counter++);
    viewer->addCube(node->minBound.x(), node->maxBound.x(),
                    node->minBound.y(), node->maxBound.y(),
                    node->minBound.z(), node->maxBound.z(),
                    1.0, 0.0, 0.0, box_id);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, 0.5, box_id);

    if (!node->isLeaf) {
        for (int i = 0; i < 8; ++i) {
            if (node->children[i] != nullptr) {
                visualizeOctreeNode(node->children[i], viewer, id_counter);
            }
        }
    }
}

void collectOctreeLeafNodes(VoxelOctreeNode* node, std::vector<Eigen::Vector3f>& leaf_points, std::vector<Eigen::Vector3f>& leaf_normals) {
    if (node == nullptr) return;
    if (node->isLeaf) {
        leaf_points.push_back(node->point);
        leaf_normals.push_back(node->normal);
    } else {
        for (int i = 0; i < 8; ++i) {
            if (node->children[i] != nullptr) {
                collectOctreeLeafNodes(node->children[i], leaf_points, leaf_normals);
            }
        }
    }
}

int main(int argc, char **argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <PLY file path>" << std::endl;
        return -1;
    }

    std::string ply_file_path = argv[1];

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPLYFile<pcl::PointXYZ>(ply_file_path, *cloud) == -1) {
        PCL_ERROR("Couldn't read PLY file \n");
        return (-1);
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << ply_file_path << std::endl;

    float mapMinX = -10.0f;
    float mapMaxX = 10.0f;
    float mapMinY = -10.0f;
    float mapMaxY = 10.0f;
    float mapMinZ = -1.0f;
    float mapMaxZ = 6.0f;
    float gridSize = 0.5f;
    OccupancyGridMap ogm(0.0f, 0.0f, 0.0f, gridSize,
                         mapMinX, mapMaxX, mapMinY, mapMaxY, mapMinZ, mapMaxZ);

    std::vector<std::tuple<float, float, float>> grids;
    for (auto &p : cloud->points) {
        grids.push_back(std::make_tuple(p.x, p.y, p.z));
    }

    // Update the map
    ogm.updateMap(0.0f, 0.0f, 0.0f, grids);

    ogm.outputDownsampledPointCloud(".");

    // Create PCL Visualizer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("OGM Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    int octree_id_counter = 0;
    for (size_t i = 0; i < ogm.mapGrids.size(); i++) {
        Grid &grid = ogm.mapGrids[i];
        if (grid.isOccupied() && grid.octree != nullptr) {
            VoxelOctree *octree = grid.octree;
            visualizeOctreeNode(octree->root, viewer, octree_id_counter);
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_voxels(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < ogm.mapGrids.size(); i++) {
        Grid &grid = ogm.mapGrids[i];
        if (grid.isOccupied()) {
            for (auto &point : grid.points) {
                pcl::PointXYZRGB p;
                p.x = point.x();
                p.y = point.y();
                p.z = point.z();
                p.r = 255;
                p.g = 255;
                p.b = 255;
                cloud_in_voxels->points.push_back(p);
            }
        }
    }
    cloud_in_voxels->width = cloud_in_voxels->points.size();
    cloud_in_voxels->height = 1;
    cloud_in_voxels->is_dense = true;
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud_in_voxels, "cloud_in_voxels");
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals_points(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    for (size_t i = 0; i < ogm.mapGrids.size(); i++) {
        Grid &grid = ogm.mapGrids[i];
        if (grid.isOccupied() && grid.octree != nullptr) {
            std::vector<Eigen::Vector3f> leaf_points;
            std::vector<Eigen::Vector3f> leaf_normals;
            collectOctreeLeafNodes(grid.octree->root, leaf_points, leaf_normals);

            // Add points and normals to the point clouds
            for (size_t j = 0; j < leaf_points.size(); ++j) {
                pcl::PointXYZ p;
                p.x = leaf_points[j].x();
                p.y = leaf_points[j].y();
                p.z = leaf_points[j].z();
                normals_points->points.push_back(p);

                pcl::Normal n;
                n.normal_x = leaf_normals[j].x();
                n.normal_y = leaf_normals[j].y();
                n.normal_z = leaf_normals[j].z();
                normals->points.push_back(n);
            }
        }
    }

    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(normals_points, normals, 1, 0.5, "normals");

    // Start visualization loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}