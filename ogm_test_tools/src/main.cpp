#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <Eigen/Geometry>
#include <pcl/io/ply_io.h>

#include "occupancy_grid_map/ogm.hpp"

std::vector<std::string> getFilesInDirectory(const std::string& directory) {
  std::vector<std::string> files;
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".ply") {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

void loadPosesFromCSV(const std::string& filepath, std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> &poses) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    std::cerr << "Unable to open pose file: " << filepath << std::endl;
    return;
  }

  std::string line;
  bool first_line = true;
  while (std::getline(file, line)) {
    if (first_line) {
      first_line = false;  // Skip the first line
      continue;
    }
    std::istringstream ss(line);
    std::string item;
    std::vector<float> numbers;

    while (std::getline(ss, item, ',')) {
      std::istringstream itemStream(item);
      float number;
      if (itemStream >> number) {
        numbers.push_back(number);
      } else {
        std::cerr << "Error parsing number: " << item << std::endl;
      }
    }
    Eigen::Quaternionf pose(numbers[7], numbers[4], numbers[5], numbers[6]);
    poses.push_back(std::make_pair(pose, Eigen::Vector3f(numbers[1], numbers[2], numbers[3])));
  }
}

int main(int argc, char **argv) {
  if (argc != 5) {
    std::cerr << "Usage: " << argv[0] << "<pose file path> <point cloud ply files> <ogm_output_path> <is_under_camera_frame_pose>" << std::endl;
    return -1;
  }

  std::string posefile_path = argv[1];
  std::string pc_path = argv[2];
  std::string ogm_output_path = argv[3];
  bool is_under_camera_frame_pose = std::stoi(argv[4]);

  std::vector<std::string> pc_files = getFilesInDirectory(pc_path);
  std::vector<std::pair<Eigen::Quaternionf, Eigen::Vector3f>> poses;
  loadPosesFromCSV(posefile_path, poses);

  std::cout << "Number of pointcloud files: " << pc_files.size() << std::endl;
  std::cout << "Number of poses: " << poses.size() << std::endl;

  if (pc_files.size() != poses.size() ) {
    std::cerr << "The number of pointcloud files does not match the number of poses!" << std::endl;
    return -1;
  }

  OccupancyGridMap ogm(0.0f, 0.0f, 0.0f, 0.02f, -5.5, 5.5, -5.5, 5.5, -1.0f, 4.0f);

  int pc_files_index = 0;
  for (auto& pc_filepath : pc_files) {
    std::cout << "Process index: " << pc_files_index << ", " << pc_filepath << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPLYFile(pc_filepath, *cloud);

    std::vector<std::tuple<float, float, float>> points;
    Eigen::Quaternionf q_T = poses[pc_files_index].first;
    Eigen::Vector3f t = poses[pc_files_index].second;
    for (size_t i = 0; i < cloud->points.size(); ++i) {
      if (is_under_camera_frame_pose == 0) {
        float x = cloud->points[i].x;
        float y = cloud->points[i].y;
        float z = cloud->points[i].z;
        points.push_back(std::make_tuple(x, y, z));
      } else {
        Eigen::Vector3f point;
        point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
        point = q_T * point + t;
        points.push_back(std::make_tuple(point.x(), point.y(), point.z()));
      }
    }

    if (is_under_camera_frame_pose == 0) {
      ogm.updateMap(0, 0, 0, points);
    } else {
      ogm.updateMap(t.x(), t.y(), t.z(), points);
    }
    
    pc_files_index++;
  }
  ogm.outputAsPointCloud(ogm_output_path);
}