project(navigation_map_ros_adapter)
find_package(catkin REQUIRED roscpp pcl_ros sensor_msgs nav_msgs)

set(CMAKE_CXX_STANDARD 14)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES navigation_map_ros_adapter
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${OGM_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_vins_node src/navigation_map_ros_adapter_vins_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_vins_node
  ${catkin_LIBRARIES}
  occupancy_grid_map_lib
)
