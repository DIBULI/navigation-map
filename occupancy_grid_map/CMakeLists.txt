cmake_minimum_required(VERSION 3.10)
project(OccupancyGridMap)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable debugging symbols if not set
if (CMAKE_BUILD_TYPE STREQUAL "")
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type: Debug, Release, etc." FORCE)
endif()

# Find and include Eigen3
find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Set the include directory for OGM
set(OGM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE STRING "Path of the OGM include directories")
include_directories(${OGM_INCLUDE_DIR})

# Add the occupancy grid map library
add_library(occupancy_grid_map_lib
  src/structs/grid.cpp
  src/structs/voxel_octree.cpp
  src/structs/surface_cluster.cpp
  src/operators/surface_operator.cpp
  src/ogm.cpp
)

# Link Eigen3 to the library if necessary
target_link_libraries(occupancy_grid_map_lib Eigen3::Eigen)

# Add the GTest executable and link libraries
add_executable(occupancy_grid_map-test tests/test_ogm.cpp)
target_link_libraries(occupancy_grid_map-test occupancy_grid_map_lib)

# Set up GTest to run as part of the build
enable_testing()
add_test(NAME TestOGMGridTravel
         COMMAND occupancy_grid_map-test)
