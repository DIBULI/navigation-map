find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

set(OGM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE STRING "Path of the OGM include directories")

## Declare a C++ library
add_library(occupancy_grid_map_lib
  src/structs/grid.cpp
  src/structs/surface_cluster.cpp
  src/operators/surface_operator.cpp
  src/ogm.cpp
)

## Add gtest based cpp test target and link libraries
add_executable(occupancy_grid_map-test tests/test_ogm.cpp)
target_link_libraries(occupancy_grid_map-test occupancy_grid_map_lib)
add_test(NAME TestOGMGridTravel
         COMMAND occupancy_grid_map-test)