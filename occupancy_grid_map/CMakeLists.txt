

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(occupancy_grid_map_lib
  src/grid.cpp
  src/ogm.cpp
)

## Add gtest based cpp test target and link libraries
add_executable(occupancy_grid_map-test tests/test_ogm.cpp)
target_link_libraries(occupancy_grid_map-test occupancy_grid_map_lib)
add_test(NAME TestOGMGridTravel
         COMMAND occupancy_grid_map-test)
