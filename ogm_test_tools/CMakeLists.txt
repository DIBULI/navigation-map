set(CMAKE_CXX_STANDARD 17)         

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(
  ${OGM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


## Add gtest based cpp test target and link libraries
add_executable(ogm-test src/main.cpp)
target_link_libraries(ogm-test occupancy_grid_map_lib ${PCL_LIBRARIES})