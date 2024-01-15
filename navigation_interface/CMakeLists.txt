cmake_minimum_required(VERSION 3.5)
project(navigation_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  interfaces_msgs
  serial
)

add_compile_options(-std=c++11)
add_definitions(-std=c++11)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf2_geometry_msgs tf2 serial interfaces_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(nav_interface src/main.cpp src/navigation_interface.cpp)
add_dependencies(nav_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_interface ${catkin_LIBRARIES})
