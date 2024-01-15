cmake_minimum_required(VERSION 2.8.3)
project(udp_sender_node)
ADD_COMPILE_OPTIONS(-std=c++14)

#find and include eigen3

find_package(Eigen3 REQUIRED)
message(Eigen: ${EIGEN3_INCLUDE_DIR})

# Find the catkin packages
find_package(catkin REQUIRED
  roscpp
  interfaces_msgs
  geometry_msgs
)

# Declare the package
#catkin_package()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp interfaces_msgs
)

# Specify the include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Add the UDP sender node executable
add_executable(udp_sender_node src/main.cpp)

# Link the UDP sender node with catkin libraries
target_link_libraries(udp_sender_node
  ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIR}
)

# Install the executable
install(TARGETS udp_sender_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)