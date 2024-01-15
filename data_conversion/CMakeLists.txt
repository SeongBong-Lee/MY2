cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(data_conversion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  interfaces_msgs
  std_msgs
  pcl_conversions  # Add pcl_conversions package here
)



# Add the following line to find the YAML library
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs interfaces_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}  # Add this line to include YAML headers
)

# add_executable(initialpose_publisher src/initialpose_publisher.cpp)
add_executable(data_conversion src/data_conversion.cpp)

target_link_libraries(data_conversion
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES}  # Add this line to link against YAML library
  ${PCL_CONVERSIONS_LIBRARIES}  # Link against pcl_conversions library
  ${EIGEN3_INCLUDE_DIR}
)

install(TARGETS data_conversion
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
