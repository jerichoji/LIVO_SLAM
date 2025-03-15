cmake_minimum_required(VERSION 3.0.2)
project(rslidar_to_livox)

set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    livox_ros_driver
)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(rslidar_to_livox_node src/rslidar_to_livox.cpp)
target_link_libraries(rslidar_to_livox_node ${catkin_LIBRARIES})