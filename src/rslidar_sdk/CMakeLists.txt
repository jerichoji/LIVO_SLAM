cmake_minimum_required(VERSION 3.5)   # 编译（catkin）至少需要3.5版本以上的Cmake
cmake_policy(SET CMP0048 NEW)   # 使用 NEW 选项的 cmake_policy 命令明确告诉 CMake 使用策略的新行为。
project(rslidar_sdk)   # 通过project()这个函数指定包的名字，在CMake中指定后，你可在其他地方通过使用变量${PROJECT_NAME}来引用它

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)   # 编译环境设置，ros1 OR ros2的选择
#=======================================
set(COMPILE_METHOD CATKIN) 

#=======================================
# Custom Point Type (XYZI, XYZIRT)   # 点云类型
#=======================================
set(POINT_TYPE XYZIRT)

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)   # 项目名称
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")   # 
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-Wall)

#========================
# Dependencies Setup
#========================
#ROS#
find_package(roscpp 1.12 QUIET)   # 使用的程序语言为c++
if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)
#Protobuf#
find_package(Protobuf QUIET)   # 查找Protobuf依赖包
find_program(PROTOC protoc)
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/src/msg/proto_msg)
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  foreach(proto_file ${PROTOBUF_FILELIST})
    message(STATUS "COMPILING ${proto_file} USING ${PROTOBUF_PROTOC_EXECUTABLE}")
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    --proto_path=${PROTO_FILE_PATH}
                    --cpp_out=${PROTO_FILE_PATH}
                    ${proto_file})
  endforeach()
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
#Others#
find_package(yaml-cpp REQUIRED)   # yaml-ccp依赖，默认安装ros-desktop时已经安装好了
find_package(PCL QUIET REQUIRED)   # PCL库，需要自己安装，自选
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Catkin#
if(${COMPILE_METHOD} STREQUAL "CATKIN")
  find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
	tf
          )   # catkin必备的依赖包

  catkin_package(
          CATKIN_DEPENDS sensor_msgs
  )
endif(${COMPILE_METHOD} STREQUAL "CATKIN")
#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)
#Driver core#
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)   # 查找雷达驱动包
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")   # 四元数
add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")   # 六元数
add_definitions(-DPOINT_TYPE_XYZIRT)
endif()
message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Build Setup
#========================
#Protobuf#
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              src/manager/adapter_manager.cpp
              ${PROTO_FILE_PATH}/packet.pb.cc
              ${PROTO_FILE_PATH}/scan.pb.cc
              ${PROTO_FILE_PATH}/point_cloud.pb.cc
              )
  target_link_libraries(rslidar_sdk_node                   
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${PROTOBUF_LIBRARY}
                      ${rs_driver_LIBRARIES}
                      )
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              src/manager/adapter_manager.cpp
              )
  target_link_libraries(rslidar_sdk_node                   
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES}
                      )
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
#Ros#
if(roscpp_FOUND)
  target_link_libraries(rslidar_sdk_node ${roscpp_LIBRARIES})
endif(roscpp_FOUND)
#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  ament_target_dependencies(rslidar_sdk_node rclcpp sensor_msgs std_msgs rslidar_msg)
  install(TARGETS
    rslidar_sdk_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

add_executable(tf_broadcaster src/tf.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})
