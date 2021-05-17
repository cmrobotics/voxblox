cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(voxblox REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(PCL REQUIRED)
find_package(voxblox_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)



#find_package(minkindr_conversions REQUIRED)


set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )



include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIRS}
  ${voxblox_INCLUDE_DIRS}
  ${voxblox_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
)

set(executable_name tsdf_server_node)

add_executable(${executable_name}
  src/tsdf_server_node.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  Glog
  rclcpp
  std_msgs
  geometry_msgs
  std_srvs
  voxblox
  tf2_ros
  rclcpp_action
  PCL
  voxblox_msgs
  OpenCV
  visualization_msgs
  Gflags
  pcl_conversions
  tf2_eigen
)

add_library(${library_name} SHARED
  src/tsdf_server.cpp
  src/tsdf_server_node.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})
ament_package()