cmake_minimum_required(VERSION 3.5)
project(voxblox_ros)

################
# DEPENDENCIES #
################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(dependencies
    rclcpp
    rclcpp_action
)

#############
# LIBRARIES #
#############

include_directories(include)

add_library(esdf_server SHARED src/esdf_server.cc)
list(APPEND voxblox_libs esdf_server)
add_library(interactive_slider SHARED src/interactive_slider.cc)
list(APPEND voxblox_libs interactive_slider)
add_library(simulation_server SHARED src/simulation_server.cc)
list(APPEND voxblox_libs simulation_server)
add_library(intensity_server SHARED src/intensity_server.cc)
list(APPEND voxblox_libs intensity_server)
add_library(transformer SHARED src/transformer.cc)
list(APPEND voxblox_libs transformer)
add_library(tsdf_server SHARED src/tsdf_server.cc)
list(APPEND voxblox_libs tsdf_server)

foreach(voxblox_lib ${voxblox_libs})
  ament_target_dependencies(${voxblox_lib} ${dependencies})
  target_compile_definitions(${voxblox_lib} PRIVATE VOXBLOX_EXPORT)
endforeach()

###############
# EXECUTABLES #
###############

add_executable(voxblox_eval src/voxblox_eval.cpp)
ament_target_dependencies(voxblox_eval ${dependencies})

add_executable(tsdf_server src/tsdf_server.cpp)
ament_target_dependencies(tsdf_server ${dependencies})

add_executable(esdf_server src/esdf_server.cpp)
ament_target_dependencies(esdf_server ${dependencies})

add_executable(intensity_server src/intensity_server.cpp)
ament_target_dependencies(intensity_server ${dependencies})

add_executable(simulation_eval src/simulation_eval.cpp)
ament_target_dependencies(simulation_eval ${dependencies})

add_executable(visualize_tsdf src/visualize_tsdf.cpp)
ament_target_dependencies(visualize_tsdf ${dependencies})

install(
  DIRECTORY launch cfg mesh_results
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  voxblox_eval
  tsdf_server
  esdf_server
  intensity_server
  simulation_eval
  visualize_tsdf
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()