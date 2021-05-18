cmake_minimum_required(VERSION 3.5)
project(voxblox)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++17 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )

set(dependencies
  Protobuf
  Eigen3
  Gflags
  Glog
)

############
# PROTOBUF #
############

INCLUDE(FindProtobuf)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
SET_SOURCE_FILES_PROPERTIES(${PROTO_SRC} ${PROTO_INCL} PROPERTIES GENERATED TRUE)
set(PROTOBUF_COMPILE_PYTHON true)
PROTOBUF_GENERATE_CPP(PROTO_SRC PROTO_HEADER proto/voxblox/Layer.proto proto/voxblox/Block.proto)
ADD_LIBRARY(proto ${PROTO_HEADER} ${PROTO_SRC})
##
## TODO: move proto headers to source
##


####################
# SET SOURCE FILES #
####################

set("${PROJECT_NAME}_SRCS"
  src/alignment/icp.cc
  src/core/block.cc
  src/core/esdf_map.cc
  src/core/tsdf_map.cc
  src/integrator/esdf_integrator.cc
  src/integrator/esdf_occ_integrator.cc
  src/integrator/integrator_utils.cc
  src/integrator/intensity_integrator.cc
  src/integrator/tsdf_integrator.cc
  src/io/mesh_ply.cc
  src/io/sdf_ply.cc
  src/mesh/marching_cubes.cc
  src/simulation/objects.cc
  src/simulation/simulation_world.cc
  src/utils/camera_model.cc
  src/utils/evaluation_utils.cc
  src/utils/layer_utils.cc
  src/utils/neighbor_tools.cc
  src/utils/protobuf_utils.cc
  src/utils/timing.cc
  src/utils/voxel_utils.cc
)


#########
# TESTS #
#########
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

#add_custom_target(test_data)
#add_custom_command(TARGET test_data
#                  COMMAND rm -rf test_data
#                  COMMAND mkdir -p test_data
#                  COMMAND cp -r ${CMAKE_SOURCE_DIR}/test/test_data/*
#                                test_data/ || :)
#
##add_definitions(-DVISUALIZE_UNIT_TEST_RESULTS)
#
#catkin_add_gtest(test_approx_hash_array
#  test/test_approx_hash_array.cc
#)
#target_link_libraries(test_approx_hash_array ${PROJECT_NAME})
#
#catkin_add_gtest(test_tsdf_map
#  test/test_tsdf_map.cc
#)
#target_link_libraries(test_tsdf_map ${PROJECT_NAME})
#
#catkin_add_gtest(test_protobuf
#  test/test_protobuf.cc
#)
#target_link_libraries(test_protobuf ${PROJECT_NAME})
#
#catkin_add_gtest(test_tsdf_interpolator
#  test/test_tsdf_interpolator.cc
#)
#target_link_libraries(test_tsdf_interpolator ${PROJECT_NAME})
#
#catkin_add_gtest(test_layer
#  test/test_layer.cc
#)
#target_link_libraries(test_layer ${PROJECT_NAME})
#
#catkin_add_gtest(test_merge_integration
#  test/test_merge_integration.cc
#)
#target_link_libraries(test_merge_integration ${PROJECT_NAME})
#
#catkin_add_gtest(test_layer_utils
#  test/test_layer_utils.cc
#)
#target_link_libraries(test_layer_utils ${PROJECT_NAME})
#
#catkin_add_gtest(test_sdf_integrators
#  test/test_sdf_integrators.cc
#)
#target_link_libraries(test_sdf_integrators ${PROJECT_NAME})
#
#catkin_add_gtest(test_bucket_queue
#  test/test_bucket_queue.cc
#)
#target_link_libraries(test_bucket_queue ${PROJECT_NAME})
#
#catkin_add_gtest(test_clear_spheres
#  test/test_clear_spheres.cc
#)
#target_link_libraries(test_clear_spheres ${PROJECT_NAME})



############
# BINARIES #
############


include_directories(include)
add_library (voxblox ${${PROJECT_NAME}_SRCS})
target_include_directories(voxblox
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_export_targets(export_voxblox HAS_LIBRARY_TARGET)


install(
  TARGETS voxblox
  EXPORT export_voxblox
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(include)
ament_export_libraries(voxblox)
ament_export_dependencies(${dependencies})
ament_package()
