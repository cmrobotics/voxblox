cmake_minimum_required(VERSION 3.5)
project(voxblox)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Gflags REQUIRED)
find_package(Glog REQUIRED)

set(CMAKE_CXX_STANDARD 11)
#set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
#set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )

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


############
# BINARIES #
############
add_definitions(${EIGEN3_DEFINITIONS})
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
include_directories(include
  ${EIGEN3_INCLUDE_DIR}
)


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

ament_target_dependencies(voxblox ${dependencies})

ament_export_include_directories(include)
ament_export_libraries(voxblox)

#########
# TESTS #
#########
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()


ament_export_dependencies(${dependencies})
ament_package()
