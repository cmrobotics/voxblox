cmake_minimum_required(VERSION 3.5)
project(voxblox)

find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Eigen 3 QUIET)
find_package(Gflags REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall --std=c++11 -O3 -fPIC" )
set( CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall -O3 -fPIC" )


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
