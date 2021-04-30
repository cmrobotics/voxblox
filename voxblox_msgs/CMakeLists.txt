cmake_minimum_required(VERSION 3.5)
project(voxblox_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC)
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Block.msg"
  "msg/Layer.msg"
  "msg/Mesh.msg"
  "msg/MeshBlock.msg"
  "msg/VoxelEvaluationDetails.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()