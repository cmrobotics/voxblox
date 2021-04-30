
cmake_minimum_required(VERSION 3.5)

project(voxblox_rviz_plugin)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


## This setting causes Qt's "MOC" generation to happen automatically.
set(CMAKE_AUTOMOC ON)

FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui QUIET)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)

## Avoid Qt signals and slots defining "emit", "slots", etc.
add_definitions(-DQT_NO_KEYWORDS)

set(HEADER_FILES include/voxblox_rviz_plugin/voxblox_mesh_display.h include/voxblox_rviz_plugin/voxblox_mesh_visual.h)

set(SRC_FILES src/voxblox_mesh_display.cc src/voxblox_mesh_visual.cc )

#cs_add_library(${PROJECT_NAME}
#  ${SRC_FILES}
#  ${HEADER_FILES}
#  ${MOC_FILES}
#)

#target_link_libraries(${PROJECT_NAME}
#  ${QT_LIBRARIES}
#)


##########
# EXPORT #
##########

#install(
#  FILES plugin_description.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#install(
#  DIRECTORY icons/
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)


include_directories(
  include
  ${voxblox_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${RVIZ_INCLUDE_DIRS}
  ${RVIZ_COMMON_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

# If rviz is found, add plugins
IF(rviz_default_plugins_FOUND)

    ## We also use Ogre for rviz plugins
    #include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
    #pkg_check_modules(OGRE OGRE)
    #include_directories( ${OGRE_INCLUDE_DIRS} )
    #link_directories( ${OGRE_LIBRARY_DIRS} )

    MESSAGE(STATUS "WITH rviz")

    ## RVIZ plugin
    IF(QT4_FOUND)
	    qt4_wrap_cpp(MOC_FILES
        include/voxblox_rviz_plugin/voxblox_mesh_display.h
        include/voxblox_rviz_plugin/voxblox_mesh_visual.h	      
	    )
    ELSE()
	   qt5_wrap_cpp(MOC_FILES
      include/voxblox_rviz_plugin/voxblox_mesh_display.h
      include/voxblox_rviz_plugin/voxblox_mesh_visual.h	      
    )
    ENDIF()
    
    # tf:message_filters, mixing boost and Qt signals
    set_property(
       SOURCE src/voxblox_mesh_display.cc src/voxblox_mesh_visual.cc
       PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
    )
    add_library(voxblox_rviz_plugin SHARED
       src/voxblox_mesh_display.cc
       src/voxblox_mesh_visual.cc
       ${MOC_FILES}
    )
    target_include_directories(voxblox_rviz_plugin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    
    ament_target_dependencies(voxblox_rviz_plugin
      ${Libraries} 
      rviz_common 
      rviz_rendering 
      rviz_default_plugins
    )
    #ament_export_interfaces(voxblox_rviz_plugin HAS_LIBRARY_TARGET)
    target_link_libraries(voxblox_rviz_plugin rtabmap_ros)
    IF(Qt5_FOUND)
        QT5_USE_MODULES(voxblox_rviz_plugin Widgets Core Gui)
    ENDIF(Qt5_FOUND)

    foreach(typesupport_impl ${typesupport_impls})
	  rosidl_target_interfaces(voxblox_rviz_plugin
	    ${PROJECT_NAME}_msgs ${typesupport_impl}
	  )
    endforeach()
    
    # Causes the visibility macros to use dllexport rather than dllimport,
    # which is appropriate when building the dll but not consuming it.
    target_compile_definitions(voxblox_rviz_plugin PRIVATE "VOXBLOX_RVIZ_PLUGIN_BUILDING_LIBRARY") #TODO: this is unlikely to work

    # prevent pluginlib from using boost
    target_compile_definitions(voxblox_rviz_plugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
    
    pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ENDIF(rviz_default_plugins_FOUND)

ament_export_dependencies()

ament_package()