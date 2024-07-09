cmake_minimum_required(VERSION 3.0.2)
project(drone_toolbox_ext_control_template)

add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  mav_msgs
  nav_msgs
  roscpp
  std_srvs
  tf2
  aruco_msgs
  message_generation
  tf2_ros


)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}
 LIBRARIES drone_toolbox_ext_control_template
 CATKIN_DEPENDS
 message_runtime
 geometry_msgs
 std_msgs

 # no CATKIN_DEPENDS, because this package does not contain header files that will be used by other packages
)

include_directories(include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)




# !---------------] find libs what wavehands need
set(CMAKE_PREFIX_PATH "/root/dev/catkin_ws/src/libtorch-cxx11-abi-shared-with-deps-2.1.2+cu121/libtorch")
set(Torch_DIR $ENV{/root/dev/catkin_ws/src/libtorch-cxx11-abi-shared-with-deps-2.1.2+cu121/libtorch/share/cmake/Torch})
message(STATUS "++++++ torch find path" ${CMAKE_PREFIX_PATH})
find_package(Torch REQUIRED)
include_directories(include/)
include_directories(${TORCH_INCLUDE_DIRS})

if (NOT Torch_FOUND)
    message(FATAL_ERROR "you should change CMAKE_PREFIX_PATH to libtorch dir")
else ()
    message(STATUS "found libtorch at " ${TORCH_LIBRARIES})
endif ()

add_executable(controller_node src/controller_node.cpp)

add_executable(marker_pose_estimator_node src/marker_pose_estimator_node.cpp)
add_dependencies(marker_pose_estimator_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_library(mantis_controller src/mantis_controller.cpp)

target_link_libraries(controller_node
  mantis_controller
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
)
target_link_libraries(marker_pose_estimator_node
  
  ${TORCH_LIBRARIES}
  ${catkin_LIBRARIES}
)

## For debugging
set (CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}  -g ")
set (CMAKE_VERBOSE_MAKEFILE ON)



