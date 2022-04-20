cmake_minimum_required(VERSION 3.0.2)
project(drone_toolbox_ext_control_template)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  mav_msgs
  nav_msgs
  roscpp
  std_srvs
  tf2
)

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}
 LIBRARIES drone_toolbox_ext_control_template
 CATKIN_DEPENDS dynamic_reconfigure geometry_msgs mavros_msgs mav_msgs nav_msgs roscpp std_srvs tf2
)

include_directories(include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

add_library(controller src/controller.cpp)

add_executable(controller_node src/controller_node.cpp)
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(controller_node
  controller
  ${catkin_LIBRARIES}
)
