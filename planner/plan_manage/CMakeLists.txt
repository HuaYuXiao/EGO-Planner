cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_PREFIX_PATH /opt/ros/noetic;/usr/local;$ENV{HOME}/Prometheus/devel/;$ENV{HOME}/planner_ws/devel;$ENV{HOME}/mapping_ws/devel;$ENV{HOME}/mapping_ws/devel_isolated;$ENV{HOME}/tool_ws/devel)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  plan_env
  path_searching
  bspline_opt
  traj_utils
  cv_bridge
  prometheus_msgs
        std_msgs
        geometry_msgs
        quadrotor_msgs
        message_generation
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(ego_planner_node
  src/ego_planner_node.cpp
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node
  ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
  )
#add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server
        ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
)
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
