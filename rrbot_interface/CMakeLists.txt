cmake_minimum_required(VERSION 3.0.2)
project(rrbot_interface)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur_interface
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrbot_move src/rrbot_move.cpp)
target_link_libraries(rrbot_move ${catkin_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/rrbot_move_python.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)