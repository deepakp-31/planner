cmake_minimum_required(VERSION 3.0.2)
project(rrt_planner)

# # Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(

  # INCLUDE_DIRS include
  # LIBRARIES rrt_planner
  # CATKIN_DEPENDS roscpp std_msgs
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrt src/rrt_node.cpp src/planner.cpp src/rrt_star_planner.cpp src/visualisation.cpp)
target_link_libraries(rrt ${catkin_LIBRARIES})
