cmake_minimum_required(VERSION 2.8.3)
project(assignment2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  message_generation
  geometry_msgs
  nav_msgs
)


catkin_package(
CATKIN_DEPENDS message_runtime)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(pointshoot_node src/pointshoot_node.cpp)
add_executable(servoing_node src/servoing_node.cpp)
add_dependencies(pointshoot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(servoing_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pointshoot_node
  ${catkin_LIBRARIES}
 )
target_link_libraries(servoing_node
  ${catkin_LIBRARIES}
 )