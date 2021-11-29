cmake_minimum_required(VERSION 3.0.2)
project(riaodo_teleop)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs mavros_msgs geometry_msgs
)



include_directories(include ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/rc_controller.cpp src/set_rate.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(${PROJECT_NAME}_node src/riaodo_teleop_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME})



install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



