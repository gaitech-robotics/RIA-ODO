cmake_minimum_required(VERSION 3.0.2)
project(riaodo_wheel_support_motion)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(CATKIN_DEPENDS  roscpp sensor_msgs std_msgs tf2 tf2_ros geometry_msgs)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
 
add_definitions(${EIGEN3_DEFINITIONS})

add_executable(${PROJECT_NAME} src/riaodo_wheel_support_motion.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



