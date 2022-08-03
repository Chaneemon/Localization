cmake_minimum_required(VERSION 3.0.2)
project(kkanbu_sensor)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  rviz
  image_transport
  darknet_ros_msgs
  ublox_gps
  ublox_msgs
  nodelet
  kkanbu_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)


add_message_files(
  FILES
#  C_Obstacle.msg
#  C_Obstacles.msg
#  current_state.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kkanbu_sensor opencv
  CATKIN_DEPENDS message_generation roscpp std_msgs sensor_msgs cv_bridge image_transport darknet_ros_msgs ublox_msgs ublox_gps
  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(kkanbu_camera src/kkanbu_camera.cpp)
add_dependencies(kkanbu_camera 
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${caktin_EXPORTED_TARGETS}
	kkanbu_msgs_generate_messages_cpp
)
target_link_libraries(kkanbu_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(kkanbu_pose src/kkanbu_pose.cpp src/mkgmtime.c)
add_dependencies(kkanbu_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${caktin_EXPORTED_TARGETS})
target_link_libraries(kkanbu_pose
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_executable(kkanbu_merge src/kkanbu_merge.cpp)
add_dependencies(kkanbu_merge 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${caktin_EXPORTED_TARGETS}
	kkanbu_msgs_generate_messages_cpp
)
target_link_libraries(kkanbu_merge
    ${catkin_LIBRARIES}
)

# add_executable(kkanbu_filter src/kkanbu_filter.cpp)
# add_dependencies(kkanbu_filter 
# 	${${PROJECT_NAME}_EXPORTED_TARGETS} 
# 	${caktin_EXPORTED_TARGETS}
# 	kkanbu_msgs_generate_messages_cpp
# )
# target_link_libraries(kkanbu_filter
#     ${catkin_LIBRARIES}
# )



