cmake_minimum_required(VERSION 2.8.3)
project(camera_command)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
)
find_package( OpenCV 3.0 REQUIRED )
add_definitions(-g)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS cv_bridge image_transport roscpp
)

include_directories(include)
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(camera_command_node src/camera_command_node.cpp src/camera_command.cpp)
target_link_libraries(camera_command_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(test_camera src/test_camera.cpp)
target_link_libraries(test_camera
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
