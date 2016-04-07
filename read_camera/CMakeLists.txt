cmake_minimum_required(VERSION 2.8.3)
project(read_camera)

find_package(catkin REQUIRED COMPONENTS 
	cv_bridge 
	image_transport 
	roscpp 
	std_msgs)

# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_LIBS}")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES read_cam_uav
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(video_load src/video_load.cpp)
target_link_libraries(video_load  ${catkin_LIBRARIES}   ${OpenCV_LIBS}  )

add_executable(camera_reader src/camera_reader.cpp)
target_link_libraries(camera_reader  ${catkin_LIBRARIES}   ${OpenCV_LIBS}  )

add_executable(image_save src/image_save.cpp)
target_link_libraries(image_save  ${catkin_LIBRARIES}   ${OpenCV_LIBS}  )


