cmake_minimum_required(VERSION 2.8.3)
project(camera)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	rospy
	std_msgs
	message_generation
	)
add_service_files(
	FILES
	FindObjects.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
)

find_package(catkin REQUIRED)

find_package( OpenCV REQUIRED )
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
add_executable(camera_controller_node src/camera_controller.cpp)
target_link_libraries(camera_controller_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)




## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
 camera.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

