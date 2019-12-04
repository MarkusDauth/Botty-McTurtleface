cmake_minimum_required(VERSION 2.8.3)
project(camera)
catkin_package(
)

find_package(catkin REQUIRED COMPONENTS roscpp )
find_package(catkin REQUIRED)

find_package( OpenCV REQUIRED )
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
add_executable(action_controller_node src/action_controller.cpp)
target_link_libraries(action_controller_node
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)



## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
 camera.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

