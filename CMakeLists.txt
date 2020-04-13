cmake_minimum_required(VERSION 2.8.3)
project(genie_nano)

#Compiler flags
add_definitions(-std=c++1y)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
)

include_directories(
  include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


## Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN
    "*.h"
  EXCLUDE
)



add_executable(test_cam src/test_cam.cpp)
target_link_libraries(test_cam ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(test_cam genie_nano_generate_messages_cpp)
