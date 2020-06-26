cmake_minimum_required(VERSION 2.8.3)
project(genie_nano)

#Compiler flags
add_definitions(-std=c++1y)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)
find_package( OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS DEPENDS
    OpenCV
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    image_transport
)

include_directories(
  include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    /opt/genicam_v3_0/library/CPP/include
)

link_directories(${catkin_LIBRARY_DIRS})
link_directories(/opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so)
link_directories(/opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so)
link_directories(/usr/local/lib/libGevApi.so)
link_directories(/usr/local/lib/libCorW32.so)
link_directories(/usr/lib/aarch64-linux-gnu/libX11.so)
link_directories(/usr/lib/aarch64-linux-gnu/libXext.so)
link_directories(/usr/lib/aarch64-linux-gnu/libpthread.so)
link_directories(/usr/lib/aarch64-linux-gnu/libstdc++.so.6)

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

add_library(convertBayer src/convertBayer.c)
target_link_libraries(convertBayer
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6)
add_dependencies(convertBayer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(GevFileUtils src/GevFileUtils.c)
target_link_libraries(GevFileUtils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6)
add_dependencies(GevFileUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(X_Display_utils src/X_Display_utils.c)
target_link_libraries(X_Display_utils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6)
add_dependencies(X_Display_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(FileUtil_tiff src/FileUtil_tiff.c)
target_link_libraries(FileUtil_tiff
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6)
add_dependencies(FileUtil_tiff ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(GevUtils src/GevUtils.c)
target_link_libraries(GevUtils
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES}
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6
                      convertBayer)
add_dependencies(GevUtils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_cam src/test_cam.cpp)
target_link_libraries(test_cam 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
                      /usr/local/lib/libGevApi.so
                      /usr/local/lib/libCorW32.so
                      /usr/lib/aarch64-linux-gnu/libX11.so
                      /usr/lib/aarch64-linux-gnu/libXext.so
                      /usr/lib/aarch64-linux-gnu/libpthread.so
                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6
                      convertBayer 
                      GevFileUtils 
                      X_Display_utils 
                      FileUtil_tiff 
                      GevUtils)
add_dependencies(test_cam genie_nano_generate_messages_cpp)


#add_executable(test src/test.cpp)
#target_link_libraries(test 
#                      ${catkin_LIBRARIES} 
#                      ${OpenCV_LIBRARIES} 
#                      /opt/genicam_v3_0/bin/Linux64_ARM/libGenApi_gcc54_v3_0.so
#                      /opt/genicam_v3_0/bin/Linux64_ARM/libGCBase_gcc54_v3_0.so
#                      /usr/local/lib/libGevApi.so
#                      /usr/local/lib/libCorW32.so
#                      /usr/lib/aarch64-linux-gnu/libX11.so
#                      /usr/lib/aarch64-linux-gnu/libXext.so
#                      /usr/lib/aarch64-linux-gnu/libpthread.so
#                      /usr/lib/aarch64-linux-gnu/libstdc++.so.6
#                      convertBayer 
#                      GevFileUtils 
#                      X_Display_utils 
#                      FileUtil_tiff 
#                      GevUtils)
#add_dependencies(test genie_nano_generate_messages_cpp)
