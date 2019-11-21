cmake_minimum_required(VERSION 2.8.3)
project(underwater_color_enhance)


add_compile_options(-std=c++11)


add_subdirectory(third-party/yaml-cpp)
add_subdirectory(third-party/ticpp)


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 2.4.3 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   endif()
endif()

find_package(Boost COMPONENTS system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  cv_bridge
  message_filters
  image_transport
  roslint
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 rospy
                 sensor_msgs
                 mavros_msgs
                 cv_bridge
                 opencv2
                 message_filters
                 image_transport
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/yaml-cpp/include
  ${CMAKE_SOURCE_DIR}/ticpp
  /home/darobot/orb_slam/ORB_SLAM2/msg_gen/cpp/include
  /home/darobot/Research/escalibr/catkin_ws/src/escalibr/include
)


add_executable(myProgram
  src/Options/ros_correct.cpp
  src/ColorCorrect.cpp
  src/ImageHandler.cpp
  src/NewModel.cpp
  src/Scene.cpp
)

add_executable(secondProgram
  src/Options/image_correct.cpp
  src/ColorCorrect.cpp
  src/NewModel.cpp
  src/Scene.cpp
)

add_library(${PROJECT_NAME}
  src/ColorCorrect.cpp
  src/ImageHandler.cpp
  src/NewModel.cpp
  src/Scene.cpp
)

target_link_libraries(myProgram
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  yaml-cpp
)

target_link_libraries(secondProgram
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  yaml-cpp
)

roslint_cpp(
  src/Options/image_correct.cpp
  src/ColorCorrect.cpp
  include/${PROJECT_NAME}/ColorCorrect.h
  src/ImageHandler.cpp
  include/${PROJECT_NAME}/ImageHandler.h
  include/${PROJECT_NAME}/Method.h
  src/Scene.cpp
  include/${PROJECT_NAME}/Scene.h
  src/NewModel.cpp
  include/${PROJECT_NAME}/NewModel.h
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
