cmake_minimum_required(VERSION 2.8.3)
project(underwater_color_enhance)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

add_compile_options(-std=c++11)


add_subdirectory(third-party/yaml-cpp)
add_subdirectory(third-party/ticpp)
add_subdirectory(third-party/dlib)


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
  ORB_SLAM2
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
                 ORB_SLAM2
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  third-party/dlib
  third-party/ticpp
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
  dlib::dlib
  ticpp
)

target_link_libraries(secondProgram
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  yaml-cpp
  dlib::dlib
  ticpp
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
