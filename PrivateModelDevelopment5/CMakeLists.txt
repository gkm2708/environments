# Basic CMake Check and global settings

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gzmaze)
add_compile_options(-r -O -g -W -Wall)



# Build for MazePlugin

#find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(roscpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
#find_package(chrono REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${image_transport_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${CHRONO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
link_directories(${image_transport_LIBRARY_DIRS})

link_directories(${roscpp_INCLUDE_DIRS})
link_directories(${geometry_msgs_INCLUDE_DIRS})
link_directories(${std_msgs_INCLUDE_DIRS})


list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS} -std=c++11")

#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

#add_library(lmaze_ros_listener SHARED lmaze_plugin/LmazeRosListener.cc)
#target_link_libraries(lmaze_ros_listener ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES} )

add_library(lmaze_controller_plugin SHARED lmaze_plugin/LmazeControllerPlugin.cc)
target_link_libraries(lmaze_controller_plugin ${GAZEBO_LIBRARIES} 
#lmaze_ros_listener
${roscpp_LIBRARIES} )

add_library(lmaze_plugin SHARED lmaze_plugin/LmazePlugin.cc)
target_link_libraries(lmaze_plugin ${GAZEBO_LIBRARIES} 
#lmaze_ros_listener 
${roscpp_LIBRARIES})


add_library(contact SHARED ball_plugin/ContactPlugin.cc)
target_link_libraries(contact ${GAZEBO_LIBRARIES} ${chrono_LIBRARIES})

add_library(ball_plugin SHARED ball_plugin/BallPlugin.cc)
target_link_libraries(ball_plugin ${GAZEBO_LIBRARIES})

add_library(ball_controller_plugin SHARED ball_plugin/BallControllerPlugin.cc)
target_link_libraries(ball_controller_plugin ${GAZEBO_LIBRARIES})

add_library(camera_capture SHARED camera_plugin/CameraCapture.cc)
target_link_libraries(camera_capture ${GAZEBO_LIBRARIES} CameraPlugin ${roscpp_LIBRARIES} ${image_transport_LIBRARIES} ${OpenCV_LIBRARIES} ${PROTOBUF_LIBRARIES})

add_library(camera_follow SHARED camera_plugin/CameraFollow.cc)
target_link_libraries(camera_follow ${GAZEBO_LIBRARIES})

# Build the stand-alone test program
#add_executable(dummy_data dummy_Data/dummy_data.cc)

#if (${gazebo_VERSION_MAJOR} LESS 6)
  # These two
  #include(FindBoost)
  #find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
  #target_link_libraries(dummy_data ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
#else()
  #target_link_libraries(dummy_data ${GAZEBO_LIBRARIES})
#endif()

