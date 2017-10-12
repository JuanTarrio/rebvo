# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(rebvo)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp nodelet std_msgs geometry_msgs gencpp tf tf_conversions)

add_message_files(
   FILES
   Keyline.msg
   EdgeMap.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rebvo_nodelet
  CATKIN_DEPENDS nodelet roscpp std_msgs geometry_msgs tf tf_conversions pcl pcl_ros
)

include_directories(
	include
        ../../../include
	include/rebvo
	${catkin_INCLUDE_DIRS}
)

link_directories(
    ../../../rebvolib/
    )

add_definitions(-std=c++11)
add_definitions(-O2)

FILE(GLOB incFiles "include/rebvo/*.h")

add_library(rebvo_nodelet src/rebvo_nodelet.cpp ${incFiles})
target_link_libraries(rebvo_nodelet rebvolib  X11 v4l2 m gd lapack ${catkin_LIBRARIES})
add_dependencies(rebvo_nodelet rebvo_generate_messages_cpp)

add_executable(rebvo src/rebvo_node.cpp ${incFiles})
target_link_libraries(rebvo rebvolib ${catkin_LIBRARIES})

## rebvoLib



# %EndTag(FULLTEXT)%
