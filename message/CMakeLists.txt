cmake_minimum_required(VERSION 2.8.8)

# Project name
project(message)

# Include our cmake files
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/)

# Find catkin (the ROS build system)
find_package(catkin QUIET COMPONENTS roscpp rosbag std_msgs geometry_msgs sensor_msgs message_generation)

add_message_files(
   FILES
   UVmsg.msg
   UVsmsg.msg
   UVListstamped.msg
   Point.msg
   featsgt.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs 
   sensor_msgs
)


# Describe catkin project
if (catkin_FOUND)
    add_definitions(-DROS_AVAILABLE=1)
    catkin_package(
            CATKIN_DEPENDS roscpp rosbag std_msgs geometry_msgs sensor_msgs message_runtime
    )
else()
    message(WARNING "CATKIN NOT FOUND BUILDING WITHOUT ROS!")
endif()