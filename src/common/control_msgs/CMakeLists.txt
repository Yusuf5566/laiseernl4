cmake_minimum_required(VERSION 3.0.2)
project(control_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        planning_msgs
        carla_msgs
        message_generation
)

add_message_files(
        DIRECTORY msgs
        FILES
        vehicle_control.msg
        ackerman_control.msg
        control_debug.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        planning_msgs
        carla_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES msg
        CATKIN_DEPENDS roscpp std_msgs message_runtime
        #  DEPENDS system_lib
)