cmake_minimum_required(VERSION 3.15)
project(routing_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
)

add_message_files(
        DIRECTORY msgs
        FILES
        Point.msg
        routing_point.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES msg
          CATKIN_DEPENDS roscpp std_msgs message_runtime
        #  DEPENDS system_lib
)

