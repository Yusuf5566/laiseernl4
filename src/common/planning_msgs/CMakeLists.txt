cmake_minimum_required(VERSION 3.0.2)
project(planning_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
)

add_message_files(
        DIRECTORY msgs
        FILES
        PlanningPoint.msg
        way_point.msg
)

add_service_files(
        DIRECTORY srv
        FILES
        Trajectory.srv
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
