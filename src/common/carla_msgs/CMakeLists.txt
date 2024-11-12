cmake_minimum_required(VERSION 3.0.2)
project(carla_msgs)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        message_generation
)

add_message_files(
        DIRECTORY msgs
        FILES
        carla_waypoint.msg
        carla_route.msg
        carla_vehicle_state.msg
		carla_traffic_light.msg

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
