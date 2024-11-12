cmake_minimum_required(VERSION 3.0.2)
project(control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        carla_msgs
        control_msgs
)

catkin_package(
        #  INCLUDE_DIRS include
        #        LIBRARIES carla_bridge
        #  CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
# 统计当前文件下所有参与编译的文件
file(GLOB_RECURSE SRC_LIST
        CONFIGURE_DEPENDS
        src/*.cpp
        src/*.c
        src/*.cxx
        src/*.h
        src/*.hpp)


# ros glog
find_package(Glog REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE
        ${catkin_LIBRARIES} glog::glog
)
# 将文件加入到工程依赖
target_sources(${PROJECT_NAME} PRIVATE ${SRC_LIST})

#Python
#find_package(Python3 COMPONENTS Development NumPy)
#target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)
SET(Python3_INCLUDE_DIRS "/usr/include/python3.8")
SET(Python3_LIBRARIES "/lib/python3.8/config-3.8-x86_64-linux-gnu/libpython3.8.so")
SET(PYTHON_EXECUTABLE /usr/bin/python3.8)
include_directories(./include/ ${Python3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${Python3_LIBRARIES})
