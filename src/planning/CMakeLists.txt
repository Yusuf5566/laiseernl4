cmake_minimum_required(VERSION 3.0.2)
project(planning)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        routing_msgs
        planning_msgs
        tf
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES hello
        #  CATKIN_DEPENDS roscpp std_msgs
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

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
)

# 将文件加入到工程依赖
target_sources(${PROJECT_NAME} PRIVATE ${SRC_LIST})

#Python
find_package(Python3 COMPONENTS Development NumPy)
target_link_libraries(${PROJECT_NAME} Python3::Python Python3::NumPy)

#Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# osqpg
find_package(OsqpEigen REQUIRED)
target_link_libraries(${PROJECT_NAME}  OsqpEigen::OsqpEigen)

# glog
find_package(Glog REQUIRED)
target_link_libraries(${PROJECT_NAME} glog::glog)

# Threads
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} Threads::Threads)