cmake_minimum_required(VERSION 3.5.0)
project(point_cloud_fusion)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
    message_filters sensor_msgs tf pcl_ros)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_filters sensor_msgs tf pcl_ros
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    include/point_cloud_fusion/point_cloud_fusion.hpp
    src/point_cloud_fusion.cpp
    )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
    src/point_cloud_fusion_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})