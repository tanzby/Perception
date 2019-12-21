cmake_minimum_required(VERSION 3.5.0)
project(lidar_object_detector)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "")

find_package(catkin REQUIRED COMPONENTS
    sensor_msgs pcl_ros perception_msgs jsk_recognition_msgs)

find_package(PCL REQUIRED COMPONENTS segmentation )

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS sensor_msgs pcl_ros perception_msgs jsk_recognition_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/cluster_detector.cpp
    src/box_shape_estimation.cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

add_executable(${PROJECT_NAME}_node
    src/lidar_object_detector.cpp
    src/lidar_object_detector_node.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PROJECT_NAME})