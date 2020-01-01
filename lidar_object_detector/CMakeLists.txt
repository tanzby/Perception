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

add_executable(lidar_object_detector_node
    src/lidar_object_detector.cpp
    src/lidar_object_detector_node.cpp
    src/cluster_detector.cpp
    src/box_shape_estimation.cpp)

add_executable(lidar_object_tracker_node
    src/lidar_object_tracker_node.cpp
    src/imm_ukf_pda.cpp
    src/ukf.cpp)

add_executable(lidar_object_converter_node
    src/convertor.cpp)

target_link_libraries(lidar_object_detector_node ${catkin_LIBRARIES} )
target_link_libraries(lidar_object_tracker_node  ${catkin_LIBRARIES} )
target_link_libraries(lidar_object_converter_node  ${catkin_LIBRARIES})