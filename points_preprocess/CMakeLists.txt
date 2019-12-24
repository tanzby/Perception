cmake_minimum_required(VERSION 3.5.0)
project(points_preprocess)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "")

find_package(catkin REQUIRED COMPONENTS
    message_filters sensor_msgs tf pcl_ros hdmap_msgs)

find_package(PCL REQUIRED COMPONENTS filters)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS message_filters sensor_msgs tf pcl_ros hdmap_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_map_roi_filter SHARED
    src/bitmap_2d.cpp
    src/map_roi_filter.cpp)
target_link_libraries(${PROJECT_NAME}_map_roi_filter glog)

add_library(${PROJECT_NAME}_rules_filter SHARED
    src/rules_based_filter.cpp)

add_library(${PROJECT_NAME}_ground_removal SHARED
    src/ground_removal.cpp)

add_executable(${PROJECT_NAME}_node
    src/points_preprocess_node.cpp
    src/points_preprocess.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES}
    ${PROJECT_NAME}_map_roi_filter
    ${PROJECT_NAME}_rules_filter
    ${PROJECT_NAME}_ground_removal)