cmake_minimum_required(VERSION 3.5.0)
project(perception_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation message_runtime geometry_msgs jsk_recognition_msgs)

add_message_files(
    FILES
    Object.msg ObjectArray.msg)

generate_messages(
    DEPENDENCIES
    std_msgs geometry_msgs jsk_recognition_msgs
)

catkin_package(
    CATKIN_DEPENDS message_generation message_runtime geometry_msgs jsk_recognition_msgs)