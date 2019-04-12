#############
## Project ##
#############

cmake_minimum_required(VERSION 3.5)
project(laser_object_tracker)

## Build flags ##
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


## Dependancies ##
find_package(OpenCV REQUIRED COMPONENTS
        tracking)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        laser_geometry
        pcl_conversions
        pcl_ros
        rviz_visual_tools
        sensor_msgs
        visualization_msgs)


## catkin specific configuration ##
catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES vb_range_to_laserscan
        CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
)

###########
## Build ##
###########

## Includes ##

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        include/)

## Libraries ##

add_library(${PROJECT_NAME}_data_types
        src/data_types/laser_scan_fragment.cpp)

target_link_libraries(${PROJECT_NAME}_data_types
        ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_segmentation
        src/segmentation/breakpoint_detection.cpp
        src/segmentation/adaptive_breakpoint_detection.cpp)

target_link_libraries(${PROJECT_NAME}_segmentation
        ${PROJECT_NAME}_data_types)



## Executables ##
add_executable(${PROJECT_NAME}_node
        src/laser_object_tracker.cpp
        src/visualization/laser_object_tracker_visualization.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_data_types
        ${PROJECT_NAME}_segmentation)


## Tests ##
include_directories(
        test/include)

catkin_add_gtest(${PROJECT_NAME}_test
        test/src/test_motion_tracking.cpp
        test/src/data_types/definitions_test.cpp
        test/src/data_types/laser_scan_fragment_test.cpp
        test/src/segmentation/adaptive_breakpoint_detection_test.cpp
        test/src/segmentation/breakpoint_detection_test.cpp
        test/src/segmentation/distance_calculation_test.cpp)

target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}_data_types
        ${PROJECT_NAME}_segmentation)