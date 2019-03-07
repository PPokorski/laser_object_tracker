#############
## Project ##
#############

cmake_minimum_required(VERSION 3.12)
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
        src/data_types/laser_scan_fragment.cpp test/include/test/data_types/test_data.hpp)

target_link_libraries(${PROJECT_NAME}_data_types
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS})

## Executables ##
add_executable(${PROJECT_NAME}_node
        src/laser_object_tracker.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_data_types)


## Tests ##
include_directories(
        test/include)

catkin_add_gtest(${PROJECT_NAME}_test
        test/src/test_motion_tracking.cpp
        test/src/data_types/definitions_test.cpp
        test/src/data_types/laser_scan_fragment_test.cpp)

target_link_libraries(${PROJECT_NAME}_test
        ${PROJECT_NAME}_data_types)