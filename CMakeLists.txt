#############
## Project ##
#############

cmake_minimum_required(VERSION 3.5)
project(laser_object_tracker)

## Build flags ##
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_CPPLINT /usr/local/bin/cpplint;--linelength=120 --root=include)

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

add_library(${PROJECT_NAME}_filtering
        src/filtering/aggregate_segmented_filtering.cpp
        src/filtering/base_segmented_filtering.cpp
        src/filtering/obb_area_filter.cpp
        src/filtering/occlusion_detection.cpp
        src/filtering/points_number_filter.cpp)

target_link_libraries(${PROJECT_NAME}_filtering
        ${PROJECT_NAME}_data_types)

add_library(${PROJECT_NAME}_feature_extraction
        src/feature_extraction/random_sample_consensus_corner_detection.cpp
        src/feature_extraction/random_sample_consensus_segment_detection.cpp
        src/feature_extraction/search_based_corner_detection.cpp)

target_link_libraries(${PROJECT_NAME}_feature_extraction
        ${PROJECT_NAME}_data_types)

add_library(${PROJECT_NAME}_tracking
        src/tracking/base_tracking.cpp
        src/tracking/iteration_tracker_rejection.cpp
        src/tracking/kalman_filter.cpp
        src/tracking/multi_tracker.cpp)

target_link_libraries(${PROJECT_NAME}_tracking
        ${OpenCV_LIBS}
        ${PROJECT_NAME}_data_association)

add_library(${PROJECT_NAME}_data_association
        src/data_association/base_data_association.cpp
        src/data_association/hungarian_algorithm.cpp
        src/data_association/naive_linear_assignment.cpp)



## Executables ##
add_executable(${PROJECT_NAME}_node
        src/laser_object_tracker.cpp
        src/visualization/laser_object_tracker_visualization.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}_data_association
        ${PROJECT_NAME}_data_types
        ${PROJECT_NAME}_feature_extraction
        ${PROJECT_NAME}_filtering
        ${PROJECT_NAME}_segmentation
        ${PROJECT_NAME}_tracking)

#add_executable(${PROJECT_NAME}_multi_tracker
#        src/multi_tracker_test.cpp)
#
#target_link_libraries(${PROJECT_NAME}_multi_tracker
#        ${catkin_LIBRARIES}
#        ${PROJECT_NAME}_data_association
#        ${PROJECT_NAME}_tracking)


## Tests ##
include_directories(
        test/include)

catkin_add_gtest(${PROJECT_NAME}_test
        test/src/data_association/hungarian_algorithm_test.cpp
        test/src/data_association/naive_linear_assignment_test.cpp
        test/src/data_types/definitions_test.cpp
        test/src/data_types/laser_scan_fragment_test.cpp
        test/src/feautre_extraction/random_sample_consensus_segment_detection_test.cpp
        test/src/feautre_extraction/sample_consensus_model_cross2d_test.cpp
        test/src/feautre_extraction/search_based_corner_detection_test.cpp
        test/src/filtering/aggregate_segmented_filtering_test.cpp
        test/src/filtering/base_segmented_filtering_test.cpp
        test/src/filtering/obb_area_filter_test.cpp
        test/src/filtering/points_number_filter_test.cpp
        test/src/segmentation/adaptive_breakpoint_detection_test.cpp
        test/src/segmentation/breakpoint_detection_test.cpp
        test/src/segmentation/distance_calculation_test.cpp
        test/src/tracking/iteration_tracker_rejection_test.cpp
        test/src/tracking/kalman_filter_test.cpp
        test/src/tracking/multi_tracker_test.cpp)

target_link_libraries(${PROJECT_NAME}_test
        gmock_main
        gtest_main
        ${PROJECT_NAME}_data_types
        ${PROJECT_NAME}_segmentation
        ${PROJECT_NAME}_feature_extraction
        ${PROJECT_NAME}_filtering
        ${PROJECT_NAME}_data_association
        ${PROJECT_NAME}_tracking)