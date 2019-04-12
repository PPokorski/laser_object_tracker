# Dynamic object detection and tracking using planar laser scanners
The `laser_object_tracker` provides [ROS](http://www.ros.org/) functionality for detection and tracking of moving
objects using laser scanners. It aims to be an easily expandable framework with some pre-defined tools and methods.
It uses and takes inspiration from both [OpenCV](https://opencv.org/) and [PCL](http://pointclouds.org/)

## Requirements
The package is built upon [ROS](http://www.ros.org/) software with heavy usage of [OpenCV](https://opencv.org/) and
[PCL](http://pointclouds.org/), so all of these packages are required. Did I mention it's Linux only?
So, to minimal requirements:
* C++14 (maybe C++11 if need be)
* ROS Kinetic
* PCL (developed with [ROS-bundled](http://wiki.ros.org/pcl) version)
* OpenCV (developed with [ROS-bundled](http://wiki.ros.org/opencv3) version)

## Idea
The main idea of the framework is to develop a pipeline consisting of several steps:
* Segmentation - step of dividing sensory data into seperate pieces, each one corresponding to a single object
* Feature extraction - extracting features. What more to say?
* Data association - process of assigning detected objects to existing tracks
* Tracker update - updating tracker with detected objects
