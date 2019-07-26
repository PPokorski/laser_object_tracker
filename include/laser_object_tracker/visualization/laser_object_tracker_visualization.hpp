/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2019, Piotr Pokorski
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*
*  1. Redistributions of source code must retain the above copyright notice, this
*     list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright notice,
*     this list of conditions and the following disclaimer in the documentation
*     and/or other materials provided with the distribution.
*
*  3. Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
*  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef LASER_OBJECT_TRACKER_VISUALIZATION_LASER_OBJECT_TRACKER_VISUALIZATION_HPP
#define LASER_OBJECT_TRACKER_VISUALIZATION_LASER_OBJECT_TRACKER_VISUALIZATION_HPP

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <rviz_visual_tools/rviz_visual_tools.h>

#include "laser_object_tracker/data_types/definitions.hpp"
#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"
#include "laser_object_tracker/feature_extraction/features/features.hpp"
#include "laser_object_tracker/tracking/multi_tracking.hpp"

namespace laser_object_tracker {
namespace visualization {
class LaserObjectTrackerVisualization {
 public:
  LaserObjectTrackerVisualization(ros::NodeHandle& pnh, const std::string& base_frame) {
    rviz_visual_tools_.reset(new rviz_visual_tools::RvizVisualTools(base_frame, "visualization/markers"));
    rviz_visual_tools_->setLifetime(0.0);

    pub_point_cloud_ = pnh.advertise<data_types::PointCloudType>("visualization/point_cloud", 1);
    pub_point_clouds_ = pnh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("visualization/point_clouds", 1);
  }

  void publishPointCloud(const data_types::LaserScanFragment& fragment) {
    pub_point_cloud_.publish(fragment.pointCloud());
  }

  void trigger() {
    rviz_visual_tools_->trigger();
  }

  void clearMarkers() {
    rviz_visual_tools_->deleteAllMarkers();
  }

  void publishPointClouds(const std::vector<data_types::LaserScanFragment>& fragments);

  void publishFeatures(const std::vector<data_types::LaserScanFragment>& fragments);

  void publishSegment(const feature_extraction::features::Segment2D& segment, const std_msgs::ColorRGBA& color);

  void publishSegments(const feature_extraction::features::Segments2D& segments);

  void publishMultiSegments(const feature_extraction::features::MultiSegments2D& multi_segments);

  void publishCorner(const feature_extraction::features::Corner2D& corner, const std_msgs::ColorRGBA& color);

  void publishCorners(const feature_extraction::features::Corners2D& corners);

  void publishPoint(const feature_extraction::features::Point2D& point, const std_msgs::ColorRGBA& color);

  void publishTracker(const tracking::BaseTracking& tracker, const std_msgs::ColorRGBA& color);

  void publishMultiTracker(const tracking::MultiTracking& multi_tracker);

  void publishAssignments(const tracking::MultiTracking& multi_tracker,
                          const std::vector<feature_extraction::features::Feature>& measurements,
                          const Eigen::MatrixXd& cost_matrix,
                          const Eigen::VectorXi& assignment_vector);

 private:
  void expandToNColors(int colors);

  rviz_visual_tools::RvizVisualToolsPtr rviz_visual_tools_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_point_clouds_;

  std::vector<float> colours_;
  std::vector<std_msgs::ColorRGBA> rgb_colors_;
};
}  // namespace visualization
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_VISUALIZATION_LASER_OBJECT_TRACKER_VISUALIZATION_HPP
