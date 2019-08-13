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

#ifndef LASER_OBJECT_TRACKER_MULTI_TRACKER_ROS_HPP
#define LASER_OBJECT_TRACKER_MULTI_TRACKER_ROS_HPP

#include "laser_object_tracker/data_association/data_association.hpp"
#include "laser_object_tracker/data_types/data_types.hpp"
#include "laser_object_tracker/feature_extraction/feature_extraction.hpp"
#include "laser_object_tracker/filtering/filtering.hpp"
#include "laser_object_tracker/segmentation/segmentation.hpp"
#include "laser_object_tracker/tracking/tracking.hpp"
#include "laser_object_tracker/visualization/visualization.hpp"

namespace laser_object_tracker {
template<class T>
void getParam(ros::NodeHandle& node_handle, const std::string& key, T& param) {
  if (!node_handle.getParam(key, param)) {
    throw std::logic_error("Param " + key + " not found!");
  }
}

class MultiTrackerROS {
 public:
  using Feature = feature_extraction::features::Object;

  MultiTrackerROS(const ros::NodeHandle& node_handle);

  void update();

 private:
  void laserScanCallback(const sensor_msgs::LaserScan::Ptr& laser_scan);

  static std::shared_ptr<segmentation::BaseSegmentation>getSegmentation(
      ros::NodeHandle& node_handle);
  static std::shared_ptr<filtering::BaseSegmentedFiltering> getSegmentedFiltering(
      ros::NodeHandle& node_handle);
  static std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> getFeatureExtraction(
      ros::NodeHandle& node_handle);
  static std::shared_ptr<tracking::BaseMultiTracking<Feature>> getMultiTracking(
      ros::NodeHandle& node_handle);
  static std::shared_ptr<visualization::LaserObjectTrackerVisualization> getVisualization(
      ros::NodeHandle& node_handle);

  ros::NodeHandle node_handle_;
  ros::Subscriber sub_laser_scan;

  std::string base_frame_;
  ros::Duration transform_wait_timeout_;

  bool is_scan_updated_;

  data_types::LaserScanFragment::LaserScanFragmentFactory scan_fragment_factory_;
  data_types::LaserScanFragment last_scan_fragment_;

  std::shared_ptr<segmentation::BaseSegmentation> segmentation_;
  std::shared_ptr<filtering::BaseSegmentedFiltering> segmented_filtering_;
  std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> feature_extraction_;
  std::shared_ptr<tracking::BaseMultiTracking<Feature>> multi_tracking_;
  std::shared_ptr<visualization::LaserObjectTrackerVisualization> visualization_;
};
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_MULTI_TRACKER_ROS_HPP
