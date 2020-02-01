/*********************************************************************
*
* BSD 3-Clause License
*
*  Copyright (c) 2020, Piotr Pokorski
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

#ifndef LASER_OBJECT_TRACKER_MULTI_SCANNER_TRACKING_ROS_MULTI_SCANNER_TRACKING_ROS_HPP
#define LASER_OBJECT_TRACKER_MULTI_SCANNER_TRACKING_ROS_MULTI_SCANNER_TRACKING_ROS_HPP

#include <vector>

#include <ros/ros.h>

#include "laser_object_tracker/multi_scanner_tracking/track_unifying/track_unifying.hpp"
#include "laser_object_tracker/tracking/ros/multi_tracking_ros.hpp"
#include "laser_object_tracker/visualization/laser_object_tracker_visualization.hpp"

namespace laser_object_tracker {
namespace multi_scanner_tracking {
namespace ros {

class MultiScannerTrackingROS {
 public:
  explicit MultiScannerTrackingROS(const ::ros::NodeHandle& node_handle);

  void update();

 private:
  track_unifying::TrackUnifying getTrackUnifying(
      ::ros::NodeHandle& node_handle);
  visualization::LaserObjectTrackerVisualization getVisualization(
      ::ros::NodeHandle& node_handle);

  ::ros::NodeHandle node_handle_;

  int scanners_number_;

  std::vector<tracking::ros::MultiTrackingROS> ros_trackers_;
  track_unifying::TrackUnifying track_unifying_;

  visualization::LaserObjectTrackerVisualization visualization_;
};

}  // namespace ros
}  // namespace multi_scanner_tracking
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_MULTI_SCANNER_TRACKING_ROS_MULTI_SCANNER_TRACKING_ROS_HPP
