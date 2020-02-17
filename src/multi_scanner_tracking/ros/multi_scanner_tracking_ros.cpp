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

#include "laser_object_tracker/multi_scanner_tracking/ros/multi_scanner_tracking_ros.hpp"

namespace laser_object_tracker {
namespace multi_scanner_tracking {
namespace ros {
MultiScannerTrackingROS::MultiScannerTrackingROS(::ros::NodeHandle& node_handle)
    : node_handle_(node_handle),
      pub_tracks_(node_handle.advertise<laser_object_tracker_msgs::TrackArray>("out_tracks/unified", 1, true)),
      scanners_number_(getParam<int>(node_handle_, "scanners_number")),
      track_unifying_(getTrackUnifying(node_handle_)),
      visualization_(getVisualization(node_handle_)) {
  getParam(node_handle_, "base_frame", base_frame_);
  ros_trackers_.reserve(scanners_number_);
  for (int i = 0; i < scanners_number_; ++i) {
    ros_trackers_.emplace_back(i, node_handle_);
  }
}

void MultiScannerTrackingROS::update() {
  std::map<int, std::optional<tracking::MultiHypothesisTracking::Container>> tracks;
  for (auto& tracker: ros_trackers_) {
    tracks.emplace(tracker.getID(), tracker.update());
  }

  if (std::any_of(tracks.begin(), tracks.end(), [](const auto& id_track) {return id_track.second;})) {
    visualization_.clearMarkers();
    auto unified_tracks = track_unifying_.unifyTracks(tracks);
    pub_tracks_.publish(tracking::ros::toROSMsg(unified_tracks, ::ros::Time::now(), base_frame_));
    visualization_.publishMultiTracker(unified_tracks);
    visualization_.trigger();
  }
}

track_unifying::TrackUnifying MultiScannerTrackingROS::getTrackUnifying(
    ::ros::NodeHandle& node_handle) {
  int angle_threshold, distance_threshold;
  getParam(node_handle, "track_unifying/angle_threshold", angle_threshold);
  getParam(node_handle, "track_unifying/distance_threshold", distance_threshold);

  return track_unifying::TrackUnifying(angle_threshold, distance_threshold);
}

visualization::LaserObjectTrackerVisualization MultiScannerTrackingROS::getVisualization(
    ::ros::NodeHandle& node_handle) {
  std::string base_frame;
  getParam(node_handle, "base_frame", base_frame);

  return visualization::LaserObjectTrackerVisualization("unified",
                                                        node_handle,
                                                        base_frame);
}
}  // namespace ros
}  // namespace multi_scanner_tracking
}  // namespace laser_object_tracker
