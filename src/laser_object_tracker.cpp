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

#include <chrono>

#include "laser_object_tracker/multi_tracker_ros.hpp"
#include "laser_object_tracker/track_unifying.hpp"

int main(int ac, char **av) {
  ros::init(ac, av, "laser_object_tracker");
  ros::NodeHandle pnh("~");

  ros::Rate rate(10.0);
  int scanners_number;
  laser_object_tracker::getParam(pnh, "scanners_number", scanners_number);
  std::vector<laser_object_tracker::MultiTrackerROS> trackers_ros;
  trackers_ros.reserve(scanners_number);
  for (int i = 0; i < scanners_number; ++i) {
    trackers_ros.emplace_back(i, pnh);
  }
  ROS_INFO("Done initialization");

  // TODO Use parameters, move to one common class
  // Add support for velocity difference
  laser_object_tracker::track_unifying::TrackUnifying track_unifying(0.52, 0.5);

  std::string base_frame;
  laser_object_tracker::getParam(pnh, "base_frame", base_frame);
  laser_object_tracker::visualization::LaserObjectTrackerVisualization vis(10, pnh, base_frame);

  std::chrono::high_resolution_clock::time_point begin, end;
  while (ros::ok()) {
    vis.clearMarkers();
    ros::spinOnce();
    begin = std::chrono::high_resolution_clock::now();

    std::map<int, laser_object_tracker::tracking::MultiHypothesisTracking::Container> tracks;

    for (auto& tracker : trackers_ros) {
      tracks.emplace(tracker.getID(), tracker.update());
    }
    if (std::any_of(tracks.begin(), tracks.end(), [](const auto& id_track) {return !id_track.second.empty();})) {
      vis.publishMultiTracker(track_unifying.unifyTracks(tracks));
    }
    vis.trigger();

    end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - begin;
    ROS_INFO("Iteration took: %.2f ms", (duration.count()));

    rate.sleep();
  }
  return 0;
}
