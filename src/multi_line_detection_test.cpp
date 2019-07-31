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

#include "laser_object_tracker/feature_extraction/feature_extraction.hpp"
#include "laser_object_tracker/segmentation/segmentation.hpp"
#include "laser_object_tracker/visualization/visualization.hpp"

using Feature = laser_object_tracker::feature_extraction::features::Object;

laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;
laser_object_tracker::data_types::LaserScanFragment fragment;

void laserScanCallback(const sensor_msgs::LaserScan::Ptr& laser_scan) {
  fragment = factory.fromLaserScan(std::move(*laser_scan));
}

using namespace laser_object_tracker;

std::shared_ptr<segmentation::BaseSegmentation> getSegmentation(ros::NodeHandle& nh) {
  std::shared_ptr<segmentation::BaseSegmentation> segmentation;

  std::string type;
  nh.getParam("segmentation/type", type);
  if (type == "BreakpointDetection") {
    double threshold;
    nh.getParam("segmentation/threshold", threshold);

    segmentation.reset(new segmentation::BreakpointDetection(threshold));
  } else if (type == "AdaptiveThresholdDetection") {
    double angle, sigma;
    nh.getParam("segmentation/angle", angle);
    nh.getParam("segmentation/sigma", sigma);

    segmentation.reset(new segmentation::AdaptiveBreakpointDetection(angle, sigma));
  }

  return segmentation;
}

std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> getFeatureExtraction(ros::NodeHandle& nh) {
  std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> feature_extraction;

  double min_angle_between_lines, max_distance, rho_resolution, theta_resolution;
  int voting_threshold;
  nh.getParam("feature_extraction/min_angle_between_lines", min_angle_between_lines);
  nh.getParam("feature_extraction/max_distance", max_distance);
  nh.getParam("feature_extraction/rho_resolution", rho_resolution);
  nh.getParam("feature_extraction/theta_resolution", theta_resolution);
  nh.getParam("feature_extraction/voting_threshold", voting_threshold);

  feature_extraction.reset(new feature_extraction::MultiLineDetection(min_angle_between_lines,
                                                                      max_distance,
                                                                      rho_resolution,
                                                                      theta_resolution,
                                                                      voting_threshold));

  return feature_extraction;
}

int main(int ac, char **av) {
  ros::init(ac, av, "multi_line_detection_test");
  ros::NodeHandle pnh("~");

  auto segmentation = getSegmentation(pnh);
  auto feature_extraction = getFeatureExtraction(pnh);

  std::string base_frame;
  pnh.getParam("base_frame", base_frame);
  laser_object_tracker::visualization::LaserObjectTrackerVisualization visualization(pnh, base_frame);
  ros::Subscriber subscriber_laser_scan = pnh.subscribe("in_scan", 1, laserScanCallback);
  ros::Rate rate(10.0);

  std::chrono::high_resolution_clock::time_point begin, end;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    if (!fragment.empty()) {
      visualization.clearMarkers();
      visualization.publishPointCloud(fragment);
      auto segments = segmentation->segment(fragment);

      visualization.publishPointClouds(segments);

      std::vector<Feature> features;
      Feature feature;
      begin = std::chrono::high_resolution_clock::now();
      for (const auto& segment : segments) {
        if (segment.isValid()) {
          if (feature_extraction->extractFeature(segment, feature)) {
            features.push_back(feature);
          } else {
          }
        }
      }
      end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> duration = end - begin;
      ROS_INFO("Iteration took: %.2f ms", (duration.count()));

      visualization.publishObjects(features);
      visualization.trigger();
    } else {
      ROS_WARN("Received laser scan is empty");
    }
  }
}