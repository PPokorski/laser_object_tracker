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

#include "laser_object_tracker/multi_tracker_ros.hpp"

namespace laser_object_tracker {
MultiTrackerROS::MultiTrackerROS(const ros::NodeHandle& node_handle)
  : node_handle_(node_handle),
    sub_laser_scan(node_handle_.subscribe("in_laser_scan",
                                          1,
                                          &MultiTrackerROS::laserScanCallback,
                                          this)),
    is_scan_updated_(false),
    scan_fragment_factory_(),
    last_scan_fragment_(),
    segmentation_(getSegmentation(node_handle_)),
    segmented_filtering_(getSegmentedFiltering(node_handle_)),
    feature_extraction_(getFeatureExtraction(node_handle_)),
    multi_tracking_(getMultiTracking(node_handle_)),
    visualization_(getVisualization(node_handle_)) {}

void MultiTrackerROS::update() {
  if (!is_scan_updated_) {
    ROS_WARN("None laser scan received!");
    return;
  }

  // PREDICTION
  multi_tracking_->predict();

  if (last_scan_fragment_.empty()) {
    ROS_WARN("Laser scan fragment is empty!");
    return;
  }

  visualization_->clearMarkers();
  visualization_->publishPointCloud(last_scan_fragment_);

  // SEGMENTATION
  auto segments = segmentation_->segment(last_scan_fragment_);

  // FILTERING
  segmented_filtering_->filter(segments);
  visualization_->publishPointClouds(segments);

  // FEATURES EXTRACTION
  auto features = feature_extraction_->extractFeatures(segments);
  visualization_->publishObjects(features);

  // UPDATING
  multi_tracking_->update(features);
  visualization_->publishMultiTracker(multi_tracking_);

  visualization_->trigger();
  is_scan_updated_ = false;
}

void MultiTrackerROS::laserScanCallback(const sensor_msgs::LaserScan::Ptr& laser_scan) {
  last_scan_fragment_ = scan_fragment_factory_.fromLaserScan(std::move(*laser_scan));

  is_scan_updated_ = true;
}

std::shared_ptr<segmentation::BaseSegmentation> MultiTrackerROS::getSegmentation(ros::NodeHandle& node_handle) {
  std::shared_ptr<segmentation::BaseSegmentation> segmentation;
  double angle, sigma;
  node_handle.getParam("segmentation/angle", angle);
  node_handle.getParam("segmentation/sigma", sigma);

  segmentation.reset(new segmentation::AdaptiveBreakpointDetection(angle, sigma));
  return segmentation;
}

std::shared_ptr<filtering::BaseSegmentedFiltering> MultiTrackerROS::getSegmentedFiltering(ros::NodeHandle& node_handle) {
  std::shared_ptr<filtering::BaseSegmentedFiltering> segmented_filtering;
  int min_points, max_points;
  node_handle.getParam("filtering/min_points", min_points);
  node_handle.getParam("filtering/max_points", max_points);
  auto points = std::make_unique<laser_object_tracker::filtering::PointsNumberFilter>(min_points, max_points);

  double min_area, max_area, min_dimension;
  node_handle.getParam("filtering/min_area", min_area);
  node_handle.getParam("filtering/max_area", max_area);
  node_handle.getParam("filtering/min_dimension", min_dimension);
  auto obb = std::make_unique<laser_object_tracker::filtering::OBBAreaFilter>(min_area, max_area, min_dimension);

  std::vector<std::unique_ptr<laser_object_tracker::filtering::BaseSegmentedFiltering>> filters;
  filters.push_back(std::move(points));
  filters.push_back(std::move(obb));

  segmented_filtering.reset(new filtering::AggregateSegmentedFiltering(std::move(filters)));

  return segmented_filtering;
}

std::shared_ptr<feature_extraction::BaseFeatureExtraction<MultiTrackerROS::Feature>>
MultiTrackerROS::getFeatureExtraction(ros::NodeHandle& node_handle) {
  std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> feature_extraction;
  double min_angle_between_lines, max_distance, rho_resolution, theta_resolution;
  int voting_threshold;
  node_handle.getParam("feature_extraction/min_angle_between_lines", min_angle_between_lines);
  node_handle.getParam("feature_extraction/max_distance", max_distance);
  node_handle.getParam("feature_extraction/rho_resolution", rho_resolution);
  node_handle.getParam("feature_extraction/theta_resolution", theta_resolution);
  node_handle.getParam("feature_extraction/voting_threshold", voting_threshold);

  feature_extraction.reset(new feature_extraction::MultiLineDetection(min_angle_between_lines,
                                                                      max_distance,
                                                                      rho_resolution,
                                                                      theta_resolution,
                                                                      voting_threshold));
  return feature_extraction;
}

std::shared_ptr<tracking::BaseMultiTracking<MultiTrackerROS::Feature>>
MultiTrackerROS::getMultiTracking(ros::NodeHandle& node_handle) {
  std::shared_ptr<tracking::BaseMultiTracking<Feature>> multi_tracking;
  double time_step,
         max_mahalanobis_distance,
         skip_decay_rate,
         probability_start,
         probability_detection,
         mean_false_alarms,
         min_g_hypothesis_ratio;

  int max_depth,
      max_g_hypothesis;

  std::vector<double> measurement_noise_covariance_data;
  std::vector<double> initial_state_covariance_data;
  std::vector<double> process_noise_covariance_data;

  node_handle.getParam("tracking/time_step", time_step);
  node_handle.getParam("tracking/max_mahalanobis_distance", max_mahalanobis_distance);
  node_handle.getParam("tracking/skip_decay_rate", skip_decay_rate);
  node_handle.getParam("tracking/probability_start", probability_start);
  node_handle.getParam("tracking/probability_detection", probability_detection);
  node_handle.getParam("tracking/measurement_noise_covariance", measurement_noise_covariance_data);
  node_handle.getParam("tracking/initial_state_covariance", initial_state_covariance_data);
  node_handle.getParam("tracking/process_noise_covariance", process_noise_covariance_data);
  node_handle.getParam("tracking/mean_false_alarms", mean_false_alarms);
  node_handle.getParam("tracking/max_depth", max_depth);
  node_handle.getParam("tracking/min_g_hypothesis_ratio", min_g_hypothesis_ratio);
  node_handle.getParam("tracking/max_g_hypothesis", max_g_hypothesis);

  tracking::mht::ObjectState::MeasurementNoiseCovariance measurement_noise_covariance(
      measurement_noise_covariance_data.data());
  tracking::mht::ObjectState::InitialStateCovariance initial_state_covariance(
      initial_state_covariance_data.data());
  tracking::mht::ObjectState::ProcessNoiseCovariance process_noise_covariance(
      process_noise_covariance_data.data());

  multi_tracking.reset(new tracking::MultiHypothesisTracking(time_step,
                                                                max_mahalanobis_distance,
                                                                skip_decay_rate,
                                                                probability_start,
                                                                probability_detection,
                                                                measurement_noise_covariance,
                                                                initial_state_covariance,
                                                                process_noise_covariance,
                                                                mean_false_alarms,
                                                                max_depth,
                                                                min_g_hypothesis_ratio,
                                                                max_g_hypothesis));
  return multi_tracking;
}

std::shared_ptr<visualization::LaserObjectTrackerVisualization> MultiTrackerROS::getVisualization(ros::NodeHandle& node_handle) {
  std::string base_frame;
  node_handle.getParam("base_frame", base_frame);

  return std::make_shared<visualization::LaserObjectTrackerVisualization>(node_handle, base_frame);
}
}  // namespace laser_object_tracker
