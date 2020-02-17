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

#include "laser_object_tracker/tracking/ros/multi_tracking_ros.hpp"

#include "laser_object_tracker/filtering/occlusion_detection.hpp"

namespace laser_object_tracker {
namespace tracking {
namespace ros {
laser_object_tracker_msgs::TrackElement toROSMsg(const ObjectTrackElement& track_element, const std::string& frame) {
  laser_object_tracker_msgs::TrackElement ros_track_element {};
  ros_track_element.header.stamp = track_element.timestamp_;
  ros_track_element.header.frame_id = frame;

  ros_track_element.position.pose.position.x = track_element.position_.x();
  ros_track_element.position.pose.position.y = track_element.position_.y();
  ros_track_element.position.pose.orientation.w = 1.0;

  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> covariance = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>::Zero();
  covariance.topLeftCorner<2, 2>() = track_element.position_covariance_;
  std::copy(covariance.data(), covariance.data() + covariance.size(), ros_track_element.position.covariance.data());

  ros_track_element.velocity.twist.linear.x = track_element.velocity_.x();
  ros_track_element.velocity.twist.linear.y = track_element.velocity_.y();
  covariance.topLeftCorner<2, 2>() = track_element.velocity_covariance_;
  std::copy(covariance.data(), covariance.data() + covariance.size(), ros_track_element.velocity.covariance.data());

  ros_track_element.polygon.points.reserve(track_element.polyline_.size());
  for (const auto& point : track_element.polyline_) {
    geometry_msgs::Point32 polygon_point;
    polygon_point.x = point.x();
    polygon_point.y = point.y();
    polygon_point.z = 0.0f;
    ros_track_element.polygon.points.push_back(polygon_point);
  }

  return ros_track_element;
}

laser_object_tracker_msgs::Track toROSMsg(const ObjectTrack& track,
                                          const ::ros::Time& stamp,
                                          const std::string& frame) {
  laser_object_tracker_msgs::Track ros_track {};
  ros_track.header.stamp = stamp;
  ros_track.header.frame_id = frame;

  ros_track.id = track.id_;
  ros_track.track.reserve(track.track_.size());
  for (const auto& track_element : track.track_) {
    ros_track.track.push_back(toROSMsg(track_element, frame));
  }

  return ros_track;
}

laser_object_tracker_msgs::TrackArray toROSMsg(const std::vector<ObjectTrack>& tracks,
                                               const ::ros::Time& stamp,
                                               const std::string& frame) {
  laser_object_tracker_msgs::TrackArray ros_tracks {};
  ros_tracks.header.stamp = stamp;
  ros_tracks.header.frame_id = frame;

  ros_tracks.tracks.reserve(tracks.size());
  for (const auto& track : tracks) {
    ros_tracks.tracks.push_back(toROSMsg(track, stamp, frame));
  }

  return ros_tracks;
}

MultiTrackingROS::MultiTrackingROS(int id, const ::ros::NodeHandle& node_handle)
  : id_(id),
    node_handle_(node_handle),
    sub_laser_scan_(node_handle_.subscribe("in_laser_scan/" + std::to_string(id_),
                                           1,
                                           &MultiTrackingROS::laserScanCallback,
                                           this)),
    pub_tracks_(node_handle_.advertise<laser_object_tracker_msgs::TrackArray>("out_tracks/" + std::to_string(id_),
                                                                              1,
                                                                              true)),
    is_scan_updated_(false),
    scan_fragment_factory_(),
    last_scan_fragment_(),
    segmentation_(getSegmentation(node_handle_)),
    segmented_filtering_(getSegmentedFiltering(node_handle_)),
    feature_extraction_(getFeatureExtraction(node_handle_)),
    multi_tracking_(getMultiTracking(node_handle_)),
    visualization_(getVisualization(id, node_handle_)) {
  getParam(node_handle_, "base_frame", base_frame_);
  double transform_wait_timeout;
  getParam(node_handle_, "transform_wait_timeout", transform_wait_timeout);
  transform_wait_timeout_ = transform_wait_timeout_.fromSec(transform_wait_timeout);
}

std::optional<tracking::BaseMultiTracking<MultiTrackingROS::Feature, MultiTrackingROS::Track>::Container>
MultiTrackingROS::update() {
  std::optional<tracking::BaseMultiTracking<Feature, Track>::Container> tracks_optional;
  if (!is_scan_updated_) {
//    ROS_WARN("None laser scan received!");
    return tracks_optional;
  }

  // PREDICTION
  multi_tracking_->predict();

  if (last_scan_fragment_.empty()) {
    ROS_WARN("Laser scan fragment is empty!");
    tracks_optional.emplace();
    return tracks_optional;
  }
  ROS_INFO("Updating no.%d", id_);

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
  tracks_optional.emplace(multi_tracking_->update(features));
  pub_tracks_.publish(toROSMsg(*tracks_optional, last_scan_fragment_.getHeader().stamp, base_frame_));
  visualization_->publishMultiTracker(multi_tracking_);

  visualization_->trigger();
  is_scan_updated_ = false;

  return tracks_optional;
}

void MultiTrackingROS::laserScanCallback(const sensor_msgs::LaserScan::Ptr& laser_scan) {
  last_scan_fragment_ = scan_fragment_factory_.fromLaserScan(std::move(*laser_scan),
                                                             base_frame_,
                                                             transform_wait_timeout_);

  is_scan_updated_ = true;
}

std::shared_ptr<segmentation::BaseSegmentation> MultiTrackingROS::getSegmentation(::ros::NodeHandle& node_handle) {
  std::shared_ptr<segmentation::BaseSegmentation> segmentation;
  double angle, sigma;
  getParam(node_handle, "segmentation/angle", angle);
  getParam(node_handle, "segmentation/sigma", sigma);

  segmentation.reset(new segmentation::AdaptiveBreakpointDetection(angle, sigma));
  return segmentation;
}

std::shared_ptr<filtering::BaseSegmentedFiltering> MultiTrackingROS::getSegmentedFiltering(::ros::NodeHandle& node_handle) {
  std::shared_ptr<filtering::BaseSegmentedFiltering> segmented_filtering;

  double max_angle_gap;
  getParam(node_handle, "filtering/occlusion/max_angle_gap", max_angle_gap);
  auto occlusion = std::make_unique<filtering::OcclusionDetection>(max_angle_gap);

  int min_points, max_points;
  getParam(node_handle, "filtering/points_number/min_points", min_points);
  getParam(node_handle, "filtering/points_number/max_points", max_points);
  auto points = std::make_unique<filtering::PointsNumberFilter>(min_points, max_points);

  double min_area, max_area, min_dimension;
  getParam(node_handle, "filtering/area/min_area", min_area);
  getParam(node_handle, "filtering/area/max_area", max_area);
  getParam(node_handle, "filtering/area/min_dimension", min_dimension);
  auto obb = std::make_unique<filtering::OBBAreaFilter>(min_area, max_area, min_dimension);

  std::vector<std::unique_ptr<filtering::BaseSegmentedFiltering>> filters;
  filters.push_back(std::move(occlusion));
  filters.push_back(std::move(points));
  filters.push_back(std::move(obb));

  segmented_filtering.reset(new filtering::AggregateSegmentedFiltering(std::move(filters)));

  return segmented_filtering;
}

std::shared_ptr<feature_extraction::BaseFeatureExtraction<MultiTrackingROS::Feature>>
MultiTrackingROS::getFeatureExtraction(::ros::NodeHandle& node_handle) {
  std::shared_ptr<feature_extraction::BaseFeatureExtraction<Feature>> feature_extraction;
  double occlusion_distance_threshold,
         min_angle_between_lines,
         max_distance,
         rho_resolution,
         theta_resolution;
  int voting_threshold;
  getParam(node_handle, "feature_extraction/occlusion_checking/distance_threshold", occlusion_distance_threshold);
  getParam(node_handle, "feature_extraction/min_angle_between_lines", min_angle_between_lines);
  getParam(node_handle, "feature_extraction/max_distance", max_distance);
  getParam(node_handle, "feature_extraction/rho_resolution", rho_resolution);
  getParam(node_handle, "feature_extraction/theta_resolution", theta_resolution);
  getParam(node_handle, "feature_extraction/voting_threshold", voting_threshold);

  feature_extraction::occlusion_checking::DistanceOcclusionChecking occlusion_checking(occlusion_distance_threshold);

  feature_extraction.reset(new feature_extraction::MultiLineDetection(occlusion_checking,
                                                                         min_angle_between_lines,
                                                                         max_distance,
                                                                         rho_resolution,
                                                                         theta_resolution,
                                                                         voting_threshold));
  return feature_extraction;
}

std::shared_ptr<tracking::BaseMultiTracking<MultiTrackingROS::Feature, MultiTrackingROS::Track>>
MultiTrackingROS::getMultiTracking(::ros::NodeHandle& node_handle) {
  std::shared_ptr<tracking::BaseMultiTracking<Feature, Track>> multi_tracking;
  double buffer_length,
         min_buffer_filling,
         distance_threshold,
         orientation_angle_threshold,
         aperture_angle_threshold,
         time_step,
         max_mahalanobis_distance,
         target_confirmation_rate,
         target_confirmation_threshold,
         hold_target_probability,
         probability_start,
         probability_detection,
         min_velocity,
         false_alarm_likelihood,
         min_g_hypothesis_ratio;

  int max_depth,
      max_g_hypothesis;

  std::vector<double> measurement_noise_covariance_data;
  std::vector<double> initial_state_covariance_data;
  std::vector<double> process_noise_covariance_data;

  getParam(node_handle, "tracking/object_matching/buffer_length", buffer_length);
  getParam(node_handle, "tracking/object_matching/min_buffer_filling", min_buffer_filling);
  getParam(node_handle, "tracking/object_matching/distance_threshold", distance_threshold);
  getParam(node_handle, "tracking/object_matching/orientation_angle_threshold", orientation_angle_threshold);
  getParam(node_handle, "tracking/object_matching/aperture_angle_threshold", aperture_angle_threshold);
  getParam(node_handle, "tracking/model/time_step", time_step);
  getParam(node_handle, "tracking/model/max_mahalanobis_distance", max_mahalanobis_distance);
  getParam(node_handle, "tracking/model/target_confirmation_rate", target_confirmation_rate);
  getParam(node_handle, "tracking/model/target_confirmation_threshold", target_confirmation_threshold);
  getParam(node_handle, "tracking/model/hold_target_probability", hold_target_probability);
  getParam(node_handle, "tracking/model/probability_start", probability_start);
  getParam(node_handle, "tracking/model/probability_detection", probability_detection);
  getParam(node_handle, "tracking/model/measurement_noise_covariance", measurement_noise_covariance_data);
  getParam(node_handle, "tracking/model/initial_state_covariance", initial_state_covariance_data);
  getParam(node_handle, "tracking/model/process_noise_covariance", process_noise_covariance_data);
  getParam(node_handle, "tracking/min_velocity", min_velocity);
  getParam(node_handle, "tracking/false_alarm_likelihood", false_alarm_likelihood);
  getParam(node_handle, "tracking/max_depth", max_depth);
  getParam(node_handle, "tracking/min_g_hypothesis_ratio", min_g_hypothesis_ratio);
  getParam(node_handle, "tracking/max_g_hypothesis", max_g_hypothesis);

  tracking::mht::ObjectState::MeasurementNoiseCovariance measurement_noise_covariance(
      measurement_noise_covariance_data.data());
  tracking::mht::ObjectState::InitialStateCovariance initial_state_covariance(
      initial_state_covariance_data.data());
  tracking::mht::ObjectState::ProcessNoiseCovariance process_noise_covariance(
      process_noise_covariance_data.data());

  tracking::object_matching::FastObjectMatching fast_object_matching(
      buffer_length,
      min_buffer_filling,
      distance_threshold,
      orientation_angle_threshold,
      aperture_angle_threshold);

  auto* model = new tracking::mht::ObjectModel(
      time_step,
      max_mahalanobis_distance,
      target_confirmation_rate,
      target_confirmation_threshold,
      hold_target_probability,
      probability_start,
      probability_detection,
      measurement_noise_covariance,
      initial_state_covariance,
      process_noise_covariance);

  multi_tracking.reset(new tracking::MultiHypothesisTracking(fast_object_matching,
                                                             model,
                                                             min_velocity,
                                                             false_alarm_likelihood,
                                                             max_depth,
                                                             min_g_hypothesis_ratio,
                                                             max_g_hypothesis));
  return multi_tracking;
}

std::shared_ptr<visualization::LaserObjectTrackerVisualization> MultiTrackingROS::getVisualization(
    int id,
    ::ros::NodeHandle& node_handle) {
  std::string base_frame;
  getParam(node_handle, "base_frame", base_frame);

  return std::make_shared<visualization::LaserObjectTrackerVisualization>("scanner/" + std::to_string(id),
                                                                          node_handle,
                                                                          base_frame);
}
}  // namespace ros
}  // namespace tracking
}  // namespace laser_object_tracker
