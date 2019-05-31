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

#include "laser_object_tracker/data_association/data_association.hpp"
#include "laser_object_tracker/data_types/data_types.hpp"
#include "laser_object_tracker/feature_extraction/feature_extraction.hpp"
#include "laser_object_tracker/filtering/filtering.hpp"
#include "laser_object_tracker/segmentation/segmentation.hpp"
#include "laser_object_tracker/visualization/visualization.hpp"
#include "laser_object_tracker/tracking/tracking.hpp"

laser_object_tracker::data_types::LaserScanFragment::LaserScanFragmentFactory factory;
laser_object_tracker::data_types::LaserScanFragment fragment;

void laserScanCallback(const sensor_msgs::LaserScan::Ptr& laser_scan) {
  ROS_INFO("Received laser scan");
  fragment = factory.fromLaserScan(std::move(*laser_scan));

  ROS_INFO("Fragment has %d elements.", fragment.size());
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

std::map<std::string, feature_extraction::SearchBasedCornerDetection::CriterionFunctor> getCriterions() {
  return {{"areaCriterion", feature_extraction::areaCriterion},
          {"closenessCriterion", feature_extraction::closenessCriterion},
          {"varianceCriterion", feature_extraction::varianceCriterion}};
}

std::shared_ptr<laser_object_tracker::filtering::BaseSegmentedFiltering> getFiltering(ros::NodeHandle& nh) {
  int min_points, max_points;
  nh.getParam("filtering/min_points", min_points);
  nh.getParam("filtering/max_points", max_points);
  auto points = std::make_unique<laser_object_tracker::filtering::PointsNumberFilter>(min_points, max_points);

  double min_area, max_area, min_dimension;
  nh.getParam("filtering/min_area", min_area);
  nh.getParam("filtering/max_area", max_area);
  nh.getParam("filtering/min_dimension", min_dimension);
  auto obb = std::make_unique<laser_object_tracker::filtering::OBBAreaFilter>(min_area, max_area, min_dimension);

  std::vector<std::unique_ptr<laser_object_tracker::filtering::BaseSegmentedFiltering>> filters;
  filters.push_back(std::move(points));
  filters.push_back(std::move(obb));

  return std::make_unique<laser_object_tracker::filtering::AggregateSegmentedFiltering>(std::move(filters));
}

std::unique_ptr<laser_object_tracker::tracking::BaseTracking> getTracker() {
//  Eigen::MatrixXd transition(4, 4);
//  transition << 1.0, 0.0, 0.1, 0.0,
//                0.0, 1.0, 0.0, 0.1,
//                0.0, 0.0, 1.0, 0.0,
//                0.0, 0.0, 0.0, 1.0;

//  Eigen::MatrixXd measurement(2, 4);
//  measurement << 1.0, 0.0, 0.0, 0.0,
//                 0.0, 1.0, 0.0, 0.0;

  Eigen::MatrixXd process_noise_covariance(6, 6);
  process_noise_covariance << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 0.1;

  Eigen::MatrixXd measurement_noise_covariance(3, 3);
  measurement_noise_covariance << 0.01, 0.00, 0.00,
                                  0.00, 0.01, 0.00,
                                  0.00, 0.00, 0.01;

  Eigen::MatrixXd initial_state_covariance(6, 6);
  initial_state_covariance << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.3, 0.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.3, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                              0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  return std::make_unique<laser_object_tracker::tracking::CornerTracker>(0.1, 0.1,
                          measurement_noise_covariance,
                          initial_state_covariance,
                          process_noise_covariance);
}

std::unique_ptr<laser_object_tracker::data_association::BaseDataAssociation> getDataAssociation(ros::NodeHandle& nh) {
  double max_cost;
  nh.getParam("data_association/max_cost", max_cost);
  return std::make_unique<laser_object_tracker::data_association::HungarianAlgorithm>(max_cost);
}

laser_object_tracker::tracking::MultiTracker::DistanceFunctor getDistanceFunctor() {
  return [](const laser_object_tracker::feature_extraction::features::Feature& observation,
           const laser_object_tracker::tracking::BaseTracking& tracker) {
    return (observation.observation_.head<2>() - tracker.getStateVector().head<2>()).squaredNorm();
  };
}

std::unique_ptr<laser_object_tracker::tracking::BaseTrackerRejection> getTrackerRejection() {
  return std::make_unique<laser_object_tracker::tracking::IterationTrackerRejection>(40);
}

int main(int ac, char **av) {
  pcl::PointCloud<pcl::PointXYZ> pcl;
  pcl::SampleConsenusModelCross2D<pcl::PointXYZ> corner(pcl.makeShared());

  ros::init(ac, av, "laser_object_detector");
  ros::NodeHandle pnh("~");

  ROS_INFO("Initializing segmentation");
  auto segmentation = getSegmentation(pnh);
  auto filtering = getFiltering(pnh);

  std::string feature_type;
  double angle_resolution;
  std::string criterion_name;
  pnh.getParam("feature_extraction/type", feature_type);
  pnh.getParam("feature_extraction/angle_resolution", angle_resolution);
  pnh.getParam("feature_extraction/criterion", criterion_name);

  feature_extraction::SearchBasedCornerDetection::CriterionFunctor criterion;
  try {
    criterion = getCriterions().at(criterion_name);
  } catch (std::exception& e) {
    ROS_ERROR("%s", e.what());
    throw;
  }
  feature_extraction::SearchBasedCornerDetection detection(angle_resolution, criterion);

  ROS_INFO("Initializing visualization");
  std::string base_frame;
  pnh.getParam("base_frame", base_frame);
  laser_object_tracker::visualization::LaserObjectTrackerVisualization visualization(pnh, base_frame);
  ROS_INFO("Initializing subscriber");
  ros::Subscriber subscriber_laser_scan = pnh.subscribe("in_scan", 1, laserScanCallback);

  ros::Rate rate(10.0);
  ROS_INFO("Done initialization");

  laser_object_tracker::tracking::MultiTracker multi_tracker(
      getDistanceFunctor(),
      getDataAssociation(pnh),
      getTracker(),
      getTrackerRejection());

  while (ros::ok()) {
    ros::spinOnce();
    multi_tracker.predict();

    if (!fragment.empty()) {
      visualization.clearMarkers();
      visualization.publishPointCloud(fragment);
      auto segments = segmentation->segment(fragment);
      filtering->filter(segments);
      ROS_INFO("Detected %lu segments", segments.size());
//      visualization.publishFeatures(segments);

      visualization.publishPointClouds(segments);
      laser_object_tracker::feature_extraction::features::Corners2D corners_2_d;
      laser_object_tracker::feature_extraction::features::Feature feature;
      std::vector<laser_object_tracker::feature_extraction::features::Feature> features;
      for (const auto& segment : segments) {
        if (segment.isValid()) {
          if (detection.extractFeature(segment, feature)) {
            features.push_back(feature);
//            features.push_back({feature.observation_.head<2>(), std::vector<int>(), std::vector<bool>()});
//            std::cout << "Feature vector:\n" << feature.head(2) << std::endl;
            corners_2_d.push_back(laser_object_tracker::feature_extraction::features::Corner2D(feature.observation_));
          }
        }
      }

//      Eigen::MatrixXd cost_matrix = multi_tracker.buildCostMatrix(features);
//      Eigen::VectorXi assignment_vector = multi_tracker.buildAssignmentVector(cost_matrix);
//      visualization.publishAssignments(multi_tracker, features, cost_matrix, assignment_vector);

      multi_tracker.update(features);
      visualization.publishCorners(corners_2_d);
      visualization.publishMultiTracker(multi_tracker);

      visualization.trigger();
    } else {
      ROS_WARN("Received laser scan is empty");
    }

    rate.sleep();
  }
  return 0;
}
