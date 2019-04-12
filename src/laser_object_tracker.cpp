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

#include "laser_object_tracker/feature_extraction/random_sample_consensus_corner_detection.hpp"
#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"
#include "laser_object_tracker/feature_extraction/search_based_corner_detection.hpp"
#include "laser_object_tracker/segmentation/adaptive_breakpoint_detection.hpp"
#include "laser_object_tracker/segmentation/breakpoint_detection.hpp"
#include "laser_object_tracker/visualization/laser_object_tracker_visualization.hpp"

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
    if (type == "BreakpointDetection")
    {
        double threshold;
        nh.getParam("segmentation/threshold", threshold);

        segmentation.reset(new segmentation::BreakpointDetection(threshold));
    }
    else if (type == "AdaptiveThresholdDetection")
    {
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

#include "laser_object_tracker/feature_extraction/pcl/sac_model_cross2d.hpp"

int main(int ac, char** av) {
    pcl::PointCloud<pcl::PointXYZ> pcl;
    pcl::SampleConsenusModelCross2D<pcl::PointXYZ> corner(pcl.makeShared());

    ros::init(ac, av, "laser_object_detector");
    ros::NodeHandle pnh("~");

    ROS_INFO("Initializing segmentation");
    auto segmentation = getSegmentation(pnh);

//    std::string feature_type;
//    double angle_resolution;
//    std::string criterion_name;
//    pnh.getParam("feature_extraction/type", feature_type);
//    pnh.getParam("feature_extraction/angle_resolution", angle_resolution);
//    pnh.getParam("feature_extraction/criterion", criterion_name);
//
//    feature_extraction::SearchBasedCornerDetection::CriterionFunctor criterion;
//    try
//    {
//        criterion = getCriterions().at(criterion_name);
//    }
//    catch (std::exception& e)
//    {
//        ROS_ERROR("%s", e.what());
//        throw;
//    }
    double distance_threshold;
    int max_iterations;
    double probability;
    pnh.getParam("feature_extraction/distance_threshold", distance_threshold);
    pnh.getParam("feature_extraction/max_iterations", max_iterations);
    pnh.getParam("feature_extraction/probability", probability);

    laser_object_tracker::feature_extraction::RandomSampleConsensusCornerDetection detection(distance_threshold,
            max_iterations,
            probability);
    ROS_INFO("Initializing visualization");

    std::string base_frame;
    pnh.getParam("base_frame", base_frame);
    laser_object_tracker::visualization::LaserObjectTrackerVisualization visualization(pnh, base_frame);
    ROS_INFO("Initializing subscriber");
    ros::Subscriber subscriber_laser_scan = pnh.subscribe("in_scan", 1, laserScanCallback);

    ros::Rate rate(10.0);
    ROS_INFO("Done initialization");
//    pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
    while (ros::ok())
    {
        ros::spinOnce();

        if (!fragment.empty())
        {
            visualization.clearMarkers();
            visualization.publishPointCloud(fragment);
            auto segments = segmentation->segment(fragment);
            ROS_INFO("Detected %lu segments", segments.size());
            visualization.publishPointClouds(segments);
            laser_object_tracker::feature_extraction::features::Corners2D corners_2_d;
            Eigen::VectorXd feature;
            for (const auto& segment : segments)
            {
                if (segment.isValid())
                {
                    if (detection.extractFeature(segment, feature))
                    {
                        corners_2_d.push_back(laser_object_tracker::feature_extraction::features::Corner2D(feature));
                    }
                }
            }
            visualization.publishCorners(corners_2_d);

            visualization.trigger();
        }
        else
        {
            ROS_WARN("Received laser scan is empty");
        }

        rate.sleep();
    }

    return 0;
}
