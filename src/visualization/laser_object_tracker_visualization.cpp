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

#include "laser_object_tracker/visualization/laser_object_tracker_visualization.hpp"

#include <random>

namespace laser_object_tracker {
namespace visualization {

void LaserObjectTrackerVisualization::publishPointClouds(const std::vector<data_types::LaserScanFragment>& fragments) {
  pcl::PointCloud<pcl::PointXYZRGB> pcl;
  if (!fragments.empty()) {
    pcl.header = fragments.front().pointCloud().header;
  }
  expandToNColors(fragments.size());

  for (int i = 0; i < fragments.size(); ++i) {
    const auto& fragment = fragments.at(i);
    const auto& color = colours_.at(i);

    pcl::PointCloud<pcl::PointXYZRGB> tmp;
    pcl::copyPointCloud(fragment.pointCloud(), tmp);
    for (auto& point : tmp.points) {
      point.rgb = color;
    }

    pcl += tmp;
  }

  pub_point_clouds_.publish(pcl);
}

void LaserObjectTrackerVisualization::expandToNColors(int colors) {
  // Taken from
  // http://docs.pointclouds.org/1.9.1/structpcl_1_1_point_x_y_z_r_g_b.html#ab8cae6380d0d2c30c63ac92053aa2e82
  if (colors < colours_.size()) {
    return;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint8_t> dist;

  while (colors > colours_.size()) {
    uint8_t r = dist(gen),
        g = dist(gen),
        b = dist(gen);
    uint32_t rgb = static_cast<uint32_t>(r) << 16 |
        static_cast<uint32_t>(g) << 8 |
        static_cast<uint32_t>(b);

    colours_.push_back(*reinterpret_cast<float *>(&rgb));

    std_msgs::ColorRGBA color;
    color.r = r / 255.0;
    color.g = g / 255.0;
    color.b = b / 255.0;
    color.a = 1.0;
    rgb_colors_.push_back(color);
  }
}

void LaserObjectTrackerVisualization::publishSegment(const feature_extraction::features::Segment2D& segment,
                                                     const std_msgs::ColorRGBA& color) {
  Eigen::Vector3d point_1, point_2;
  point_1.head<2>() = segment.start_;
  point_2.head<2>() = segment.end_;

  rviz_visual_tools_->publishLine(point_1, point_2, color);
}

void LaserObjectTrackerVisualization::publishSegments(const feature_extraction::features::Segments2D& segments) {
  expandToNColors(segments.size());

  for (int i = 0; i < segments.size(); ++i) {
    publishSegment(segments.at(i), rgb_colors_.at(i));
  }
}

void LaserObjectTrackerVisualization::publishCorner(const feature_extraction::features::Corner2D& corner,
                                                    const std_msgs::ColorRGBA& color) {
  publishSegment({corner.corner_, corner.point_1_}, color);
  publishSegment({corner.corner_, corner.point_2_}, color);
}

void LaserObjectTrackerVisualization::publishPoint(const feature_extraction::features::Point2D& point,
                                                   const std_msgs::ColorRGBA& color) {
    Eigen::Vector3d publish_point;
    publish_point.head<2>() = point.point_;

    rviz_visual_tools_->publishSphere(publish_point, color,
            rviz_visual_tools_->getScale(rviz_visual_tools::scales::XLARGE));
}

void LaserObjectTrackerVisualization::publishCorners(const feature_extraction::features::Corners2D& corners) {
  expandToNColors(corners.size());

  for (int i = 0; i < corners.size(); ++i) {
    publishCorner(corners.at(i), rgb_colors_.at(i));
  }
}
}  // namespace visualization
}  // namespace laser_object_tracker
