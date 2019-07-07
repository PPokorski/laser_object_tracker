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

#include "laser_object_tracker/feature_extraction/multi_line_detection.hpp"

#include <Eigen/StdVector>

#include <pcl/common/common.h>

namespace laser_object_tracker {
namespace feature_extraction {
MultiLineDetection::MultiLineDetection(double max_distance,
                                       double rho_resolution,
                                       double theta_resolution,
                                       int voting_threshold,
                                       double rho_min,
                                       double rho_max,
                                       double theta_min,
                                       double theta_max)
    : max_distance_(max_distance),
      rho_resolution_(rho_resolution),
      theta_resolution_(theta_resolution),
      voting_threshold_(voting_threshold),
      rho_min_(rho_min),
      rho_max_(rho_max),
      theta_min_(theta_min),
      theta_max_(theta_max),
      rho_min_defined_(std::isfinite(rho_min_)),
      rho_max_defined_(std::isfinite(rho_max_)) {}

bool MultiLineDetection::extractFeature(const data_types::LaserScanFragment& fragment, features::Feature& feature) {
  std::vector<cv::Point2f> cv_points = pointsFromFragment(fragment);

  Lines lines;
  bool finished = false;
  while (!finished) {
    initializeRhoLimits(cv_points);

    std::vector<cv::Vec3d> cv_lines;
    cv::HoughLinesPointSet(cv_points, cv_lines, 1, voting_threshold_,
                           rho_min_, rho_max_, rho_resolution_,
                           theta_min_, theta_max_, theta_resolution_);

    if (!cv_lines.empty()) {
      Line line;
      line.coeffs() << std::cos(cv_lines.at(0)(2)), std::sin(cv_lines.at(0)(2)), -cv_lines.at(0)(1);
      lines.push_back(line);

      // Remove inliers
      Eigen::Vector2d eigen_point;
      cv_points.erase(std::remove_if(cv_points.begin(),
                                     cv_points.end(),
                                     [this, &line, &eigen_point](const auto& point) {
                                       eigen_point << point.x, point.y;
                                       return this->isInlier(line, eigen_point);}),
                      cv_points.end());

      if (!rho_min_defined_) {
        rho_min_ = -std::numeric_limits<double>::infinity();
      }
      if (!rho_max_defined_) {
        rho_max_ = std::numeric_limits<double>::infinity();
      }

      finished = cv_points.empty();
    } else {
      finished = true;
    }
  }

  buildObservationVector(fragment, lines, feature.observation_);

  return !lines.empty();
}

std::vector<cv::Point2f> MultiLineDetection::pointsFromFragment(const data_types::LaserScanFragment& fragment) const {
  std::vector<cv::Point2f> cv_points(fragment.size());
  for (int i = 0; i < fragment.size(); ++i) {
    cv_points.at(i).x = fragment.at(i).point().x;
    cv_points.at(i).y = fragment.at(i).point().y;
  }

  return cv_points;
}

void MultiLineDetection::initializeRhoLimits(const std::vector<cv::Point2f>& points) {
  if (!rho_min_defined_ ||
      !rho_max_defined_) {
    double greatest_distance = 0.0;

    for (const auto& point : points) {
      double distance_to_zero = point.x * point.x + point.y * point.y;
      greatest_distance = std::max(greatest_distance, distance_to_zero);
    }

    if (!rho_min_defined_) {
      rho_min_ = -std::sqrt(greatest_distance);
    }
    if (!rho_max_defined_) {
      rho_max_ = std::sqrt(greatest_distance);
    }
  }

  if (rho_max_ < rho_min_) {
    throw std::logic_error("Rho max: " + std::to_string(rho_max_) +
                           " is smaller than rho min: " + std::to_string(rho_min_));
  }
}

bool MultiLineDetection::isInlier(const Line& line, const Eigen::Vector2d& point) const {
  return line.absDistance(point) <= this->max_distance_;
}

void MultiLineDetection::buildObservationVector(const data_types::LaserScanFragment& fragment,
                                                const Lines& lines,
                                                Eigen::VectorXd& observation) const {
  observation.resize(4 * lines.size());
  int i = 0;
  for (const auto& line : lines) {
    Eigen::Vector2d min_point = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity()),
        max_point = Eigen::Vector2d::Constant(-std::numeric_limits<double>::infinity());

    // The coefficient (x or y) by which we will be comparing points, is the one with the smallest
    // absolute value in the normal vector of the line
    Line::Index relevant_coeff;
    line.normal().array().abs().minCoeff(&relevant_coeff);

    Eigen::Vector2d eigen_point;
    for (const auto& point : fragment) {
      eigen_point << point.point().x, point.point().y;
      if (isInlier(line, eigen_point)) {
        eigen_point = line.projection(eigen_point);
        if (eigen_point(relevant_coeff) < min_point(relevant_coeff)) {
          min_point = eigen_point;
        }
        if (eigen_point(relevant_coeff) > max_point(relevant_coeff)) {
          max_point = eigen_point;
        }
      }
    }

    observation.segment<2>(4 * i) = min_point;
    observation.segment<2>(4 * i + 2) = max_point;
    ++i;
  }
}
}  // namespace feature_extraction
}  // namespace laser_object_tracker