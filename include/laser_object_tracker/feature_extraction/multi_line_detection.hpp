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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_MULTI_LINE_DETECTION_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_MULTI_LINE_DETECTION_HPP

#include <opencv2/imgproc.hpp>

#include "laser_object_tracker/feature_extraction/base_feature_extraction.hpp"
#include "laser_object_tracker/feature_extraction/features/object.hpp"

namespace laser_object_tracker {
namespace feature_extraction {
class MultiLineDetection : public BaseFeatureExtraction<features::Object> {
 public:

  MultiLineDetection(OcclusionChecking occlusion_checking,
                     double min_angle_between_lines,
                     double max_distance,
                     double rho_resolution,
                     double theta_resolution,
                     int voting_threshold,
                     double rho_min = -std::numeric_limits<double>::infinity(),
                     double rho_max = std::numeric_limits<double>::infinity(),
                     double theta_min = 0.0,
                     double theta_max = M_PI);

  bool extractFeature(const data_types::LaserScanFragment& fragment, FeatureT& feature) override;

 protected:
  using Line = Eigen::Hyperplane<double, 2>;
  using Lines = std::vector<Line, Eigen::aligned_allocator<Line>>;

  std::vector<cv::Point2f> pointsFromFragment(const data_types::LaserScanFragment& fragment) const;
  cv::Point2f demeanPoints(std::vector<cv::Point2f>& points) const;
  void initializeRhoLimits(const std::vector<cv::Point2f>& points);
  bool isInlier(const Line& line, const Eigen::Vector2d& point) const;
  features::Segments2D segmentsFromLines(const data_types::LaserScanFragment& fragment, const Lines& lines) const;

  double min_angle_between_lines_;
  double max_distance_;
  double rho_resolution_, theta_resolution_;
  int voting_threshold_;
  double rho_min_, rho_max_;
  double theta_min_, theta_max_;
  bool rho_min_defined_, rho_max_defined_;
};
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_MULTI_LINE_DETECTION_HPP
