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

#include "laser_object_tracker/feature_extraction/features/object.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace laser_object_tracker {
namespace feature_extraction {
namespace features {

Object::Object(const data_types::LaserScanFragment& fragment,
               const Segments2D& segments,
               const Corners2D& corners)
    : points_number_(fragment.size()),
      segments_(segments),
      corners_(corners) {
  if (!corners.empty()) {
    reference_point_ = corners.front().getCorner();
    orientation_ = corners.front().getOrientation();
  } else if (!segments.empty()) {
    auto longest_segment = std::max_element(segments.begin(),
                                            segments.end(),
                                            [](const auto& lhs, const auto& rhs) {return lhs.length() < rhs.length();});

      orientation_ = longest_segment->getOrientation();
      if (!longest_segment->isStartOccluded()) {
        reference_point_ = longest_segment->getStart();
      } else if (!longest_segment->isEndOccluded()) {
        reference_point_ = longest_segment->getEnd();
      } else {
        reference_point_.setZero();
      }
  } else {
    throw std::invalid_argument("Object has no features - corners or segments");
  }
}

void Object::initializeOBBFromFragment(const data_types::LaserScanFragment& fragment) {
  if (fragment.empty()) {
    return;
  }

  std::vector<cv::Point2f> points(fragment.size());
  for (int i = 0; i < fragment.size(); ++i) {
    points.at(i).x = fragment.at(i).point().x;
    points.at(i).y = fragment.at(i).point().y;
  }
  auto rectangle = cv::minAreaRect(points);

  cv::Point2f obb_points[4];
  rectangle.points(obb_points);

  width_ = rectangle.size.width;
  height_ = rectangle.size.height;

  perimeter_ = 2 * width_ + 2 * height_;
  area_ = width_ * height_;

  polyline_.resize(4);
  for (int i = 0; i < polyline_.size(); ++i) {
    polyline_.at(i).x() = obb_points[i].x;
    polyline_.at(i).y() = obb_points[i].y;
  }
}
}  // namespace features
}  // namespace feature_extraction
}  // namespace laser_object_tracker