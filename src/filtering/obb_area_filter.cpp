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

#include "laser_object_tracker/filtering/obb_area_filter.hpp"

#include <opencv2/imgproc/imgproc.hpp>

namespace laser_object_tracker {
namespace filtering {
OBBAreaFilter::OBBAreaFilter(double min_area, double max_area, double min_box_dimension)
    : min_area_(min_area), max_area_(max_area), min_box_dimension_(min_box_dimension) {}

bool OBBAreaFilter::shouldFilter(const data_types::LaserScanFragment& fragment) const {
  double area = getOBBArea(fragment);
  return area < min_area_ || area > max_area_;
}

double OBBAreaFilter::getMinArea() const {
  return min_area_;
}

void OBBAreaFilter::setMinArea(double min_area) {
  min_area_ = min_area;
}

double OBBAreaFilter::getMaxArea() const {
  return max_area_;
}

void OBBAreaFilter::setMaxArea(double max_area) {
  max_area_ = max_area;
}

double OBBAreaFilter::getMinBoxDimension() const {
  return min_box_dimension_;
}

void OBBAreaFilter::setMinBoxDimension(double min_box_dimension) {
  min_box_dimension_ = min_box_dimension;
}

double OBBAreaFilter::getOBBArea(const data_types::LaserScanFragment& fragment) const {
  if (fragment.empty()) {
    return 0.0;
  }
  std::vector<cv::Point2f> points(fragment.size());
  for (int i = 0; i < fragment.size(); ++i) {
    points.at(i).x = fragment.at(i).point().x;
    points.at(i).y = fragment.at(i).point().y;
  }
  auto rectangle = cv::minAreaRect(points);

  rectangle.size.height = std::max(static_cast<float>(min_box_dimension_), rectangle.size.height);
  rectangle.size.width = std::max(static_cast<float>(min_box_dimension_), rectangle.size.width);

  return rectangle.size.area();
}
}  // namespace filtering
}  // namespace laser_object_tracker
