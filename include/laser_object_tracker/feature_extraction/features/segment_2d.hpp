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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_SEGMENT_2D_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_SEGMENT_2D_HPP

#include "point_2d.hpp"

namespace laser_object_tracker {
namespace feature_extraction {
namespace features {

class Segment2D {
 public:
  Segment2D() = default;

  Segment2D(const Eigen::Vector2d& start,
            const Eigen::Vector2d& end,
            bool is_start_occluded,
            bool is_end_occluded) :
      start_(start),
      end_(end),
      is_start_occluded_(is_start_occluded),
      is_end_occluded_(is_end_occluded),
      orientation_(segmentOrientation()) {}

  explicit Segment2D(const Eigen::VectorXd& coefficients) {
    if (coefficients.size() != 4) {
      throw std::invalid_argument("Segment2D needs 4 coefficients. " +
          std::to_string(coefficients.size()) + " provided.");
    }

    start_ = Point2D(coefficients.head<2>());
    end_ = Point2D(coefficients.tail<2>());
    orientation_ = segmentOrientation();
  }

  Line2D line() const {
    return Line2D::Through(start_, end_);
  }

  double length() const {
    return (end_ - start_).squaredNorm();
  }

  const Point2D& getStart() const {
    return start_;
  }

  void setStart(const Point2D& start) {
    start_ = start;
    orientation_ = segmentOrientation();
  }

  const Point2D& getEnd() const {
    return end_;
  }

  void setEnd(const Point2D& end) {
    end_ = end;
    orientation_ = segmentOrientation();
  }

  bool isStartOccluded() const {
    return is_start_occluded_;
  }

  void setIsStartOccluded(bool is_start_occluded) {
    is_start_occluded_ = is_start_occluded;
  }

  bool isEndOccluded() const {
    return is_end_occluded_;
  }

  void setIsEndOccluded(bool is_end_occluded) {
    is_end_occluded_ = is_end_occluded;
  }

  double getOrientation() const {
    return orientation_;
  }

 private:
  double segmentOrientation() const {
    return std::atan2(end_.y() - start_.y(), end_.x() - start_.x());
  }

  Point2D start_, end_;
  bool is_start_occluded_, is_end_occluded_;
  double orientation_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

using Segments2D = std::vector<Segment2D, Eigen::aligned_allocator<Segment2D>>;

inline double angleBetweenSegments(const Segment2D& lhs, const Segment2D& rhs) {
  return angleBetweenLines(lhs.line(), rhs.line());
}
}  // namespace features
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_SEGMENT_2D_HPP
