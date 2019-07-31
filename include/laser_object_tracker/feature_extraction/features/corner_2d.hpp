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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_CORNER_2_D_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_CORNER_2_D_HPP

#include "laser_object_tracker/feature_extraction/features/line_2d.hpp"
#include "point_2d.hpp"
#include "laser_object_tracker/feature_extraction/features/segment_2d.hpp"

namespace laser_object_tracker {
namespace feature_extraction {
namespace features {

class Corner2D {
 public:
  Corner2D() = default;

  Corner2D(const Point2D& corner, double orientation, double aperture)
      : corner_(corner),
        orientation_(orientation),
        aperture_(aperture) {}

  Corner2D(const Line2D& line_1, const Line2D& line_2, double orientation)
      : corner_(line_1.intersection(line_2)),
        orientation_(orientation),
        aperture_(angleBetweenLines(line_1, line_2)) {
    if (aperture_ < SMALL_ANGLE) {
      std::stringstream ss;
      ss << "The angle between lines is to small. Line 1: " << line_1.coeffs().transpose()
         << " Line 2: " << line_2.coeffs().transpose() << ". The angle between them: " << aperture_;
      throw std::invalid_argument(ss.str());
    }
  }

  Corner2D(const Segment2D& segment_1, const Segment2D& segment_2)
      : Corner2D(segment_1.line(),
                 segment_2.line(),
                 // Orientation is the orientation of the longer segment or the first one
                 segment_1.length() >= segment_2.length() ?
                      segment_1.getOrientation() :
                      segment_2.getOrientation()) {}

  const Point2D& getCorner() const {
    return corner_;
  }

  void setCorner(const Point2D& corner) {
    corner_ = corner;
  }

  double getOrientation() const {
    return orientation_;
  }

  void setOrientation(double orientation) {
    orientation_ = orientation;
  }

  double getAperture() const {
    return aperture_;
  }

  void setAperture(double aperture) {
    aperture_ = aperture;
  }

 private:
  static constexpr double SMALL_ANGLE = 0.02; // 1 degree

  Point2D corner_;
  double orientation_;
  // The angle between segments creating a corner
  double aperture_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

using Corners2D = std::vector<Corner2D, Eigen::aligned_allocator<Corner2D>>;

Corners2D findIntersections(const Segments2D& segments, double min_angle);

}  // namespace features
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_CORNER_2_D_HPP
