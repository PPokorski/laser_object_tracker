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

#ifndef LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_OBJECT_HPP
#define LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_OBJECT_HPP

#include "laser_object_tracker/data_types/laser_scan_fragment.hpp"
#include "laser_object_tracker/feature_extraction/features/corner_2d.hpp"
#include "laser_object_tracker/feature_extraction/features/point_2d.hpp"
#include "laser_object_tracker/feature_extraction/features/segment_2d.hpp"

namespace laser_object_tracker {
namespace feature_extraction {
namespace features {

class Object {
 public:
  Object() = default;

  Object(const data_types::LaserScanFragment& fragment,
         const Segments2D& segments,
         const Corners2D& corners);

  const ros::Time& getTimestamp() const {
    return timestamp_;
  }

  void setTimestamp(const ros::Time& timestamp) {
    timestamp_ = timestamp;
  }

  const Point2D& getReferencePoint() const {
    return reference_point_;
  }

  void setReferencePoint(const Point2D& reference_point) {
    reference_point_ = reference_point;
  }

  double getOrientation() const {
    return orientation_;
  }

  void setOrientation(double orientation) {
    orientation_ = orientation;
  }

  const Segments2D& getSegments() const {
    return segments_;
  }

  void setSegments(const Segments2D& segments) {
    segments_ = segments;
  }

  const Corners2D& getCorners() const {
    return corners_;
  }

  void setCorners(const Corners2D& corners) {
    corners_ = corners;
  }

  double getWidth() const {
    return width_;
  }

  double getHeight() const {
    return height_;
  }

  double getPerimeter() const {
    return perimeter_;
  }

  double getArea() const {
    return area_;
  }

  int getPointsNumber() const {
    return points_number_;
  }

  const Points2D& getPolyline() const {
    return polyline_;
  }

 private:
  void initializeOBBFromFragment(const data_types::LaserScanFragment& fragment);

  ros::Time timestamp_;

  Point2D reference_point_;
  double orientation_;

  Segments2D segments_;
  Corners2D corners_;

  double width_;
  double height_;

  double perimeter_;
  double area_;

  int points_number_;

  Points2D polyline_;
};

}  // namespace features
}  // namespace feature_extraction
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_FEATURE_EXTRACTION_FEATURES_OBJECT_HPP
