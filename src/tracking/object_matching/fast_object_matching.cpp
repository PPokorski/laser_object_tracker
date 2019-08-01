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

#include "laser_object_tracker/tracking/object_matching/fast_object_matching.hpp"

namespace laser_object_tracker {
namespace tracking {
namespace object_matching {

FastObjectMatching::FastObjectMatching(double buffer_length,
                                       double distance_threshold,
                                       double orientation_angle_threshold,
                                       double aperture_angle_threshold)
    : buffer_length_(buffer_length),
      distance_threshold_(distance_threshold),
      orientation_angle_threshold_(orientation_angle_threshold),
      aperture_angle_threshold_(aperture_angle_threshold) {}

bool FastObjectMatching::isMatched(const feature_extraction::features::Object& object) const {
  if (buffer_.empty()) {
    return false;
  }

  const auto& last_objects = buffer_.front().first;
  return std::any_of(last_objects.begin(),
                     last_objects.end(),
                     [&object, this] (const auto& current_object) {
    if (object.getCorners().empty() || current_object.getCorners().empty()) {
      return segmentsMatched(object, current_object);
    } else {
      return cornersMatched(object, current_object);
    }
  });
}

void FastObjectMatching::buffer(const std::vector<feature_extraction::features::Object>& objects) {
  if (objects.empty()) {
    return;
  }

  buffer_.emplace(objects, objects.front().getTimestamp());
}

void FastObjectMatching::popOutdated(const ros::Time& current_time) {
  if (buffer_.empty()) {
    return;
  }

  while (current_time - buffer_.front().second > buffer_length_) {
    buffer_.pop();
  }
}

bool FastObjectMatching::cornersMatched(const feature_extraction::features::Object& lhs,
                                        const feature_extraction::features::Object& rhs) const {
  for (const auto& lhs_corner : lhs.getCorners()) {
    for (const auto& rhs_corner : rhs.getCorners()) {
      if (cornersMatched(lhs_corner, rhs_corner)) {
        return true;
      }
    }
  }

  return false;
}

bool FastObjectMatching::cornersMatched(const feature_extraction::features::Corner2D& lhs,
                                        const feature_extraction::features::Corner2D& rhs) const {
  return (lhs.getCorner() - rhs.getCorner()).squaredNorm() < distance_threshold_ * distance_threshold_ &&
         (angleBetweenAngles(lhs.getOrientation(), rhs.getOrientation()) < orientation_angle_threshold_ ||
          std::abs(lhs.getAperture() - rhs.getAperture()) < aperture_angle_threshold_);
}

bool FastObjectMatching::segmentsMatched(const feature_extraction::features::Object& lhs,
                                         const feature_extraction::features::Object& rhs) const {
  for (const auto& lhs_segment : lhs.getSegments()) {
    for (const auto& rhs_segment : rhs.getSegments()) {
      if (segmentsMatched(lhs_segment, rhs_segment)) {
        return true;
      }
    }
  }

  return false;
}

bool FastObjectMatching::segmentsMatched(const feature_extraction::features::Segment2D& lhs,
                                         const feature_extraction::features::Segment2D& rhs) const {
  return angleBetweenAngles(lhs.getOrientation(), rhs.getOrientation()) < orientation_angle_threshold_ &&
         ((lhs.getStart() - rhs.getStart()).squaredNorm() < distance_threshold_ * distance_threshold_ ||
          (lhs.getEnd() - rhs.getEnd()).squaredNorm() < distance_threshold_ * distance_threshold_);
}

double FastObjectMatching::angleBetweenAngles(double target, double source) const {
  double delta = target - source;
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  };
  if (delta < -M_PI) {
    delta += 2 * M_PI;
  }

  return std::abs(delta);
}
}  // namespace object_matching
}  // namespace tracking
}  // namespace laser_object_tracker
