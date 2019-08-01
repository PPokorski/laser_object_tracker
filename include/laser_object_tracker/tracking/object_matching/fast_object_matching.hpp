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

#ifndef LASER_OBJECT_TRACKER_TRACKING_OBJECT_MATCHING_FAST_OBJECT_MATCHING_HPP
#define LASER_OBJECT_TRACKER_TRACKING_OBJECT_MATCHING_FAST_OBJECT_MATCHING_HPP

#include <queue>
#include <utility>
#include <vector>

#include <ros/time.h>

#include "laser_object_tracker/feature_extraction/features/object.hpp"

namespace laser_object_tracker {
namespace tracking {
namespace object_matching {

class FastObjectMatching {
 public:
  FastObjectMatching(double buffer_length,
                     double distance_threshold,
                     double orientation_angle_threshold,
                     double aperture_angle_threshold);

  bool isMatched(const feature_extraction::features::Object& object) const;

  void buffer(const std::vector<feature_extraction::features::Object>& objects);

  void popOutdated(const ros::Time& current_time);

 private:
  using BufferElement = std::pair<std::vector<feature_extraction::features::Object>,
                                  ros::Time>;

  bool cornersMatched(const feature_extraction::features::Object& lhs,
                      const feature_extraction::features::Object& rhs) const;

  bool cornersMatched(const feature_extraction::features::Corner2D& lhs,
                      const feature_extraction::features::Corner2D& rhs) const;

  bool segmentsMatched(const feature_extraction::features::Object& lhs,
                       const feature_extraction::features::Object& rhs) const;

  bool segmentsMatched(const feature_extraction::features::Segment2D& lhs,
                       const feature_extraction::features::Segment2D& rhs) const;

  double angleBetweenAngles(double target, double source) const;
  ros::Duration buffer_length_;

  double distance_threshold_;
  double orientation_angle_threshold_;
  double aperture_angle_threshold_;

  std::queue<BufferElement> buffer_;
};

}  // namespace object_matching
}  // namespace tracking
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_TRACKING_OBJECT_MATCHING_FAST_OBJECT_MATCHING_HPP
