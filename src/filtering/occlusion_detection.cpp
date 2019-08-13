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

#include "laser_object_tracker/filtering/occlusion_detection.hpp"

namespace laser_object_tracker {
namespace filtering {
OcclusionDetection::OcclusionDetection(double max_angle_gap)
    : max_angle_gap_(max_angle_gap) {}

bool OcclusionDetection::shouldFilter(const laser_object_tracker::data_types::LaserScanFragment& fragment) const {
  return false;
}

void OcclusionDetection::filter(std::vector<data_types::LaserScanFragment>& fragments) const {
  if (fragments.empty()) {
    return;
  }
  fragments.front().front().isOccluded() = true;

  for (int i = 1; i < fragments.size(); ++i) {
    auto& current_element = fragments.at(i).front();
    auto& previous_element = fragments.at(i - 1).back();
    if (current_element.getAngle() - previous_element.getAngle() > max_angle_gap_) {
      continue;
    }

    if (current_element.range() < previous_element.range()) {
      previous_element.isOccluded() = true;
    }

    if (current_element.range() > previous_element.range()) {
      current_element.isOccluded() = true;
    }
  }

  fragments.back().back().isOccluded() = true;
}
}  // namespace filtering
}  // namespace laser_object_tracker
