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

#include <laser_object_tracker/segmentation/adaptive_breakpoint_detection.hpp>

#include "laser_object_tracker/segmentation/adaptive_breakpoint_detection.hpp"

#include "laser_object_tracker/segmentation/distance_calculation.hpp"

namespace laser_object_tracker {
namespace segmentation {

AdaptiveBreakpointDetection::AdaptiveBreakpointDetection(double incidence_angle, double distance_resolution) :
        BaseSegmentation(),
        incidence_angle_(incidence_angle),
        distance_resolution_(distance_resolution) {}

std::vector<data_types::LaserScanFragment>
AdaptiveBreakpointDetection::segment(const data_types::LaserScanFragment& fragment) {
    if (fragment.empty())
    {
        return {};
    }

    auto current_begin = fragment.cbegin();
    auto previous = fragment.cbegin();
    auto current = fragment.cbegin();

    std::vector<data_types::LaserScanFragment> segments;
    while (current != fragment.cend())
    {
        previous = current++;

        if (!current_begin->isValid())
        {
            current_begin = current;
        }
        else if (current == fragment.cend() ||
                !current->isValid() ||
                isAboveThreshold(previous->range(), current->range(),
                    calculateThreshold(previous->range(), fragment.getAngleIncrement())))
        {

            segments.emplace_back(fragment,
                                  current_begin - fragment.cbegin(),
                                  current - fragment.cbegin());

            current_begin = current;
        }
    }
    return segments;
}

double AdaptiveBreakpointDetection::getIncidenceAngle() const {
    return incidence_angle_;
}

void AdaptiveBreakpointDetection::setIncidenceAngle(double incidence_angle) {
    incidence_angle_ = incidence_angle;
}

double AdaptiveBreakpointDetection::getDistanceResolution() const {
    return distance_resolution_;
}

void AdaptiveBreakpointDetection::setDistanceResolution(double distance_resolution) {
    distance_resolution_ = distance_resolution;
}

bool AdaptiveBreakpointDetection::isAboveThreshold(double previous_range, double current_range, double threshold) {
    return distance(previous_range, current_range) > threshold;
}

double AdaptiveBreakpointDetection::calculateThreshold(double previous_range, double angle_increment) {
    return previous_range * (std::sin(angle_increment) /
                             std::sin(incidence_angle_ - angle_increment)) +
                             3 * distance_resolution_;
}

}  // namespace segmentation
}  // namespace laser_object_tracker
