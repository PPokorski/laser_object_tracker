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

#include "laser_object_tracker/tracking/iteration_tracker_rejection.hpp"

namespace laser_object_tracker {
namespace tracking {

IterationTrackerRejection::IterationTrackerRejection(int max_iterations_without_update)
    : max_iterations_without_update_(max_iterations_without_update),
      iterations_without_update_(0) {}

bool IterationTrackerRejection::invalidate(const BaseTracking& tracker) const {
  return iterations_without_update_ > max_iterations_without_update_;
}

void IterationTrackerRejection::updated(const BaseTracking& tracker) {
  iterations_without_update_ = 0;
}

void IterationTrackerRejection::notUpdated(const BaseTracking& tracker) {
  ++iterations_without_update_;
}

std::unique_ptr<BaseTrackerRejection> IterationTrackerRejection::clone() const {
  return std::unique_ptr<BaseTrackerRejection>(new IterationTrackerRejection(*this));
}

int IterationTrackerRejection::getMaxIterationsWithoutUpdate() const {
  return max_iterations_without_update_;
}

void IterationTrackerRejection::setMaxIterationsWithoutUpdate(int max_iterations_without_update) {
  max_iterations_without_update_ = max_iterations_without_update;
}
}  // namespace tracking
}  // namespace laser_object_tracker
