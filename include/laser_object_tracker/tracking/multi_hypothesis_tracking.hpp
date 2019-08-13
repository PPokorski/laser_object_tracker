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

#ifndef LASER_OBJECT_TRACKER_TRACKING_MULTI_HYPOTHESIS_TRACKING_HPP
#define LASER_OBJECT_TRACKER_TRACKING_MULTI_HYPOTHESIS_TRACKING_HPP

#include <memory>

#include "laser_object_tracker/tracking/base_multi_tracking.hpp"
#include "laser_object_tracker/tracking/object_matching/fast_object_matching.hpp"

#include "laser_object_tracker/tracking/mht/object_model.hpp"

namespace laser_object_tracker {
namespace tracking {
class MultiHypothesisTracking : public BaseMultiTracking<feature_extraction::features::Object> {
 public:
  MultiHypothesisTracking(object_matching::FastObjectMatching object_matching,
                          tracking::mht::ObjectModel* model,
                          double false_alarm_likelihood,
                          int max_depth,
                          double min_g_hypothesis_ratio,
                          int max_g_hypothesis);

  void predict() override;

  void update(const std::vector<FeatureT>& measurements) override;

  ~MultiHypothesisTracking();

 private:
  long frame_number_ = 0;
  long corner_id_ = 0;

  double false_alarm_log_likelihood_;
  
  object_matching::FastObjectMatching fast_object_matching_;

  ptrDLIST_OF<MODEL> models_;
  std::unique_ptr<mht::ObjectTracker> multi_hypothesis_tracking_;
};
}  // namespace tracking
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_TRACKING_MULTI_HYPOTHESIS_TRACKING_HPP