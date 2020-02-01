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

#include "laser_object_tracker/tracking/multi_hypothesis_tracking.hpp"

namespace laser_object_tracker {
namespace tracking {
MultiHypothesisTracking::MultiHypothesisTracking(object_matching::FastObjectMatching object_matching,
                                                 tracking::mht::ObjectModel* model,
                                                 double min_velocity,
                                                 double false_alarm_likelihood,
                                                 int max_depth,
                                                 double min_g_hypothesis_ratio,
                                                 int max_g_hypothesis)
    : min_velocity_(min_velocity),
      false_alarm_log_likelihood_(std::log(false_alarm_likelihood)),
      fast_object_matching_(std::move(object_matching)) {
  models_.append(*model);

  multi_hypothesis_tracking_ = std::make_unique<mht::ObjectTracker>(false_alarm_likelihood,
                                                                    max_depth,
                                                                    min_g_hypothesis_ratio,
                                                                    max_g_hypothesis,
                                                                    models_);
}

void MultiHypothesisTracking::predict() {
}

const MultiHypothesisTracking::Container& MultiHypothesisTracking::update(const std::vector<FeatureT>& measurements) {
  std::list<REPORT*> reports;

  if (!measurements.empty()) {
    if (!fast_object_matching_.isReady(measurements.front().getTimestamp())) {
      fast_object_matching_.buffer(measurements);
      fast_object_matching_.popOutdated(measurements.front().getTimestamp());
      return tracks_;
    }

    for (const auto& measurement : measurements) {
      if (fast_object_matching_.isMatched(measurement)) {
        continue;
      }
      reports.push_back(new mht::ObjectReport(false_alarm_log_likelihood_,
                                              measurement,
                                              frame_number_,
                                              corner_id_++));
    }
  }

  multi_hypothesis_tracking_->addReports(reports);
  multi_hypothesis_tracking_->scan();

  tracks_.clear();
  tracks_.reserve(multi_hypothesis_tracking_->getTracks().size());
  for (const auto& track : multi_hypothesis_tracking_->getTracks()) {
    tracks_.emplace_back(track.id_);
    tracks_.back().track_.reserve(track.track_.size());

    for (const auto& track_element : track.track_) {
      if (track_element.is_confirmed_) {
        tracks_.back().track_.push_back({
          track_element.likelihood_,
          track_element.timestamp_,
          track_element.was_updated_,
          track_element.position_,
          track_element.position_covariance_,
          track_element.velocity_,
          track_element.velocity_covariance_,
          track_element.polyline_
        });
      }
    }

    if (tracks_.back().track_.empty()) {
      tracks_.erase(std::prev(tracks_.end()));
    }
  }

  ++frame_number_;

  if (!measurements.empty()) {
    fast_object_matching_.buffer(measurements);
    fast_object_matching_.popOutdated(measurements.front().getTimestamp());
  }

  tracks_.erase(std::remove_if(tracks_.begin(),
                               tracks_.end(),
                               [min_v = min_velocity_](const auto& track) {
                                 return track.track_.back().velocity_.squaredNorm() < min_v * min_v;}),
                tracks_.end());

  return tracks_;
}

MultiHypothesisTracking::~MultiHypothesisTracking() {
  multi_hypothesis_tracking_->clear();
}
}  // namespace tracking
}  // namespace laser_object_tracker