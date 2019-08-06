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
MultiHypothesisTracking::MultiHypothesisTracking(double time_step,
                                                 double max_mahalanobis_distance,
                                                 double skip_decay_rate,
                                                 double probability_start,
                                                 double probability_detection,
                                                 const mht::ObjectState::MeasurementNoiseCovariance& measurement_noise_covariance,
                                                 const mht::ObjectState::InitialStateCovariance& initial_state_covariance,
                                                 const mht::ObjectState::ProcessNoiseCovariance& process_noise_covariance,
                                                 double mean_false_alarms,
                                                 int max_depth,
                                                 double min_g_hypothesis_ratio,
                                                 int max_g_hypothesis)
    : false_alarm_log_likelihood_(std::log(mean_false_alarms)) {
  auto* const_velocity_model = new mht::ObjectModel(
      time_step,
      max_mahalanobis_distance,
      skip_decay_rate,
      probability_start,
      probability_detection,
      measurement_noise_covariance,
      initial_state_covariance,
      process_noise_covariance);
  models_.append(*const_velocity_model);

  multi_hypothesis_tracking_ = std::make_unique<mht::ObjectTracker>(mean_false_alarms,
                                                                    max_depth,
                                                                    min_g_hypothesis_ratio,
                                                                    max_g_hypothesis,
                                                                    models_);
}

void MultiHypothesisTracking::predict() {
}

void MultiHypothesisTracking::update(const std::vector<FeatureT>& measurements) {
  std::list<REPORT*> reports;

  for (const auto& measurement : measurements) {
    reports.push_back(new mht::ObjectReport(false_alarm_log_likelihood_,
                                            measurement,
                                            frame_number_,
                                            corner_id_++));
  }

  multi_hypothesis_tracking_->addReports(reports);
  multi_hypothesis_tracking_->scan();

  ++frame_number_;
}

MultiHypothesisTracking::~MultiHypothesisTracking() {
  multi_hypothesis_tracking_->clear();
}
}  // namespace tracking
}  // namespace laser_object_tracker