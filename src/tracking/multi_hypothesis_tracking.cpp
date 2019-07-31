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
MultiHypothesisTracking::MultiHypothesisTracking(double position_variance_x,
                                                 double position_variance_y,
                                                 double gradient_variance,
                                                 double intensity_variance,
                                                 double process_variance,
                                                 double mean_new,
                                                 double probability_end,
                                                 double probability_detection,
                                                 double state_variance,
                                                 double max_distance,
                                                 double mean_false_alarms,
                                                 int max_depth,
                                                 double min_g_hypothesis_ratio,
                                                 int max_g_hypothesis)
    : position_variance_x_(position_variance_x),
      position_variance_y_(position_variance_y),
      gradient_variance_(gradient_variance),
      intensity_variance_(intensity_variance),
      process_variance_(process_variance),
      mean_new_(mean_new),
      probability_end_(probability_end),
      probability_detection_(probability_detection),
      state_variance_(state_variance),
      max_distance_(max_distance),
      mean_false_alarms_(mean_false_alarms),
      max_depth_(max_depth),
      min_g_hypothesis_ratio_(min_g_hypothesis_ratio),
      max_g_hypothesis_(max_g_hypothesis) {
  auto* const_velocity_model = new CONSTVEL_MDL(
      position_variance_x_,
      position_variance_y_,
      gradient_variance_,
      intensity_variance_,
      process_variance_,
      mean_new_,
      probability_end_,
      probability_detection_,
      state_variance_,
      max_distance_);
  models_.append(*const_velocity_model);

  multi_hypothesis_tracking_ = std::make_unique<CORNER_TRACK_MHT>(mean_false_alarms_,
                                                                  max_depth_,
                                                                  min_g_hypothesis_ratio_,
                                                                  max_g_hypothesis_,
                                                                  models_);
}

void MultiHypothesisTracking::predict() {
}

void MultiHypothesisTracking::update(const std::vector<feature_extraction::features::Feature>& measurements) {
  std::cout << "******************CURRENT_TIME=" << multi_hypothesis_tracking_->getCurrentTime() << std::endl;
  CORNERLIST reports(measurements.size(), 0.1);

  for (const auto& measurement : measurements) {
    reports.list.emplace_back(measurement.observation_(0),
                              measurement.observation_(1),
                              frame_number_,
                              corner_id_++);
  }

  multi_hypothesis_tracking_->addReports(reports);
  multi_hypothesis_tracking_->scan();
  mht::internal::g_time = multi_hypothesis_tracking_->getCurrentTime();
//  multi_hypothesis_tracking_->printStats(2);

  ++frame_number_;

//  for (const auto& track : multi_hypothesis_tracking_->GetTracks()) {
//    std::cout << "ID: " << track.id << std::endl;
//    for (const auto& point : track.list) {
//      std::cout << "r: [" << point.rx << ", " << point.ry << "] " <<
//                   "s: [" << point.sx << ", " << point.sy << "]" << std::endl;
//    }

//  }
}

MultiHypothesisTracking::~MultiHypothesisTracking() {
  multi_hypothesis_tracking_->clear();
}
}  // namespace tracking
}  // namespace laser_object_tracker