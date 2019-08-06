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

#include "laser_object_tracker/feature_extraction/features/features.hpp"
#include "laser_object_tracker/tracking/tracking.hpp"

using namespace laser_object_tracker;

int main(int ac, char** av) {
  tracking::mht::ObjectState::MeasurementNoiseCovariance measurement_noise_covariance;
  measurement_noise_covariance << 0.1, 0.0,
      0.0, 0.1;

  tracking::mht::ObjectState::InitialStateCovariance initial_state_covariance;
  initial_state_covariance << 0.1, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 0.0, 0.1;

  tracking::mht::ObjectState::ProcessNoiseCovariance process_noise_covariance;
  process_noise_covariance << 0.2, 0.0, 0.0, 0.0,
      0.0, 0.2, 0.0, 0.0,
      0.0, 0.0, 0.2, 0.0,
      0.0, 0.0, 0.0, 0.2;

  laser_object_tracker::tracking::MultiHypothesisTracking multi_tracker(0.1,
                                                                        1.0,
                                                                        20.0,
                                                                        0.01,
                                                                        0.999,
                                                                        measurement_noise_covariance,
                                                                        initial_state_covariance,
                                                                        process_noise_covariance,
                                                                        0.00002,
                                                                        1,
                                                                        0.001,
                                                                        100);

  std::vector<laser_object_tracker::feature_extraction::features::Feature> measurements;
  laser_object_tracker::feature_extraction::features::Feature feature;
  feature.observation_.resize(2);
  feature.observation_ << 1.0, 1.0;

  measurements.push_back(feature);

  for (int i = 0; i < 10; ++i) {
//    multi_tracker.update(measurements);

    measurements.back().observation_ << 1.0 + i * 0.1, 1.0 + i * 0.1;
  }
  return 0;
}