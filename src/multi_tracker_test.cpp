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

std::unique_ptr<laser_object_tracker::tracking::BaseTracking> getTracker() {
  Eigen::MatrixXd process_noise_covariance(6, 6);
  process_noise_covariance << 0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.1;

  Eigen::MatrixXd measurement_noise_covariance(3, 3);
  measurement_noise_covariance << 0.01, 0.00, 0.00,
      0.00, 0.01, 0.00,
      0.00, 0.00, 0.01;

  Eigen::MatrixXd initial_state_covariance(6, 6);
  initial_state_covariance << 0.3, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.3, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.3, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  return std::make_unique<laser_object_tracker::tracking::CornerTracker>(0.1, 0.1,
                                                                         measurement_noise_covariance,
                                                                         initial_state_covariance,
                                                                         process_noise_covariance);
}

#include <iostream>

void printTracker(const laser_object_tracker::tracking::BaseTracking& tracker) {
  std::cout << "~~!! TRACKERS !!~~" << std::endl;
  std::cout << tracker.getStateVector() << std::endl;
}

static constexpr int STEPS = 5;

int main(int ac, char **av) {
  auto tracker = getTracker();

  laser_object_tracker::feature_extraction::features::Feature feature;
  feature.observation_.resize(6);
  feature.observation_ << 1.0, 1.0,
                          2.0, 2.0,
                          1.0, 1.0;
  feature.vector_bool_ = {false, false, false};
  std::cout << "~~~~!!!! INITIALIZATION STEP !!!!~~~~" << std::endl;
  tracker->initFromMeasurement(feature);
  printTracker(*tracker);

  feature.observation_ << 1.5, 1.0,
                          2.5, 2.0,
                          1.5, 1.0;
  std::cout << "~~~~!!!! PREDICTION STEP !!!!~~~~" << std::endl;
  tracker->predict();
  printTracker(*tracker);
  std::cout << "~~~~!!!! UPDATE STEP !!!!~~~~" << std::endl;
  tracker->update(feature);
  printTracker(*tracker);

  feature.observation_ << 2.0, 1.0,
                          3.0, 2.0,
                          2.0, 1.0;
  feature.vector_bool_= {false, true, false};
  std::cout << "~~~~!!!! PREDICTION STEP !!!!~~~~" << std::endl;
  tracker->predict();
  printTracker(*tracker);
  std::cout << "~~~~!!!! UPDATE STEP !!!!~~~~" << std::endl;
  tracker->update(feature);
  printTracker(*tracker);


//  for(int i = 0; i < STEPS; ++i) {
//    std::cout << "~~~~!!!! PREDICTION STEP !!!!~~~~" << std::endl;
//    tracker->predict();
//    printTracker(*tracker);
//    std::cout << "~~~~!!!! UPDATE STEP !!!!~~~~" << std::endl;
//    tracker->update(feature);
//    printTracker(*tracker);
//  }
}
