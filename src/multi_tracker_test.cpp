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

#include "laser_object_tracker/data_association/data_association.hpp"
#include "laser_object_tracker/tracking/tracking.hpp"

std::unique_ptr<laser_object_tracker::tracking::BaseTracking> getTracker() {
  Eigen::MatrixXd transition(4, 4);
  transition << 1.0, 0.0, 0.1, 0.0,
      0.0, 1.0, 0.0, 0.1,
      0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  Eigen::MatrixXd measurement(2, 4);
  measurement << 1.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0;

  Eigen::MatrixXd process_noise_covariance(4, 4);
  process_noise_covariance << 0.1, 0.0, 0.0, 0.0,
      0.0, 0.1, 0.0, 0.0,
      0.0, 0.0, 0.1, 0.0,
      0.0, 0.0, 0.0, 0.1;

  Eigen::MatrixXd measurement_noise_covariance(2, 2);
  measurement_noise_covariance << 0.1, 0.0,
      0.0, 0.1;

  Eigen::MatrixXd initial_state_covariance(4, 4);
  initial_state_covariance << 1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0,
      1.0, 1.0, 1.0, 1.0;

  return std::make_unique<laser_object_tracker::tracking::KalmanFilter>(4, 2,
                                                                        transition,
                                                                        measurement,
                                                                        measurement_noise_covariance,
                                                                        initial_state_covariance,
                                                                        process_noise_covariance);
}

std::unique_ptr<laser_object_tracker::data_association::BaseDataAssociation> getDataASsociation() {
  return std::make_unique<laser_object_tracker::data_association::HungarianAlgorithm>();
}

laser_object_tracker::tracking::MultiTracker::DistanceFunctor getDistanceFunctor() {
  return [](const Eigen::VectorXd& observation, const laser_object_tracker::tracking::BaseTracking& tracker) {
    return (observation - tracker.getStateVector().head<2>()).squaredNorm();
  };
}

#include <iostream>

void printTracker(const laser_object_tracker::tracking::MultiTracker& multi_tracker) {
  std::cout << "~~!! TRACKERS !!~~" << std::endl;
  for (const auto& tracker : multi_tracker) {
    std::cout << " TRACKER " << std::endl;
    std::cout << tracker->getStateVector() << std::endl;
  }
}

static constexpr int STEPS = 5;

int main(int ac, char **av) {
  laser_object_tracker::tracking::MultiTracker multi_tracker(
      getDistanceFunctor(),
      getDataASsociation(),
      getTracker());

  std::vector<Eigen::VectorXd> features;
  Eigen::VectorXd feature(2);
  feature << 1.0, 1.0;
  features.push_back(feature);
  feature << 2.0, 2.0;
  features.push_back(feature);

  for(int i = 0; i < STEPS; ++i) {
    std::cout << "~~~~!!!! PREDICTION STEP !!!!~~~~" << std::endl;
    multi_tracker.predict();
    printTracker(multi_tracker);
    std::cout << "~~~~!!!! UPDATE STEP !!!!~~~~" << std::endl;
    multi_tracker.update(features);
    printTracker(multi_tracker);

    feature << 3.0, 3.0;
    features.push_back(feature);
  }

}
