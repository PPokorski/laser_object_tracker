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

#ifndef LASER_OBJECT_TRACKER_TRACKING_KALMAN_FILTER_HPP
#define LASER_OBJECT_TRACKER_TRACKING_KALMAN_FILTER_HPP

#include <opencv2/video/tracking.hpp>

#include "laser_object_tracker/tracking/base_tracking.hpp"

namespace laser_object_tracker {
namespace tracking {
class KalmanFilter : public BaseTracking {
 public:
  KalmanFilter(int state_dimensions,
               int measurement_dimensions,
               const Eigen::MatrixXd& transition_matrix,
               const Eigen::MatrixXd& measurement_matrix,
               const Eigen::MatrixXd& measurement_noise_covariance,
               const Eigen::MatrixXd& initial_state_covariance,
               const Eigen::MatrixXd& process_noise_covariance);

  KalmanFilter(const KalmanFilter& other) noexcept;

  KalmanFilter(KalmanFilter&& other) noexcept = default;

  KalmanFilter& operator=(const KalmanFilter& other) noexcept;

  KalmanFilter& operator=(KalmanFilter&& other) noexcept = default;

  void initFromState(const feature_extraction::features::Feature& init_state) override;

  void initFromMeasurement(const feature_extraction::features::Feature& measurement) override;

  void predict() override;

  void update(const feature_extraction::features::Feature& measurement) override;

  Eigen::VectorXd getStateVector() const override;

  std::unique_ptr<BaseTracking> clone() const override;

private:
  void copyMats(const KalmanFilter& other);

  cv::KalmanFilter kalman_filter_;
  cv::Mat inverse_measurement_matrix_;
};
}  // namespace tracking
}  // namespace laser_object_tracker

#endif //LASER_OBJECT_TRACKER_TRACKING_KALMAN_FILTER_HPP
