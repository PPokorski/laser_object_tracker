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

#include "laser_object_tracker/tracking/kalman_filter.hpp"

#include <opencv2/core/eigen.hpp>

namespace laser_object_tracker {
namespace tracking {

KalmanFilter::KalmanFilter(int state_dimensions,
                           int measurement_dimensions,
                           const Eigen::MatrixXd& transition_matrix,
                           const Eigen::MatrixXd& measurement_matrix,
                           const Eigen::MatrixXd& measurement_noise_covariance,
                           const Eigen::MatrixXd& initial_state_covariance,
                           const Eigen::MatrixXd& process_noise_covariance)
    : BaseTracking(state_dimensions, measurement_dimensions),
      kalman_filter_(state_dimensions, measurement_dimensions, 0, CV_64F),
      inverse_measurement_matrix_(measurement_matrix.cols(), measurement_matrix.rows(), CV_64F) {
  cv::eigen2cv(transition_matrix, kalman_filter_.transitionMatrix);
  cv::eigen2cv(measurement_matrix, kalman_filter_.measurementMatrix);
  cv::eigen2cv(process_noise_covariance, kalman_filter_.processNoiseCov);
  cv::eigen2cv(measurement_noise_covariance, kalman_filter_.measurementNoiseCov);
  cv::eigen2cv(initial_state_covariance, kalman_filter_.errorCovPost);

  cv::invert(kalman_filter_.measurementMatrix, inverse_measurement_matrix_, cv::DECOMP_SVD);
}

KalmanFilter::KalmanFilter(const KalmanFilter& other) noexcept
    : BaseTracking(other),
      kalman_filter_(other.state_dimensions_, other.measurement_dimensions_, 0, CV_64F) {
  copyMats(other);
}

KalmanFilter& KalmanFilter::operator=(const KalmanFilter& other) noexcept {
  BaseTracking::operator=(other);
  kalman_filter_.init(other.state_dimensions_, other.measurement_dimensions_, 0, CV_64F);
  copyMats(other);
  return *this;
}

void KalmanFilter::predict() {
  kalman_filter_.predict();
}

void KalmanFilter::update(const feature_extraction::features::Feature& measurement) {
  cv::Mat measurement_cv;
  cv::eigen2cv(measurement.observation_, measurement_cv);
  kalman_filter_.correct(measurement_cv);
}

Eigen::VectorXd KalmanFilter::getStateVector() const {
  Eigen::VectorXd state_vector;
  cv::cv2eigen(kalman_filter_.statePost, state_vector);
  return state_vector;
}

std::unique_ptr<BaseTracking> KalmanFilter::clone() const {
  return std::unique_ptr<BaseTracking>(new KalmanFilter(*this));
}

void KalmanFilter::initFromState(const feature_extraction::features::Feature& init_state) {
  cv::eigen2cv(init_state.observation_, kalman_filter_.statePost);
}

void KalmanFilter::initFromMeasurement(const feature_extraction::features::Feature& measurement) {
  cv::Mat measurement_cv;
  cv::eigen2cv(measurement.observation_, measurement_cv);
  kalman_filter_.statePost = inverse_measurement_matrix_ * measurement_cv;
}

void KalmanFilter::copyMats(const KalmanFilter& other) {
  other.kalman_filter_.controlMatrix.copyTo(kalman_filter_.controlMatrix);
  other.kalman_filter_.errorCovPost.copyTo(kalman_filter_.errorCovPost);
  other.kalman_filter_.errorCovPre.copyTo(kalman_filter_.errorCovPre);
  other.kalman_filter_.gain.copyTo(kalman_filter_.gain);
  other.kalman_filter_.measurementMatrix.copyTo(kalman_filter_.measurementMatrix);
  other.kalman_filter_.measurementNoiseCov.copyTo(kalman_filter_.measurementNoiseCov);
  other.kalman_filter_.processNoiseCov.copyTo(kalman_filter_.processNoiseCov);
  other.kalman_filter_.statePost.copyTo(kalman_filter_.statePost);
  other.kalman_filter_.statePre.copyTo(kalman_filter_.statePre);
  other.kalman_filter_.temp1.copyTo(kalman_filter_.temp1);
  other.kalman_filter_.temp2.copyTo(kalman_filter_.temp2);
  other.kalman_filter_.temp3.copyTo(kalman_filter_.temp3);
  other.kalman_filter_.temp4.copyTo(kalman_filter_.temp4);
  other.kalman_filter_.temp5.copyTo(kalman_filter_.temp5);
  other.kalman_filter_.transitionMatrix.copyTo(kalman_filter_.transitionMatrix);
  other.inverse_measurement_matrix_.copyTo(inverse_measurement_matrix_);
}
}  // namespace tracking
}  // namespace laser_object_tracker
