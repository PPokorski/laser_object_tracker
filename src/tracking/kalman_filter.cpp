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
                           const Eigen::MatrixXd& process_noise_covariance) :
    BaseTracking(state_dimensions, measurement_dimensions),
    kalman_filter_(state_dimensions, measurement_dimensions, 0, CV_64F),
    inverse_measurement_matrix_(measurement_matrix.cols(), measurement_matrix.rows(), CV_64F) {
  cv::eigen2cv(transition_matrix, kalman_filter_.transitionMatrix);
  cv::eigen2cv(measurement_matrix, kalman_filter_.measurementMatrix);
  cv::eigen2cv(process_noise_covariance, kalman_filter_.processNoiseCov);
  cv::eigen2cv(measurement_noise_covariance, kalman_filter_.measurementNoiseCov);
  cv::eigen2cv(initial_state_covariance, kalman_filter_.errorCovPost);

  cv::invert(kalman_filter_.measurementMatrix, inverse_measurement_matrix_, cv::DECOMP_SVD);
}

void KalmanFilter::predict() {
  kalman_filter_.predict();
}

void KalmanFilter::update(const Eigen::VectorXd& measurement) {
  cv::Mat measurement_cv;
  cv::eigen2cv(measurement, measurement_cv);
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

void KalmanFilter::initFromState(const Eigen::VectorXd& init_state) {
  cv::eigen2cv(init_state, kalman_filter_.statePost);
}
void KalmanFilter::initFromMeasurement(const Eigen::VectorXd& measurement) {
  cv::Mat measurement_cv;
  cv::eigen2cv(measurement, measurement_cv);
  kalman_filter_.statePost = inverse_measurement_matrix_ * measurement_cv;
}
}  // namespace tracking
}  // namespace laser_object_tracker
