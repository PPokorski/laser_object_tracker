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

#include "laser_object_tracker/tracking/corner_tracker.hpp"

#include <iostream>

namespace laser_object_tracker {
namespace tracking {
constexpr int CornerTracker::STATE_DIMENSIONS_;
constexpr int CornerTracker::MEASUREMENT_DIMENSIONS_;

CornerTracker::CornerTracker(double time_step,
                             double min_distance_from_corner,
                             const Eigen::MatrixXd& measurement_noise_covariance,
                             const Eigen::MatrixXd& initial_state_covariance,
                             const Eigen::MatrixXd& process_noise_covariance)
    : KalmanFilter(STATE_DIMENSIONS_,
                   MEASUREMENT_DIMENSIONS_,
                   (Eigen::MatrixXd(STATE_DIMENSIONS_, STATE_DIMENSIONS_) << 1.0, 0.0, 0.0, time_step, 0.0, 0.0,
                                                                            0.0, 1.0, 0.0, 0.0, time_step, 0.0,
                                                                            0.0, 0.0, 1.0, 0.0, 0.0, time_step,
                                                                            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                                                            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                                            0.0, 0.0, 0.0, 0.0, 0.0, 1.0).finished(),
                   (Eigen::MatrixXd(MEASUREMENT_DIMENSIONS_, STATE_DIMENSIONS_) << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                                                   0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                                                                   0.0, 0.0, 1.0, 0.0, 0.0, 0.0).finished(),
                   measurement_noise_covariance,
                   initial_state_covariance,
                   process_noise_covariance),
      min_distance_from_corner_(min_distance_from_corner),
      was_line_(false) {}

void CornerTracker::initFromMeasurement(const feature_extraction::features::Feature& measurement) {
  feature_extraction::features::Feature internal_measurement = measurement;
  if (isLine(measurement.observation_)) {
    internal_measurement.observation_ = getMeasurementFromLine(measurement);
    was_line_ = true;
//    std::cout << "Initializing as line" << std::endl;
  } else {
    internal_measurement.observation_ = getMeasurementFromCorner(measurement.observation_);
    was_line_ = false;
//    std::cout << "Initializing as corner" << std::endl;
  }
  KalmanFilter::initFromMeasurement(internal_measurement);

  previous_measurement_ = measurement;
}

void CornerTracker::update(const feature_extraction::features::Feature& measurement) {
//  if (measurement.observation_(0) > 4.6 && measurement.observation_(0) < 4.7 &&
//      measurement.observation_(1) > 0.2 && measurement.observation_(1) < 0.3) {
//    std::cout << "sth" << std::endl;
//  }

  feature_extraction::features::Feature feature = measurement;
  bool is_line = isLine(measurement.observation_);
  if (is_line) {
//    std::cout << "Updating as a line" << std::endl;
    feature.observation_ = getMeasurementFromLine(measurement);
    handleAsCorner(measurement, feature);
    if (!was_line_) {
//      std::cout << "Changing orientation" << std::endl;
      kalman_filter_.statePre.at<double>(2) = feature.observation_(2);
    }
  } else {
//    std::cout << "Updating as a corner" << std::endl;
    feature.observation_ = getMeasurementFromCorner(measurement.observation_);
    handleAsCorner(measurement, feature);
    if (was_line_) {
//      std::cout << "Changing orientation" << std::endl;
      kalman_filter_.statePre.at<double>(2) = feature.observation_(2);
    }
  }

  KalmanFilter::update(feature);

  if (is_line) {
    handleAsLine(measurement, feature);
  }

  was_line_ = is_line;
  previous_measurement_ = measurement;
}

std::unique_ptr<BaseTracking> CornerTracker::clone() const {
  return std::unique_ptr<BaseTracking>(new CornerTracker(*this));
}

bool CornerTracker::isLine(const Eigen::VectorXd& corner) const {
  double min_distance_from_corner_squared = min_distance_from_corner_ * min_distance_from_corner_;
  return (corner.head<2>() - corner.segment<2>(2)).squaredNorm() < min_distance_from_corner_squared ||
         (corner.head<2>() - corner.tail<2>()).squaredNorm() < min_distance_from_corner_squared;
}

Eigen::VectorXd CornerTracker::getMeasurementFromCorner(const Eigen::VectorXd& corner) const {
  Eigen::VectorXd measurement(MEASUREMENT_DIMENSIONS_);

  measurement.head<2>() = corner.head<2>();
  measurement(2) = std::atan2(corner(3) - corner(1), corner(2) - corner(0));

  return measurement;
}

Eigen::VectorXd CornerTracker::getMeasurementFromLine(const feature_extraction::features::Feature& corner) const {
  Eigen::VectorXd measurement(MEASUREMENT_DIMENSIONS_);

  measurement.head<2>() = corner.observation_.head<2>();

  feature_extraction::features::Feature line = getLineFromCorner(corner);

  measurement(2) = std::atan2(line.observation_(3) - line.observation_(1),
                              line.observation_(2) - line.observation_(0));

  return measurement;
}

void CornerTracker::handleAsCorner(const feature_extraction::features::Feature& measurement,
                                   const feature_extraction::features::Feature& processed_measurement) {
  if(moveStatePoint(measurement)) {
//    std::cout << "Moving point" << std::endl;
    kalman_filter_.statePre.at<double>(0) = processed_measurement.observation_(0);
    kalman_filter_.statePre.at<double>(1) = processed_measurement.observation_(1);
    kalman_filter_.statePre.at<double>(2) = processed_measurement.observation_(2);
  }
}

void CornerTracker::handleAsLine(const feature_extraction::features::Feature& measurement,
                                 const feature_extraction::features::Feature& processed_measurement) {
  feature_extraction::features::Feature line = getLineFromCorner(measurement);

  if (line.vector_bool_.at(0) ||
      line.vector_bool_.at(1)) {
//    std::cout << "Canceling some velocity" << std::endl;
    double velocity_x = kalman_filter_.statePost.at<double>(3),
           velocity_y = kalman_filter_.statePost.at<double>(4),
           orientation = kalman_filter_.statePost.at<double>(2);

    double sinus = std::sin(orientation);
    double cosinus = std::cos(orientation);

    double local_velocity_x =  velocity_x * cosinus + velocity_y * sinus,
           local_velocity_y = -velocity_x * sinus + velocity_y * cosinus;

    if (measurement.vector_bool_.at(0) &&
        local_velocity_x < 0.0) {
//      std::cout << "Canceling negative velocity" << std::endl;
      local_velocity_x = 0.0;
    }

    if (measurement.vector_bool_.at(1) &&
        local_velocity_x > 0.0) {
//      std::cout << "Canceling positive velocity" << std::endl;
      local_velocity_x = 0.0;
    }

    velocity_x = local_velocity_x * cosinus - local_velocity_y * sinus;
    velocity_y = local_velocity_x * sinus + local_velocity_y * cosinus;

    kalman_filter_.statePost.at<double>(3) = velocity_x;
    kalman_filter_.statePost.at<double>(4) = velocity_y;
  }
}

bool CornerTracker::moveStatePoint(const feature_extraction::features::Feature& current_measurement) const {
  double distance_corner_corner =
      (previous_measurement_.observation_.head<2>() - current_measurement.observation_.head<2>()).squaredNorm(),
         distance_corner_point_1 =
      (previous_measurement_.observation_.head<2>() - current_measurement.observation_.segment<2>(2)).squaredNorm(),
         distance_corner_point_2 =
      (previous_measurement_.observation_.head<2>() - current_measurement.observation_.tail<2>()).squaredNorm();

  return distance_corner_corner > distance_corner_point_1 ||
         distance_corner_corner > distance_corner_point_2;
}

feature_extraction::features::Feature CornerTracker::getLineFromCorner(const feature_extraction::features::Feature& corner) const {
  feature_extraction::features::Feature line;
  line.observation_.resize(4);
  line.observation_.head<2>() = corner.observation_.head<2>();

  double line_1_length = (corner.observation_.head<2>() - corner.observation_.segment<2>(2)).squaredNorm();
  double line_2_length = (corner.observation_.head<2>() - corner.observation_.tail<2>()).squaredNorm();

  line.observation_.tail<2>() = line_1_length > line_2_length ? corner.observation_.segment<2>(2) :
                                                                corner.observation_.tail<2>();

  if (line_1_length > line_2_length) {
    line.vector_bool_ = {corner.vector_bool_.at(2), corner.vector_bool_.at(1)};
  } else {
    line.vector_bool_ = {corner.vector_bool_.at(1), corner.vector_bool_.at(2)};
  }

  return line;
}
}  // namespace tracking
}  // namespace laser_object_tracker
