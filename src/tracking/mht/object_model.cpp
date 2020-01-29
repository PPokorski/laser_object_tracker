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

#include "laser_object_tracker/tracking/mht/object_model.hpp"

namespace laser_object_tracker {
namespace tracking {
namespace mht {
cv::KalmanFilter buildKalmanFilter(int state_dimensions,
                                   int measurement_dimensions,
                                   const Eigen::MatrixXd& transition_matrix,
                                   const Eigen::MatrixXd& measurement_matrix,
                                   const Eigen::MatrixXd& measurement_noise_covariance,
                                   const Eigen::MatrixXd& initial_state_covariance,
                                   const Eigen::MatrixXd& process_noise_covariance) {
  cv::KalmanFilter kalman_filter(state_dimensions, measurement_dimensions, 0, CV_64F);

  cv::eigen2cv(transition_matrix, kalman_filter.transitionMatrix);
  cv::eigen2cv(measurement_matrix, kalman_filter.measurementMatrix);
  cv::eigen2cv(process_noise_covariance, kalman_filter.processNoiseCov);
  cv::eigen2cv(measurement_noise_covariance, kalman_filter.measurementNoiseCov);
  cv::eigen2cv(initial_state_covariance, kalman_filter.errorCovPost);

  return kalman_filter;
}

cv::KalmanFilter copyKalmanFilter(const cv::KalmanFilter& kalman_filter) {
  cv::KalmanFilter copy_kalman_filter(kalman_filter.measurementMatrix.cols,
                                      kalman_filter.measurementMatrix.rows,
                                      0,
                                      CV_64F);

  kalman_filter.controlMatrix.copyTo(copy_kalman_filter.controlMatrix);
  kalman_filter.errorCovPost.copyTo(copy_kalman_filter.errorCovPost);
  kalman_filter.errorCovPre.copyTo(copy_kalman_filter.errorCovPre);
  kalman_filter.gain.copyTo(copy_kalman_filter.gain);
  kalman_filter.measurementMatrix.copyTo(copy_kalman_filter.measurementMatrix);
  kalman_filter.measurementNoiseCov.copyTo(copy_kalman_filter.measurementNoiseCov);
  kalman_filter.processNoiseCov.copyTo(copy_kalman_filter.processNoiseCov);
  kalman_filter.statePost.copyTo(copy_kalman_filter.statePost);
  kalman_filter.statePre.copyTo(copy_kalman_filter.statePre);
  kalman_filter.temp1.copyTo(copy_kalman_filter.temp1);
  kalman_filter.temp2.copyTo(copy_kalman_filter.temp2);
  kalman_filter.temp3.copyTo(copy_kalman_filter.temp3);
  kalman_filter.temp4.copyTo(copy_kalman_filter.temp4);
  kalman_filter.temp5.copyTo(copy_kalman_filter.temp5);
  kalman_filter.transitionMatrix.copyTo(copy_kalman_filter.transitionMatrix);

  return copy_kalman_filter;
}

double calculateMahalanobisDistance(const cv::KalmanFilter& kalman_filter, const cv::Mat& measurement) {
  cv::Mat innovation_covariance = kalman_filter.measurementMatrix *
      kalman_filter.errorCovPre *
      kalman_filter.measurementMatrix.t() +
      kalman_filter.measurementNoiseCov;

  cv::Mat innovation_matrix = measurement - kalman_filter.measurementMatrix * kalman_filter.statePre;

  cv::Mat distance = innovation_matrix.t() * innovation_covariance.inv() * innovation_matrix;

  return std::sqrt(distance.at<double>(0));
}

double calculateLogLikelihood(const cv::KalmanFilter& kalman_filter, double mahalanobis_distance) {
  // Explained in:
  // https://stats.stackexchange.com/questions/296598/why-is-the-likelihood-in-kalman-filter-computed-using-filter-results-instead-of
  static double pi_coefficient = std::log(2 * M_PI);
  double covariance_coefficient = cv::determinant(kalman_filter.measurementMatrix *
      kalman_filter.errorCovPre *
      kalman_filter.measurementMatrix.t() +
      kalman_filter.measurementNoiseCov);

  double mahalanobis_coefficient = mahalanobis_distance * mahalanobis_distance;

  return -(pi_coefficient + covariance_coefficient + mahalanobis_coefficient) / 2.0;
}

double angleBetweenAngles(double target, double source) {
  double delta = target - source;
  if (delta > M_PI) {
    delta -= 2 * M_PI;
  };
  if (delta < -M_PI) {
    delta += 2 * M_PI;
  }

  return delta;
}

double absAngleBetweenAngles(double target, double source) {
  return std::abs(angleBetweenAngles(target, source));
}

double assignmentCost(const feature_extraction::features::Segment2D& lhs,
                      const feature_extraction::features::Segment2D& rhs) {
  using feature_extraction::features::distance;
  double distance_start_start = distance(lhs.getStart(), rhs.getStart());
  double distance_start_end = distance(lhs.getStart(), rhs.getEnd());
  double distance_end_start = distance(lhs.getEnd(), rhs.getStart());
  double distance_end_end = distance(lhs.getEnd(), rhs.getEnd());
  return std::min(distance_start_start + distance_end_end, distance_start_end + distance_end_start);
}

std::pair<const feature_extraction::features::Segment2D*,
          const feature_extraction::features::Segment2D*>
          ObjectState::updateReferencePointSource(const ReferencePointSource& reference_source) {
  using Segment2D = feature_extraction::features::Segment2D;
  using Corner2D = feature_extraction::features::Corner2D;

  std::pair<const Segment2D*, const Segment2D*> assignment(nullptr, nullptr);
  bool is_segment = std::holds_alternative<Segment2D>(reference_source);

  if (is_segment) {
    auto& segment = std::get<Segment2D>(reference_source);

    if (!is_second_initialized_) {
      segment_1_ = Segment2D(segment.getStart(),
                             segment.getOrientation(),
                             std::max(segment_1_.length(),
                                      segment.length()));

      assignment.first = &segment_1_;
    } else {
      Segment2D* closer_segment;
      if (assignmentCost(segment_1_, segment) <= assignmentCost(segment_2_, segment)) {
        closer_segment = &segment_1_;
      } else {
        closer_segment = &segment_2_;
      }

      (*closer_segment) = Segment2D(segment.getStart(),
                                    segment.getOrientation(),
                                    std::max(closer_segment->length(),
                                             segment.length()));
      assignment.first = closer_segment;
    }
  } else {
    auto& corner = std::get<Corner2D>(reference_source);

    // Minimize the cost of assignment, cost being the angle between orientations
    double segment_1_longer = assignmentCost(segment_1_, corner.getSegmentLonger());
    double segment_1_shorter = assignmentCost(segment_1_, corner.getSegmentShorter());

    // Cost assignment for uninitialized value is 0.0
    double segment_2_longer = is_second_initialized_ *
        assignmentCost(segment_2_, corner.getSegmentLonger());
    double segment_2_shorter = is_second_initialized_ *
        assignmentCost(segment_2_, corner.getSegmentShorter());

    Segment2D* closer_to_longer,* closer_to_shorter;
    if (segment_1_longer + segment_2_shorter <= segment_1_shorter + segment_2_longer) {
      closer_to_longer = &segment_1_;
      closer_to_shorter = &segment_2_;
    } else {
      closer_to_longer = &segment_2_;
      closer_to_shorter = &segment_1_;
    }

    (*closer_to_longer) = Segment2D(corner.getSegmentLonger().getStart(),
                                    corner.getSegmentLonger().getOrientation(),
                                    std::max(closer_to_longer->length(),
                                             corner.getSegmentLonger().length()));

    (*closer_to_shorter) = Segment2D(corner.getSegmentShorter().getStart(),
                                     corner.getSegmentShorter().getOrientation(),
                                     std::max(closer_to_shorter->length(),
                                              corner.getSegmentShorter().length()));

    assignment.first = closer_to_longer;
    assignment.second = closer_to_shorter;
    is_second_initialized_ = true;
  }

  return assignment;
}

void ObjectState::initializeWithPointSource(const ObjectState::ReferencePointSource& source) {
  using Segment2D = feature_extraction::features::Segment2D;
  using Corner2D = feature_extraction::features::Corner2D;
  bool is_segment = std::holds_alternative<Segment2D>(source);

  if (is_segment) {
    segment_1_ = std::get<Segment2D>(source);

    is_second_initialized_ = false;
  } else {
    auto& corner = std::get<Corner2D>(source);

    segment_1_ = corner.getSegmentLonger();
    segment_2_ = corner.getSegmentShorter();

    is_second_initialized_ = true;
  }
}

MDL_STATE *ObjectModel::getNewState(int i, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  ObjectState* next_state = nullptr;
  if (object_state == nullptr) {
    ObjectState::State state_vector;
    state_vector.setZero();
    state_vector.head<2>() = object_report->getReferencePoint();
    next_state = new ObjectState(this,
                                 time_step_,
                                 object_report->getTimestamp(),
                                 start_log_likelihood_,
                                 0,
                                 0,
                                 false,
                                 object_report->getReferencePointType(),
                                 object_report->getReferencePointSource(),
                                 std::move(getKalmanFilter()),
                                 state_vector);
  } else if (object_report == nullptr) {
    next_state = new ObjectState(*object_state);

    next_state->predict();
    next_state->incrementTimesSkipped();
  } else if (object_report->hasValidReferencePoint()) {
    next_state = new ObjectState(*object_state);

    next_state->predict();
    auto assignment = next_state->updateReferencePointSource(object_report->getReferencePointSource());

    // TODO Think which one would be better
    // if (next_state->getReferencePointType() != object_report->getReferencePointType()) {
    if (next_state->getReferencePointType() != ObjectState::ReferencePointType::CORNER ||
        object_report->getReferencePointType() != ObjectState::ReferencePointType::CORNER) {
      tryMoveState(next_state, object_report, assignment);
    }

    if (!updateState(next_state, object_report)) {
      delete next_state;
      next_state = nullptr;
    }
  }

  return next_state;
}

double ObjectModel::getIsConfirmedTargetLikelihood(const ObjectState& state) const {
  if (state.isConfirmedTarget()) {
    return 1.0;
  } else {
    return 1.0 - std::exp(-target_confirmation_rate_ * state.getTimesUpdated());
  }
}

double ObjectModel::getContinueLikelihood(const ObjectState& state) const {
  return std::pow(hold_target_probability_, state.getTimesSkipped());
}

double ObjectModel::mahalanobisDistance(const ObjectState& state, const ObjectReport& report) const {
  cv::Mat measurement;
  cv::eigen2cv(report.getObject().getReferencePoint(), measurement);

  const auto& kalman_filter = state.getKalmanFilter();

  return calculateMahalanobisDistance(kalman_filter, measurement);
}

cv::KalmanFilter ObjectModel::getKalmanFilter() const {
  return buildKalmanFilter(ObjectState::STATE_DIMENSION,
                           ObjectState::MEASUREMENT_DIMENSION,
                           state_transition_,
                           measurement_matrix_,
                           measurement_noise_covariance_,
                           initial_state_covariance_,
                           process_noise_covariance_);
}

void ObjectModel::tryMoveState(ObjectState* state,
                               const ObjectReport* report,
                               const std::pair<const feature_extraction::features::Segment2D*,
                                               const feature_extraction::features::Segment2D*>& assignment) const {
  using namespace feature_extraction::features;
  const Point2D& position = state->getPosition();
  const Point2D& measured_position = report->getReferencePoint();
  Segment2D measurement_segment(position,
                                measured_position);
  double orientation_1 = assignment.first->getOrientation();

  if (absAngleBetweenAngles(orientation_1, measurement_segment.getOrientation()) > M_PI_2) {
    orientation_1 += M_PI;
  }
  double length_1 = assignment.first->length();

  Point2D projected_position_1 = position;
  projected_position_1.x() += length_1 * std::cos(orientation_1);
  projected_position_1.y() += length_1 * std::sin(orientation_1);

  if (assignment.second == nullptr) {
    if (squaredDistance(projected_position_1, measured_position) <
        squaredDistance(position, measured_position)) {
      state->setPosition(projected_position_1);
    }
  } else {
    double orientation_2 = assignment.second->getOrientation();

    if (absAngleBetweenAngles(orientation_2, measurement_segment.getOrientation()) > M_PI_2) {
      orientation_2 += M_PI;
    }
    double length_2 = assignment.second->length();

    Point2D projected_position_2 = position;
    projected_position_2.x() += length_2 * std::cos(orientation_2);
    projected_position_2.y() += length_2 * std::sin(orientation_2);

    const Point2D& most_probable = std::min({position, projected_position_1, projected_position_2},
                                            [&measured_position](const Point2D& lhs, const Point2D& rhs)
                                            {return squaredDistance(lhs, measured_position) <
                                                squaredDistance(rhs, measured_position);});

    state->setPosition(most_probable);
  }
}

bool ObjectModel::updateState(ObjectState* state, const ObjectReport* report) const {
  double mahalanobis_distance = mahalanobisDistance(*state, *report);
  if (mahalanobis_distance <= max_mahalanobis_distance_) {
    state->incrementTimesUpdated();
    double confirmed_target_likelihood = getIsConfirmedTargetLikelihood(*state);
    if (confirmed_target_likelihood > target_confirmation_log_threshold_) {
      state->setIsConfirmedTarget(true);
      confirmed_target_likelihood = 1.0;
    }

    state->setLogLikelihood(calculateLogLikelihood(state->getKalmanFilter(), mahalanobis_distance) +
                            std::log(confirmed_target_likelihood));

    const ObjectState::Measurement& measurement = report->getReferencePoint();
    state->update(measurement);
    state->resetTimesSkipped();

    return true;
  } else {
    return false;
  }
}

void ObjectTracker::measure(const std::list<REPORT*>& new_reports) {
  for (auto report : new_reports) {
    installReport(report);
  }
}

void ObjectTracker::startTrack(int i, int i1, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  std::cout << "Start " << i << std::endl;

  TrackElement track_element{
    object_state->getLogLikelihood(),
    object_state->getTimestamp(),
    true,
    object_state->getPosition(),
    object_state->getPositionCovariance(),
    object_state->getVelocity(),
    object_state->getVelocityCovariance(),
    object_state->getPolyline()
  };

  verify(i,
         track_element);
}

void ObjectTracker::continueTrack(int i, int i1, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  std::cout << "Continue " << i << std::endl;

  TrackElement track_element{
      object_state->getLogLikelihood(),
      object_state->getTimestamp(),
      true,
      object_state->getPosition(),
      object_state->getPositionCovariance(),
      object_state->getVelocity(),
      object_state->getVelocityCovariance(),
      object_state->getPolyline()
  };

  verify(i,
         track_element);
}

void ObjectTracker::skipTrack(int i, int i1, MDL_STATE *state) {
  auto object_state = dynamic_cast<ObjectState*>(state);

  std::cout << "Skip " << i << std::endl;

  TrackElement track_element{
      object_state->getLogLikelihood(),
      object_state->getTimestamp(),
      false,
      object_state->getPosition(),
      object_state->getPositionCovariance(),
      object_state->getVelocity(),
      object_state->getVelocityCovariance(),
      object_state->getPolyline()
  };

  verify(i,
         track_element);
}

void ObjectTracker::endTrack(int i, int i1) {
  tracks_.remove_if([i](const auto& track) { return track.id_ == i;});
}

void ObjectTracker::falseAlarm(int i, MDL_REPORT *report) {
  false_alarms_.emplace_back((dynamic_cast<ObjectReport*>(report)));
}

Track& ObjectTracker::findTrack(int id) {
  auto it = std::find_if(tracks_.begin(),
                         tracks_.end(),
                         [id](const auto& track) { return track.id_ == id;});

  if (it != tracks_.end()) {
    return *it;
  } else {
    tracks_.emplace_back(id);
    return tracks_.back();
  }
}

void ObjectTracker::verify(int id,
                           const TrackElement& track_element) {
  auto& track = findTrack(id);
  track.track_.push_back(track_element);
}
}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker