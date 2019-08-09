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

#include "laser_object_tracker/tracking/mht/motion_model.hpp"

#include <algorithm>

#include "laser_object_tracker/feature_extraction/features/point_2d.hpp"

int mht::internal::g_time = 0;

namespace laser_object_tracker {
namespace tracking {
namespace mht {
ConstVelocityModel::ConstVelocityModel(double position_measure_variance_x,
                                       double position_measure_variance_y,
                                       double lambda_x,
                                       double start_probability,
                                       double detection_probability,
                                       double max_mahalanobis_distance,
                                       double process_variance,
                                       double state_variance,
                                       double time_step)
    : MODEL(),
      lambda_x_(lambda_x),
      start_log_likelihood_(std::log(start_probability)),
      skip_log_likelihood_(std::log(1.0 - detection_probability)),
      detect_log_likelihood_(std::log(detection_probability)),
      max_mahalanobis_distance_(max_mahalanobis_distance),
      process_variance_(process_variance),
      state_variance_(state_variance),
      time_step_(time_step),
      measurement_covariance_(2, 2),
      initial_covariance_(4, 4) {
  measurement_covariance_.set(position_measure_variance_x, 0.0,
                              0.0, position_measure_variance_y);

  initial_covariance_.set(position_measure_variance_x, 0.0, 0.0, 0.0,
                          0.0, state_variance_, 0.0, 0.0,
                          0.0, 0.0, position_measure_variance_y, 0.0,
                          0.0, 0.0, 0.0, state_variance_);
}

int ConstVelocityModel::beginNewStates(MDL_STATE* state, MDL_REPORT* report) {
  return 1;
}

MDL_STATE* ConstVelocityModel::getNewState(int i,
                                           MDL_STATE* state,
                                           MDL_REPORT* report) {
  auto model_state = (ConstVelocityState*) state;
  auto model_report = (PositionReport*) report;

  double dx, dy;
  if (model_state != nullptr &&
      model_report != nullptr &&
      model_state->getDx() == 0.0 &&
      model_state->getDy() == 0.0) {
    dx = model_report->getX() - model_state -> getX();
    dy = model_report->getY() - model_state -> getY();

    model_state->setDx(dx);
    model_state->setDy(dy);
  }

  return getNextState(model_state, model_report);
}

double ConstVelocityModel::getEndLogLikelihood(MDL_STATE* state) {
  auto model_state = (ConstVelocityState*) state;
  double end_probability = 1.0 - std::exp(-model_state->getTimesSkipped() / lambda_x_);
  // End probability cannot be 0.0
  end_probability = std::nextafter(end_probability, 1.0);

  return end_log_likelihood_ = std::log(end_probability);
}

double ConstVelocityModel::getContinueLogLikelihood(MDL_STATE* state) {
  auto model_state = (ConstVelocityState*) state;
  double end_probability = 1.0 - std::exp(-model_state->getTimesSkipped() / lambda_x_);
  // End probability cannot be 0.0
  end_probability = std::nextafter(end_probability, 1.0);

  return continue_log_likelihood_ = std::log(1.0 - end_probability);
}

double ConstVelocityModel::getSkipLogLikelihood(MDL_STATE* state) {
  return skip_log_likelihood_;
}

double ConstVelocityModel::getDetectLogLikelihood(MDL_STATE* state) {
  return detect_log_likelihood_;
}

ConstVelocityState* ConstVelocityModel::getNextState(ConstVelocityState* state,
                                                     PositionReport* report) {
  ConstVelocityState* next_state;
  if (state == nullptr) {
    double x = report->getX(),
           y = report->getY();

    next_state = new ConstVelocityState(this,
                                        x,
                                        0.0,
                                        y,
                                        0.0,
                                        initial_covariance_,
                                        start_log_likelihood_,
                                        0,
                                        time_step_);
  } else if (report == nullptr) {
    state->setup(process_variance_, measurement_covariance_);

    next_state = new ConstVelocityState(this,
                                        state->getPredictedX(),
                                        state->getPredictedDx(),
                                        state->getPredictedY(),
                                        state->getPredictedDy(),
                                        state->getUpdatedStateCovariance(),
                                        0.0,
                                        state->getTimesSkipped() + 1,
                                        time_step_);
  } else {
    MATRIX innovation_matrix(2, 1);
    double mahalanobis_distance;
    MATRIX observation_matrix(2, 4);
    observation_matrix.set(1.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 1.0, 0.0);

    state->setup(process_variance_, measurement_covariance_);
    innovation_matrix = report->getMeasurement() - observation_matrix * state->getStatePrediction();
    mahalanobis_distance = (innovation_matrix.trans() * state->getInnovationCovarianceInversed() * innovation_matrix)();

    if (mahalanobis_distance > max_mahalanobis_distance_) {
      next_state = nullptr;
    } else {
      MATRIX new_state = state->getStatePrediction() + state->getFilterGain() * innovation_matrix;

      next_state = new ConstVelocityState(this,
                                          new_state(0),
                                          new_state(1),
                                          new_state(2),
                                          new_state(3),
                                          state->getUpdatedStateCovariance(),
                                          state->getLogLikelihoodCoefficient() - mahalanobis_distance / 2.0,
                                          0,
                                          time_step_);
    }
  }
  return next_state;
}

void ConstVelocityState::setup(double process_variance, const MATRIX& observation_noise_) {
  if (is_setup_) {
    return;
  }

  double time_step_squared = time_step_ * time_step_;
  double time_step_cubed = time_step_squared * time_step_;

  MATRIX state_transition(4, 4);
  state_transition.set(1.0, time_step_, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, time_step_,
                       0.0, 0.0, 0.0, 1.0);

  MATRIX process_noise_covariance(4, 4);
  process_noise_covariance.set(time_step_cubed / 3.0, time_step_squared / 2.0, 0.0, 0.0,
                               time_step_squared / 2.0, time_step_, 0.0, 0.0,
                               0.0, 0.0, time_step_cubed / 3.0, time_step_squared / 2.0,
                               0.0, 0.0, time_step_squared / 2.0, time_step_);

  process_noise_covariance = process_noise_covariance * process_variance;

  MATRIX observation_matrix(2, 4);
  observation_matrix.set(1.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 1.0, 0.0);

  MATRIX covariance_prediction = state_transition * state_covariance_ * state_transition.trans() +
                                 process_noise_covariance;

  MATRIX innovation_covariance = observation_matrix * covariance_prediction * observation_matrix.trans() +
                                 observation_noise_;
  // TODO LOG_NORMFACTOR = 1.5963597 ??
  log_likelihood_coefficient_ = -(1.5963597 + std::log(innovation_covariance.det()) / 2.0);

  innovation_covariance_inversed_ = new MATRIX(innovation_covariance.inv());
  filter_gain_ = new MATRIX(covariance_prediction * observation_matrix.trans() * (*innovation_covariance_inversed_));

  updated_state_covariance_ = new MATRIX(covariance_prediction -
                                         ((*filter_gain_) * innovation_covariance * filter_gain_->trans()));

  state_prediction_ = new MATRIX(state_transition * state_);

  is_setup_ = true;
}

void MHTTracker::measure(const std::list<REPORT*>& new_reports) {
  for (auto report : new_reports) {
    installReport(report);
  }
}

void MHTTracker::startTrack(int i, int i1, MDL_STATE* state, MDL_REPORT* report) {
  auto model_state = (ConstVelocityState*) state;
  auto model_report = (PositionReport*) report;

  verify(i,
         model_report->getX(),
         model_report->getY(),
         model_state->getX(),
         model_state->getY(),
         model_state->getLogLikelihood(),
         model_report->getFrameNumber(),
         model_report->getCornerId());
}

void MHTTracker::continueTrack(int i, int i1, MDL_STATE* state, MDL_REPORT* report) {
  auto model_state = (ConstVelocityState*) state;
  auto model_report = (PositionReport*) report;

  verify(i,
         model_report->getX(),
         model_report->getY(),
         model_state->getX(),
         model_state->getY(),
         model_state->getLogLikelihood(),
         model_report->getFrameNumber(),
         model_report->getCornerId());
}

void MHTTracker::skipTrack(int i, int i1, MDL_STATE* state) {
  auto model_state = (ConstVelocityState*) state;

  double nan = std::numeric_limits<double>::quiet_NaN();
  verify(i,
         nan,
         nan,
         model_state->getX(),
         model_state->getY(),
         model_state->getLogLikelihood(),
         -1,
         0);
}

void MHTTracker::endTrack(int i, int i1) {
  tracks_.remove_if([i](const auto& track) { return track.getId() == i;});
}

void MHTTracker::falseAlarm(int i, MDL_REPORT* report) {
  false_alarms_.emplace_back(((PositionReport*) report));
}

Track* MHTTracker::findTrack(int id) {
  auto it = std::find_if(tracks_.begin(),
                         tracks_.end(),
                         [id](const auto& track) { return track.getId() == id;});

  if (it != tracks_.end()) {
    return &(*it);
  } else {
    tracks_.emplace_back(id);
    return &tracks_.back();
  }
}

void MHTTracker::verify(int track_id,
                        double state_x,
                        double state_y,
                        double report_x,
                        double report_y,
                        double likelihood,
                        int frame,
                        size_t corner_id) {
  auto track = findTrack(track_id);
  track->track_.emplace_back(state_x,
                             state_y,
                             0.0,
                             0.0,
                             report_x,
                             report_y,
                             likelihood,
                             frame,
                             corner_id);
}
}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker
