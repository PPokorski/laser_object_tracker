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

#ifndef LASER_OBJECT_TRACKER_TRACKING_MHT_MOTION_MODEL_HPP
#define LASER_OBJECT_TRACKER_TRACKING_MHT_MOTION_MODEL_HPP

#include <list>

#include <Eigen/Core>

#include <opencv2/core/eigen.hpp>
#include <opencv2/video/tracking.hpp>

#include <mht/except.h>
#include <mht/matrix.h>
#include <mht/mdlmht.h>

#include "laser_object_tracker/tracking/mht/track.hpp"

namespace mht {
namespace internal {
extern int g_time;
}  // namespace internal
}  // namespace mht

namespace laser_object_tracker {
namespace tracking {
namespace mht {
class PositionReport : public MDL_REPORT {
 public:
  PositionReport(double false_alarm_log_likelihood,
                 double x,
                 double y,
                 int frame_number,
                 size_t corner_id)
    : MDL_REPORT(),
      false_alarm_log_likelihood_(false_alarm_log_likelihood),
      measurement_(2, 1),
      frame_number_(frame_number),
      corner_id_(corner_id) {
    measurement_.set(x,  y);
  }

  double getFalarmLogLikelihood() override {
    return false_alarm_log_likelihood_;
  }

  const MATRIX& getMeasurement() const {
    return measurement_;
  }

  double getX() const {
    return measurement_(0);
  }

  double getY() const {
    return measurement_(1);
  }

  int getFrameNumber() const {
    return frame_number_;
  }

  size_t getCornerId() const {
    return corner_id_;
  }

  void print() override {
    measurement_.print();
  }
  void describe(int spaces) override {
    measurement_.print(spaces);
  }

 private:
  double false_alarm_log_likelihood_;

  MATRIX measurement_;

  int frame_number_;
  size_t corner_id_;
};

class ConstVelocityState;

class ConstVelocityModel : public MODEL {
 public:
  ConstVelocityModel(double position_measure_variance_x,
                     double position_measure_variance_y,
                     double lambda_x,
                     double start_probability,
                     double detection_probability,
                     double max_mahalanobis_distance,
                     double process_variance,
                     double state_variance,
                     double time_step);

  int beginNewStates(MDL_STATE* state, MDL_REPORT* report) override;

  MDL_STATE* getNewState(int i, MDL_STATE* state, MDL_REPORT* report) override;

  double getEndLogLikelihood(MDL_STATE* state) override;

  double getContinueLogLikelihood(MDL_STATE* state) override;

  double getSkipLogLikelihood(MDL_STATE* state) override;

  double getDetectLogLikelihood(MDL_STATE* state) override;

 private:
  ConstVelocityState* getNextState(ConstVelocityState* state,
                                   PositionReport* report);

  double lambda_x_;
  double start_log_likelihood_;
  double end_log_likelihood_;
  double continue_log_likelihood_;
  double skip_log_likelihood_;
  double detect_log_likelihood_;

  double max_mahalanobis_distance_;

  double process_variance_;
  double state_variance_;

  double time_step_;

  MATRIX measurement_covariance_;
  MATRIX initial_covariance_;
};

class ConstVelocityState : public MDL_STATE {
 public:
  ConstVelocityState(ConstVelocityModel* model,
                     double x,
                     double dx,
                     double y,
                     double dy,
                     const MATRIX& state_covariance,
                     double log_likelihood,
                     int times_skipped,
                     double time_step)
    : MDL_STATE(model),
      state_(4, 1),
      state_covariance_(state_covariance),
      is_setup_(false),
      log_likelihood_(log_likelihood),
      times_skipped_(times_skipped),
      time_step_(time_step),
      log_likelihood_coefficient_(0.0),
      innovation_covariance_inversed_(nullptr),
      filter_gain_(nullptr),
      updated_state_covariance_(nullptr),
      state_prediction_(nullptr) {
    state_(0) = x;
    state_(1) = dx;
    state_(2) = y;
    state_(3) = dy;
  }

  ConstVelocityState(const ConstVelocityState& other)
      : MDL_STATE(other.getMdl()),
        state_(other.state_),
        state_covariance_(other.state_covariance_),
        is_setup_(false),
        log_likelihood_(other.log_likelihood_),
        times_skipped_(other.times_skipped_),
        time_step_(other.time_step_),
        log_likelihood_coefficient_(0.0),
        innovation_covariance_inversed_(nullptr),
        filter_gain_(nullptr),
        updated_state_covariance_(nullptr),
        state_prediction_(nullptr) {}

  ~ConstVelocityState() {
    cleanup();
  }

  void setup(double process_variance, const MATRIX& observation_noise_);

  double getLogLikelihood() override {
    return log_likelihood_;
  }

  double getX() const {
    return state_(0);
  }

  double getDx() const {
    return state_(1);
  }

  void setDx(double dx) {
    state_(1) = dx;
  }

  double getY() const {
    return state_(2);
  }

  double getDy() const {
    return state_(3);
  }

  void setDy(double dy) {
    state_(3) = dy;
  }

  double getPredictedX() const {
    return (*state_prediction_)(0);
  }

  double getPredictedDx() const {
    return (*state_prediction_)(1);
  }

  double getPredictedY() const {
    return (*state_prediction_)(2);
  }

  double getPredictedDy() const {
    return (*state_prediction_)(3);
  }

  int getTimesSkipped() const {
    return times_skipped_;
  }

  double getTimeStep() const {
    return time_step_;
  }

  double getLogLikelihoodCoefficient() const {
    return log_likelihood_coefficient_;
  }

  const MATRIX& getInnovationCovarianceInversed() const {
    return *innovation_covariance_inversed_;
  }

  const MATRIX& getFilterGain() const {
    return *filter_gain_;
  }

  const MATRIX& getUpdatedStateCovariance() const {
    return *updated_state_covariance_;
  }

  const MATRIX& getStatePrediction() const {
    return *state_prediction_;
  }

 private:
  void cleanup() {
    if (is_setup_) {
      delete innovation_covariance_inversed_;
      innovation_covariance_inversed_ = nullptr;

      delete filter_gain_;
      filter_gain_ = nullptr;

      delete updated_state_covariance_;
      updated_state_covariance_ = nullptr;

      delete updated_state_covariance_;
      updated_state_covariance_ = nullptr;

      delete state_prediction_;
      state_prediction_ = nullptr;

      is_setup_ = false;
    }
  }

  MATRIX state_;
  MATRIX state_covariance_;

  bool is_setup_;

  double log_likelihood_;

  int times_skipped_;

  double time_step_;
  double log_likelihood_coefficient_;

  MATRIX* innovation_covariance_inversed_;
  MATRIX* filter_gain_;
  MATRIX* updated_state_covariance_;
  MATRIX* state_prediction_;
};

class FalseAlarm : public DLISTnode {
 public:
  explicit FalseAlarm(const PositionReport* report)
      : DLISTnode(),
        x_(report->getX()),
        y_(report->getY()),
        frame_number_(report->getFrameNumber()),
        corner_id_(report->getCornerId()) {}

 protected:
  MEMBERS_FOR_DLISTnode(FalseAlarm)

 private:
  double x_, y_;
  int frame_number_;
  size_t corner_id_;
};

class MHTTracker : public MDL_MHT {
 public:
  MHTTracker(double false_alarm_likelihood,
             int max_depth,
             double min_g_hypo_ratio,
             int max_g_hypos,
             const ptrDLIST_OF<MODEL>& models)
    : MDL_MHT(max_depth,
              min_g_hypo_ratio,
              max_g_hypos),
      false_alarm_log_likelihood_(std::log(false_alarm_likelihood)) {
    m_modelList.appendCopy(models);
  }

  const std::list<Track>& getTracks() const {
    return tracks_;
  }

  const std::list<FalseAlarm>& getFalseAlarms() const {
    return false_alarms_;
  }

 protected:
  void measure(const std::list<REPORT*>& new_reports) override;

  void startTrack(int i, int i1, MDL_STATE* state, MDL_REPORT* report) override;

  void continueTrack(int i, int i1, MDL_STATE* state, MDL_REPORT* report) override;

  void skipTrack(int i, int i1, MDL_STATE* state) override;

  void endTrack(int i, int i1) override;

  void falseAlarm(int i, MDL_REPORT* report) override;

 private:
  Track* findTrack(int id);

  void verify(int track_id,
              double state_x,
              double state_y,
              double report_x,
              double report_y,
              double likelihood,
              int frame,
              size_t corner_id);

  double false_alarm_log_likelihood_;
  std::list<Track> tracks_;
  std::list<FalseAlarm> false_alarms_;
};

}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker

#endif  // LASER_OBJECT_TRACKER_TRACKING_MHT_MOTION_MODEL_HPP
