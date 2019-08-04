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
#include <mht/corner.h>

#include "laser_object_tracker/feature_extraction/features/object.hpp"

namespace mht {
namespace internal {
extern int g_time;
}  // namespace internal
}  // namespace mht

namespace laser_object_tracker {
namespace tracking {
namespace mht {
cv::KalmanFilter buildKalmanFilter(int state_dimensions,
                                   int measurement_dimensions,
                                   const Eigen::MatrixXd& transition_matrix,
                                   const Eigen::MatrixXd& measurement_matrix,
                                   const Eigen::MatrixXd& measurement_noise_covariance,
                                   const Eigen::MatrixXd& initial_state_covariance,
                                   const Eigen::MatrixXd& process_noise_covariance);

cv::KalmanFilter copyKalmanFilter(const cv::KalmanFilter& kalman_filter);

double calculateMahalanobisDistance(const cv::KalmanFilter& kalman_filter,
                                    const cv::Mat& measurement);

double calculateLogLikelihood(const cv::KalmanFilter& kalman_filter,
                              double mahalanobis_distance);

class ObjectReport : public MDL_REPORT {
 public:
  ObjectReport(double false_alarm_log_likelihood,
               const feature_extraction::features::Object& object,
               int frame_number,
               size_t corner_id)
      : false_alarm_log_likelihood_(false_alarm_log_likelihood),
        object_(object),
        frame_number_(frame_number),
        corner_id_(corner_id) {}

  double getFalseAlarmLogLikelihood() const {
    return false_alarm_log_likelihood_;
  }

  const feature_extraction::features::Object& getObject() const {
    return object_;
  }

  int getFrameNumber() const {
    return frame_number_;
  }

  size_t getCornerId() const {
    return corner_id_;
  }

 private:
  double false_alarm_log_likelihood_;

  feature_extraction::features::Object object_;

  int frame_number_;
  size_t corner_id_;
};

class ObjectState : public MDL_STATE {
 public:
  static constexpr int STATE_DIMENSION = 4;
  static constexpr int MEASUREMENT_DIMENSION = 2;

  using State = Eigen::Matrix<double, STATE_DIMENSION, 1>;
  using Measurement = Eigen::Matrix<double, MEASUREMENT_DIMENSION, 1>;

  using StateTransition = Eigen::Matrix<double, STATE_DIMENSION, STATE_DIMENSION>;
  using MeasurementMatrix = Eigen::Matrix<double, MEASUREMENT_DIMENSION, STATE_DIMENSION>;

  using MeasurementNoiseCovariance = Eigen::Matrix<double, MEASUREMENT_DIMENSION, MEASUREMENT_DIMENSION>;
  using InitialStateCovariance = Eigen::Matrix<double, STATE_DIMENSION, STATE_DIMENSION>;
  using ProcessNoiseCovariance = Eigen::Matrix<double, STATE_DIMENSION, STATE_DIMENSION>;

  ObjectState(MODEL* model,
              double time_step,
              double log_likelihood,
              int times_skipped,
              const cv::KalmanFilter& kalman_filter,
              const State& state)
    : MDL_STATE(model),
      time_step_(time_step),
      log_likelihood_(log_likelihood),
      times_skipped_(times_skipped),
      kalman_filter_(copyKalmanFilter(kalman_filter)) {
    cv::eigen2cv(state, kalman_filter_.statePre);
    cv::eigen2cv(state, kalman_filter_.statePost);
  }

  ObjectState(MODEL* model,
              double time_step,
              double log_likelihood,
              int times_skipped,
              const cv::KalmanFilter& kalman_filter)
      : MDL_STATE(model),
        time_step_(time_step),
        log_likelihood_(log_likelihood),
        times_skipped_(times_skipped),
        kalman_filter_(copyKalmanFilter(kalman_filter)) {}

  ObjectState(MODEL* model,
              double time_step,
              double log_likelihood,
              int times_skipped,
              cv::KalmanFilter&& kalman_filter,
              const State& state)
    : MDL_STATE(model),
      time_step_(time_step),
      log_likelihood_(log_likelihood),
      times_skipped_(times_skipped),
      kalman_filter_(std::move(kalman_filter)) {
    cv::eigen2cv(state, kalman_filter_.statePre);
    cv::eigen2cv(state, kalman_filter_.statePost);
  }

  ObjectState(const ObjectState& other)
    : MDL_STATE(other.getMdl()),
      time_step_(other.time_step_),
      log_likelihood_(other.log_likelihood_),
      times_skipped_(other.times_skipped_),
      kalman_filter_(copyKalmanFilter(other.kalman_filter_)) {}

  double getLogLikelihood() override {
    return log_likelihood_;
  }

  void setLogLikelihood(double log_likelihood) {
    log_likelihood_ = log_likelihood;
  }

  void predict() {
    kalman_filter_.predict();
  }

  void update(const Measurement& measurement) {
    cv::Mat cv_measurement;
    cv::eigen2cv(measurement, cv_measurement);
    kalman_filter_.correct(cv_measurement);
  }

  void incrementTimesSkipped() {
    ++times_skipped_;
  }

  void resetTimesSkipped() {
    times_skipped_ = 0;
  }

  int getTimesSkipped() const {
    return times_skipped_;
  }

  double getX() const {
    return kalman_filter_.statePost.at<double>(0);
  }

  double getY() const {
    return kalman_filter_.statePost.at<double>(1);
  }

  double getVelocityX() const {
    return kalman_filter_.statePost.at<double>(2);
  }

  double getVelocityY() const {
    return kalman_filter_.statePost.at<double>(3);
  }

  const cv::KalmanFilter& getKalmanFilter() const {
    return kalman_filter_;
  }

 private:
  double time_step_;

  double log_likelihood_;

  int times_skipped_;

  cv::KalmanFilter kalman_filter_;
};

class ObjectModel : public MODEL {
 public:
  ObjectModel(double time_step,
              double max_mahalanobis_distance,
              double skip_decay_rate,
              double start_likelihood,
              double skip_likelihood,
              double detect_likelihood,
              const ObjectState::MeasurementNoiseCovariance& measurement_noise_covariance,
              const ObjectState::InitialStateCovariance& initial_state_covariance,
              const ObjectState::ProcessNoiseCovariance& process_noise_covariance)
      : MODEL(),
        time_step_(time_step),
        max_mahalanobis_distance_(max_mahalanobis_distance),
        skip_decay_rate_(skip_decay_rate),
        start_log_likelihood_(std::log(start_likelihood)),
        skip_log_likelihood_(std::log(skip_likelihood)),
        detect_log_likelihood_(std::log(detect_likelihood)),
        measurement_noise_covariance_(measurement_noise_covariance),
        initial_state_covariance_(initial_state_covariance),
        process_noise_covariance_(process_noise_covariance) {
    state_transition_ << 1.0, 0.0, time_step_, 0.0,
                         0.0, 1.0, 0.0, time_step_,
                         0.0, 0.0, 1.0, 0.0,
                         0.0, 0.0, 0.0, 1.0;

    measurement_matrix_ << 1.0, 0.0, 0.0, 0.0,
                           0.0, 1.0, 0.0, 0.0;
  }

  int beginNewStates(MDL_STATE *state, MDL_REPORT *report) override {
    return 1;
  }

  double getEndLogLikelihood(MDL_STATE *state) override {
    auto object_state = dynamic_cast<ObjectState&>(*state);
    return std::log(getEndProbability(object_state));
  }

  double getContinueLogLikelihood(MDL_STATE *state) override {
    auto object_state = dynamic_cast<ObjectState&>(*state);
    return std::log(1.0 - getEndProbability(object_state));
  }

  double getSkipLogLikelihood(MDL_STATE *state) override {
    return skip_log_likelihood_;
  }

  double getDetectLogLikelihood(MDL_STATE *state) override {
    return detect_log_likelihood_;
  }

  MDL_STATE *getNewState(int i, MDL_STATE *state, MDL_REPORT *report) override;

 private:
  double getEndProbability(const ObjectState& state) const;
  double mahalanobisDistance(const ObjectState& state,
                             const ObjectReport& report) const;
  cv::KalmanFilter getKalmanFilter() const;

  double time_step_;
  double max_mahalanobis_distance_;

  // The higher rate, the slower decay
  double skip_decay_rate_;

  double start_log_likelihood_;
  double skip_log_likelihood_;
  double detect_log_likelihood_;

  ObjectState::MeasurementNoiseCovariance  measurement_noise_covariance_;
  ObjectState::InitialStateCovariance initial_state_covariance_;
  ObjectState::ProcessNoiseCovariance process_noise_covariance_;

  ObjectState::StateTransition state_transition_;
  ObjectState::MeasurementMatrix measurement_matrix_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ObjectFalseAlarm : public DLISTnode {
 public:
  explicit ObjectFalseAlarm(const ObjectReport* report)
      : DLISTnode(),
        x_(report->getObject().getReferencePoint().x()),
        y_(report->getObject().getReferencePoint().x()),
        frame_number_(report->getFrameNumber()),
        corner_id_(report->getCornerId()) {}

 protected:
  MEMBERS_FOR_DLISTnode(ObjectFalseAlarm)

 private:
  double x_, y_;
  int frame_number_;
  size_t corner_id_;
};

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

class TrackElement : public DLISTnode {
 public:
  TrackElement(double state_x,
               double state_y,
               double report_x,
               double report_y,
               double log_likelihood,
               int frame_number,
               size_t corner_id)
      : state_x_(state_x),
        state_y_(state_y),
        report_x_(report_x),
        report_y_(report_y),
        log_likelihood_(log_likelihood),
        frame_number_(frame_number),
        corner_id_(corner_id) {
    has_report_ = !(std::isnan(report_x) || std::isnan(report_y));
  }

  size_t getCornerId() const {
    return corner_id_;
  }

  double getStateX() const {
    return state_x_;
  }

  double getStateY() const {
    return state_y_;
  }

  double getReportX() const {
    return report_x_;
  }

  double getReportY() const {
    return report_y_;
  }

 protected:
  MEMBERS_FOR_DLISTnode(TrackElement)

 private:
  bool has_report_;

  double state_x_, state_y_,
         report_x_, report_y_;

  double log_likelihood_;

  int frame_number_;
  size_t corner_id_;
};

class Track : public DLISTnode {
 public:
  explicit Track(int id) : id_(id), track_() {}

  int getId() const {
    return id_;
  }

  std::list<TrackElement> track_;

 protected:
  MEMBERS_FOR_DLISTnode(Track)

 private:
  int id_;
};

class ObjectTracker : public MDL_MHT {
 public:
  ObjectTracker(double false_alarm_likelihood,
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

  const std::list<ObjectFalseAlarm>& getFalseAlarms() const {
    return false_alarms_;
  }

 protected:
  void measure(const std::list<CORNER>& new_reports) override;

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
  std::list<ObjectFalseAlarm> false_alarms_;
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
  void measure(const std::list<CORNER>& new_reports) override;

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


//class CONSTPOS_REPORT: public MDL_REPORT
//{
//  friend class CONSTVEL_STATE;
//  friend class CONSTVEL_MDL;
//
//
// private:
//
//  double m_falarmLogLikelihood;    // log of the likelihood that
//  // this report is a false alarm
//  // (not really part of a CORNER_TRACK)
//  MATRIX m_z;                      // (x, dx, y, dy)
//
// public:
//  int m_frameNo;
//  size_t m_cornerID;
//  CONSTPOS_REPORT( const double &falarmLogLikelihood,
//                   const double &x, const double &y,
//                   const int &f, const size_t &cornerID):
//      MDL_REPORT(),
//      m_falarmLogLikelihood( falarmLogLikelihood ),
//      m_z( 2, 1 ),
//      m_frameNo(f),
//      m_cornerID(cornerID)
//
//  {
//    m_z.set( x, y);
//  }
//
//  CONSTPOS_REPORT( const CONSTPOS_REPORT &src ):
//      MDL_REPORT(),
//      m_falarmLogLikelihood( src.m_falarmLogLikelihood ),
//      m_z( src.m_z ),
//      m_frameNo(src.m_frameNo),
//      m_cornerID(src.m_cornerID)
//  {
//  }
//
//  virtual void describe(int spaces)
//  {
//    m_z.print();
//  }
//
//  virtual void print()
//  {
//    std::cout << "  " <<m_z(0) << " " << m_z(1);
//  }
//
//  virtual double getFalarmLogLikelihood()
//  {
//    return m_falarmLogLikelihood;
//  }
//
//  MATRIX &getZ()
//  {
//    return m_z;
//  }
//
//  double getX()
//  {
//    return m_z( 0 );
//  }
//  double getY()
//  {
//    return m_z( 1 );
//  }
//  void printMeas()
//  {
//    printf("%lf %lf frame=%d\n",m_z(0),m_z(1),m_frameNo);
//  }
//};
//
//
//
///*-------------------------------------------------------------------*
// | CORNER_TRACK_MDL -- model of CORNER_TRACKs
// *-------------------------------------------------------------------*/
//
//class CORNER_TRACK_MDL:public MODEL
//{
// public:
//  int type;
//  virtual double getStateX(MDL_STATE*)
//  {
//    return 0;
//  }
//
//  virtual double getStateY(MDL_STATE*)
//  {
//    return 0;
//  }
//};
//
//
///*-------------------------------------------------------------------*
// *
// * CONSTVEL_MDL -- model for cornerTracks
// *
// * CONSTVEL_MDL must be derived from MODEL
// *
// * CONSTVEL_MDL describes the user's model of a CORNER_TRACK
// * Here the model is implemented as a simple linear Kalman filter
// *
// *-------------------------------------------------------------------*/
//
//
//class CONSTVEL_MDL: public CORNER_TRACK_MDL
//{
// private:
//  double m_lambda_x;
//  double m_startLogLikelihood;     // likelihood of a CORNER_TRACK starting
//  double m_endLogLikelihood;       // likelihood of a CORNER_TRACK ending
//  double m_continueLogLikelihood;  // likelihood of a CORNER_TRACK not
//  //   ending
//  double m_skipLogLikelihood;      // likelihood of not detecting a
//  //   CORNER_TRACK that hasn't ended
//  double m_detectLogLikelihood;    // likelihood of detecting a
//  //   CORNER_TRACK that hasn't ended
//
//  double m_maxDistance;            // maximum mahalanobis distance
//  //   allowed for validating a
//  //   report to a CORNER_TRACK
//
//  double m_processVariance;        // process noise
//  double m_intensityVariance;
//  double m_stateVariance;
//  MATRIX m_R;                      // measurement covariance
//  MATRIX m_startP;                 // covariance matrix to use at
//  //   start of a CORNER_TRACK
// public:
//
//  CONSTVEL_MDL( double positionMeasureVarianceX,
//                double positionMeasureVarianceY,
//                double gradientMeasureVariance,
//                double intensityVariance,
//                double processVariance,
//                double startProb,
//                double lambda_x,
//                double detectProb,
//                double stateVar,
//                double maxDistance);
//
//  virtual int beginNewStates( MDL_STATE *mdlState,
//                              MDL_REPORT *mdlReport );
//  virtual MDL_STATE *getNewState( int stateNum,
//                                  MDL_STATE *mdlState,
//                                  MDL_REPORT *mdlReport );
//  virtual double getEndLogLikelihood( MDL_STATE * );
//  virtual double getContinueLogLikelihood( MDL_STATE * );
//  virtual double getSkipLogLikelihood( MDL_STATE *mdlState );
//  virtual double getDetectLogLikelihood( MDL_STATE * )
//  {
//    return m_detectLogLikelihood;
//  }
//  virtual double getStateX(MDL_STATE *s);
//  virtual double getStateY(MDL_STATE *s);
// private:
//
//  CONSTVEL_STATE* getNextState( CONSTVEL_STATE *state,
//                                CONSTPOS_REPORT *report );
//};
//
///*-------------------------------------------------------------------*
// *
// * CONSTVEL_STATE -- state estimate for a CORNER_TRACK
// *
// * CONSTVEL_STATE hold the state information for the CORNER_TRACK
// * E.g. the vector (x, dx, y, dy)
// *
// *-------------------------------------------------------------------*/
//
//class CONSTVEL_STATE: public MDL_STATE
//{
//  friend class ONSTPOS_REPORT;
//  friend class CONSTCURV_MDL;
//  friend class CONSTVEL_MDL;
//  friend class CONSTPOS_MDL;
//
// private:
//
//  MATRIX m_x;                      // state estimate (x, dx, y, dy)
//  MATRIX m_P;                      // covariance matrix
//  double m_logLikelihood;          // likelihood that this state
//  //   is the true state of the
//  //   CORNER_TRACK after the state
//  //   that it was born from (in
//  //   CONSTVEL_MDL::getNewState())
//
//  int m_numSkipped;
//  int m_hasBeenSetup;              // 0 before the following variables
//  //   have been filled in, 1 after
//
//  double m_ds;                     // "time" step until the next state
//  //   (chosen so that the next state
//  //   lands in a neighboring pixel)
//  double m_logLikelihoodCoef;      // part of likelihood calculation
//  //   that's independent of the
//  //   inovation
//  MATRIX *m_Sinv;                  // inverse of the innovation
//  //   covariance
//  MATRIX *m_W;                     // filter gain
//  MATRIX *m_nextP;                 // updated state covariance
//  //   (covariance for next state)
//  MATRIX *m_x1;                    // state prediction
//
// private:
//
//  CONSTVEL_STATE( CONSTVEL_MDL *mdl,
//                  const double &x,
//                  const double &dx,
//                  const double &y,
//                  const double &dy,
//                  MATRIX &P,
//                  const double &logLikelihood,
//                  const int &numSkipped):
//      MDL_STATE( mdl ),
//      m_logLikelihood( logLikelihood ),
//      m_hasBeenSetup( 0 ),
//      m_numSkipped(numSkipped),
//      m_x(4,1),
//      m_P(P),
//      m_ds( 0 ),
//      m_x1( 0 ),
//      m_nextP( 0 ),
//      m_Sinv( 0 ),
//      m_W( 0 )
//  {
//    m_x(0)=x;
//    m_x(1)=dx;
//    m_x(2)=y;
//    m_x(3)=dy;
//  }
//
//
//  CONSTVEL_STATE( const CONSTVEL_STATE &src ):
//      MDL_STATE( src.getMdl() ),
//      m_x( src.m_x ),
//      m_P( src.m_P ),
//      m_logLikelihood( src.m_logLikelihood ),
//      m_hasBeenSetup( 0 ),
//      m_numSkipped(src.m_numSkipped),
//      m_ds( 0 ),
//      m_x1( 0 ),
//      m_nextP( 0 ),
//      m_Sinv( 0 ),
//      m_W( 0 )
//  {
//  }
//
//
// private:
//
//  void setup( double processVariance, const MATRIX &R );
//
//  void cleanup()
//  {
//
//
//    if( m_hasBeenSetup )
//    {
//      delete m_x1;
//      m_x1 = 0;
//      delete m_nextP;
//      m_nextP = 0;
//      delete m_Sinv;
//      m_Sinv = 0;
//      delete m_W;
//      m_W = 0;
//
//      m_hasBeenSetup = 0;
//    }
//  }
//
//
//  int getNumSkipped()
//  {
//    return m_numSkipped;
//  }
//  double getLogLikelihoodCoef()
//  {
//    checkSetup();
//    return m_logLikelihoodCoef;
//  }
//  MATRIX &getPrediction()
//  {
//    checkSetup();
//    return *m_x1;
//  }
//  MATRIX &getNextP()
//  {
//    checkSetup();
//    return *m_nextP;
//  }
//  MATRIX &getSinv()
//  {
//    checkSetup();
//    return *m_Sinv;
//  }
//  MATRIX &getW()
//  {
//    checkSetup();
//    return *m_W;
//  }
//
//#ifdef TSTBUG
//
//  void checkSetup()
//    {
//        assert( m_hasBeenSetup );
//        //  THROW_ERR( "Trying to get derived info from a CONSTPOS state"
//        //             " that hasn't been setup()" )
//    }
//
//#else
//  void checkSetup() {}
//#endif
//
// public:
//
//  ~CONSTVEL_STATE()
//  {
//    cleanup();    //SHOULD THIS BE PRIVATE?
//  }
//  virtual double getLogLikelihood()
//  {
//    return m_logLikelihood;
//  }
//
//  virtual void print()
//  {
//    std::cout << "ConstVel State: "<< m_x(0) << " ,"
//              <<m_x(2);
//  }
//  double getX()
//  {
//    return m_x( 0 );
//  }
//  double getDX()
//  {
//    return m_x( 1 );
//  }
//  double getY()
//  {
//    return m_x( 2 );
//  }
//  double getDY()
//  {
//    return m_x( 3 );
//  }
//
//  void setDX(double val)
//  {
//    m_x( 1 )=val;
//  }
//  void setDY(double val)
//  {
//    m_x( 3 )=val;
//  }
//
//  double getX1()
//  {
//    checkSetup();
//    return (*m_x1)( 0 );
//  }
//  double getDX1()
//  {
//    checkSetup();
//    return (*m_x1)( 1 );
//  }
//  double getY1()
//  {
//    checkSetup();
//    return (*m_x1)( 2 );
//  }
//  double getDY1()
//  {
//    checkSetup();
//    return (*m_x1)( 3 );
//  }
//
//  double getDS()
//  {
//    checkSetup();
//    return m_ds;
//  }
//};
//
///*-------------------------------------------------------------------*
// *
// * FALARM -- is a structure for holding flase alarms on an
// *           "intrusive" list.
// *
// * For the definition of an intrusive list see the file list.h
// *
// *-------------------------------------------------------------------*/
//
//class FALARM: public DLISTnode
//{
// public:
//  double rX, rY;
//  int frameNo;
//  size_t cornerID;
//
//  FALARM( CONSTPOS_REPORT *xreport ):
//      DLISTnode(),
//      rX( xreport->getX() ),
//      rY( xreport->getY() ),
//      frameNo( xreport->m_frameNo ),
//      cornerID( xreport->m_cornerID )
//  {
//  }
//
// protected:
//
//  MEMBERS_FOR_DLISTnode( FALARM )
//};
//
//
///*-------------------------------------------------------------------*
// *
// * CORNER_TRACK_ELEMENT -- is a structure for holding an individual
// * element of a CORNER_TRACK on an "intrusive" doubly linked list
// *
// * For the definition of an intrusive list see the file list.h
// *
// *-------------------------------------------------------------------*/
//
//
//class CORNER_TRACK_ELEMENT: public DLISTnode
//{
// public:
//  int hasReport;
//  double sx,sy;
//  double rx,ry;
//  int frameNo;
//  int time;
//  double logLikelihood;
//  char   model[30];
//  size_t cornerID;
//
//  CORNER_TRACK_ELEMENT(double s_x, double s_y, double r_x, double r_y, double prob, int type,int t,int f,size_t id):
//      DLISTnode(),
//      sx(s_x),sy(s_y),rx(r_x),ry(r_y),logLikelihood(prob),time(t),frameNo(f),cornerID(id)
//  {
//    if (!isnan(r_x) && !isnan(r_y))
//    {
//      hasReport=1;
//    }
//    else
//    {
//      hasReport=0;
//    }
//
//    switch(type)
//    {
//      case 1:
//        sprintf(model,"CONSTANT MODEL");
//        break;
//      case 2:
//        sprintf(model,"CONSTANT VELOCITY");
//        break;
//      case 3:
//        sprintf(model,"CONSTANT CURV");
//        break;
//    }
//  }
//
// protected:
//
//  MEMBERS_FOR_DLISTnode( CORNER_TRACK_ELEMENT )
//};
//
///*-------------------------------------------------------------------*
// *
// * CORNER_TRACK -- is a structure to save info about a CORNER_TRACK
// * on an "intrusive" doubly linked list
// *
// * For the definition of an intrusive list see the file list.h
// *
// *-------------------------------------------------------------------*/
//
//class CORNER_TRACK: public DLISTnode
//{
// public:
//  int id;
//  int color;
//
//  std::list< CORNER_TRACK_ELEMENT > list;
//
//  CORNER_TRACK( int idArg, int colorArg ):
//      DLISTnode(),
//      id( idArg ),
//      color( colorArg ),
//      list()
//  {
//  }
//
// protected:
//
//  MEMBERS_FOR_DLISTnode( CORNER_TRACK )
//};
//
//
//
///*-------------------------------------------------------------------*
// *
// * CORNER_TRACK_MHT -- MDL_MHT class for CORNER_TRACK
// *
// *-------------------------------------------------------------------*/
//
//class CORNER_TRACK_MHT: public MDL_MHT
//{
// public:
//
//  CORNER_TRACK_MHT( double fprob,int maxDepth, double minGHypoRatio, int maxGHypos,
//                    ptrDLIST_OF<MODEL> mdlist ):
//      MDL_MHT( maxDepth, minGHypoRatio, maxGHypos ),
//      m_falarmLogLikelihood( log(fprob) ),
//      m_cornerTracks(),
//      m_falarms()
//  {
//    m_modelList.appendCopy( mdlist );
//  }
//  virtual void describe(int spaces=0);
//  virtual std::list< CORNER_TRACK > GetTracks() const
//  {
//    return m_cornerTracks;
//  }
//  virtual std::list< FALARM > GetFalseAlarms() const
//  {
//    return m_falarms;
//  }
//
// private:
//  double m_falarmLogLikelihood;
//  std::list< CORNER_TRACK > m_cornerTracks;
//  std::list< FALARM > m_falarms;
//
// protected:
//
//  virtual void measure(const std::list<CORNER> &newReports);
//
//  virtual void startTrack( int trackId, int,
//                           MDL_STATE *state, MDL_REPORT *report )
//  {
//    g_numTracks++;
//    CONSTPOS_REPORT* r= (CONSTPOS_REPORT*)report;
//
//    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
////      printf("Calling Verify in statrtTRack\n");
//    verify( trackId, r->getX(),
//            r->getY(),
//            mdl->getStateX(state),
//            mdl->getStateY(state),
//            state->getLogLikelihood(),
//            mdl->type,r->m_frameNo,r->m_cornerID);
//  }
//
//  virtual void continueTrack( int trackId, int,
//                              MDL_STATE *state, MDL_REPORT *report )
//  {
//    CONSTPOS_REPORT* r= (CONSTPOS_REPORT*)report;
//    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
////      printf("Calling Verify in continueTRack\n");
//    verify( trackId, r->getX(),
//            r->getY(),
//            mdl->getStateX(state),
//            mdl->getStateY(state),
//            state->getLogLikelihood(),
//            mdl->type,r->m_frameNo,r->m_cornerID);
//  }
//
//  virtual void skipTrack( int trackId, int, MDL_STATE *state )
//  {
//    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
////      printf("Calling Verify in skipTRack\n");
//    verify( trackId, NAN, NAN, mdl->getStateX(state), mdl->getStateY(state),
//            state->getLogLikelihood(),
//            mdl->type,-9,0);
//  }
//
//  virtual void endTrack( int, int )
//  {
////      printf("Verifying endTrack\n");
//    g_numTracks--;
//  }
//
//  virtual void falseAlarm( int, MDL_REPORT *report )
//  {
//    saveFalarm( (CONSTPOS_REPORT*)report );
//  }
//
// private:
//  CORNER_TRACK* findTrack( const int &id );
//  void saveFalarm( CONSTPOS_REPORT *report );
//  void verify( int trackID, double r_x, double r_y, double s_x,
//               double s_y, double likelihood, int modelType, int frame, size_t cornerID);
//};
//
//int getTrackColor( int trackId );

#endif  // LASER_OBJECT_TRACKER_TRACKING_MHT_MOTION_MODEL_HPP
