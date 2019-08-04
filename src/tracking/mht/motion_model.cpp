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

MDL_STATE *ObjectModel::getNewState(int i, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  ObjectState* next_state = nullptr;
  if (object_state == nullptr) {
    ObjectState::State state_vector;
    state_vector.setZero();
    state_vector.head<2>() = object_report->getObject().getReferencePoint();
    next_state = new ObjectState(this,
                                 time_step_,
                                 start_log_likelihood_,
                                 0,
                                 std::move(getKalmanFilter()),
                                 state_vector);
  } else if (report == nullptr) {
    object_state->predict();
    object_state->incrementTimesSkipped();

    next_state = new ObjectState(*object_state);
    next_state->setLogLikelihood(0.0);
  } else {
    object_state->predict();

    double mahalanobis_distance = mahalanobisDistance(*object_state, *object_report);
    if (mahalanobis_distance <= max_mahalanobis_distance_) {
      double log_likelihood = calculateLogLikelihood(object_state->getKalmanFilter(), mahalanobis_distance);

      ObjectState::Measurement measurement = object_report->getObject().getCorners().front().getCorner();
      object_state->update(measurement);

      next_state = new ObjectState(this,
                                   time_step_,
                                   log_likelihood,
                                   0,
                                   object_state->getKalmanFilter());
    }
  }

  return next_state;
}

double ObjectModel::getEndProbability(const ObjectState& state) const {
  double end_probability = 1.0 - std::exp(-state.getTimesSkipped() / skip_decay_rate_);
  // End probability cannot be 0.0
  return std::nextafter(end_probability, 1.0);
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
  // TODO Track initiation
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

void ObjectTracker::measure(const std::list<CORNER>& new_reports) {
//  auto* object = new ObjectReport(false_alarm_log_likelihood_,)
  for (const auto& report : new_reports) {
    feature_extraction::features::Object object;
    object.setReferencePoint(feature_extraction::features::Point2D(report.x, report.y));
    installReport(new ObjectReport(false_alarm_log_likelihood_,
                                          object,
                                          report.m_frameNo,
                                          report.m_cornerID));
  }
}

void ObjectTracker::startTrack(int i, int i1, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  verify(i,
         object_report->getObject().getReferencePoint().x(),
         object_report->getObject().getReferencePoint().y(),
         object_state->getX(),
         object_state->getY(),
         object_state->getLogLikelihood(),
         object_report->getFrameNumber(),
         object_report->getCornerId());
}

void ObjectTracker::continueTrack(int i, int i1, MDL_STATE *state, MDL_REPORT *report) {
  auto object_state = dynamic_cast<ObjectState*>(state);
  auto object_report = dynamic_cast<ObjectReport*>(report);

  verify(i,
         object_report->getObject().getReferencePoint().x(),
         object_report->getObject().getReferencePoint().y(),
         object_state->getX(),
         object_state->getY(),
         object_state->getLogLikelihood(),
         object_report->getFrameNumber(),
         object_report->getCornerId());
}

void ObjectTracker::skipTrack(int i, int i1, MDL_STATE *state) {
  auto object_state = dynamic_cast<ObjectState*>(state);

  double nan = std::numeric_limits<double>::quiet_NaN();
  verify(i,
         nan,
         nan,
         object_state->getX(),
         object_state->getY(),
         object_state->getLogLikelihood(),
         -1,
         0);
}

void ObjectTracker::endTrack(int i, int i1) {
  tracks_.remove_if([i](const auto& track) { return track.getId() == i;});
}

void ObjectTracker::falseAlarm(int i, MDL_REPORT *report) {
  false_alarms_.emplace_back((dynamic_cast<ObjectReport*>(report)));
}

Track *ObjectTracker::findTrack(int id) {
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

void ObjectTracker::verify(int track_id,
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
                             report_x,
                             report_y,
                             likelihood,
                             frame,
                             corner_id);
}

void MHTTracker::measure(const std::list<CORNER>& new_reports) {
  for (const auto& report : new_reports) {
    installReport(new PositionReport(false_alarm_log_likelihood_,
                                     report.x,
                                     report.y,
                                     report.m_frameNo,
                                     report.m_cornerID));
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
                             report_x,
                             report_y,
                             likelihood,
                             frame,
                             corner_id);
}
}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker

/*------------------------------------------------------*
 * findTrack():  look for the track with given id in the
 * cornerTrackList and return a ptr to it.  If
 * not found create one
 *------------------------------------------------------*/

//CORNER_TRACK *CORNER_TRACK_MHT::findTrack( const int &id )
//{
//
//  for (std::list<CORNER_TRACK>::iterator p = m_cornerTracks.begin();
//       p != m_cornerTracks.end();
//       p++)
//  {
//    if( p->id == id )
//    {
//      return &(*p);
//    }
//  }
//
//  m_cornerTracks.push_back( CORNER_TRACK( id, getTrackColor(id) ) );
//
//  return &(m_cornerTracks.back());
//}
//
///*-------------------------------------------------------------------*
// * saveFalarm(report): saves the given report in the FALARM list
// *-------------------------------------------------------------------*/
//void CORNER_TRACK_MHT::saveFalarm( CONSTPOS_REPORT *report )
//{
//  m_falarms.push_back( FALARM( report ) );
//}
//
//
///*-------------------------------------------------------------------*
// * verify(trackId, report, state) : find the CORNER_TRACK with the given
// *                    id, create a new CORNER_TRACK_ELEMENT with the
// *                    given report & state and append it to the
// *                    list of CORNER_TRACK_ELEMENTs of that CORNER_TRACK
// *-------------------------------------------------------------------*/
//void CORNER_TRACK_MHT::verify( int trackId, double r_x, double r_y, double s_x, double s_y,
//                               double logLikelihood,
//                               int modelType, int frame, size_t id)
//{
//  CORNER_TRACK *track;
//
////  printf("Verifying trackId=%d r_x=%lf r_y=%lf s_x=%lf s_y=%lf frame=%d\n",
////			trackId,r_x,r_y,s_x,s_y,frame);
//  track = findTrack( trackId );
//  track->list.push_back( CORNER_TRACK_ELEMENT( s_x,s_y,r_x,r_y,logLikelihood,modelType,mht::internal::g_time,frame,id));
//}
//
///**-------------------------------------------------------------------
// * void CORNER_TRACK_MHT::measure()
// * Take the corners of the current frame and install
// * them as reports
// *-------------------------------------------------------------------*/
//
//void CORNER_TRACK_MHT::measure(const std::list<CORNER> &newReports)
//{
//  for (std::list<CORNER>::const_iterator cornerPtr = newReports.begin();
//       cornerPtr != newReports.end();
//       cornerPtr++)
//  {
//    installReport(new CONSTPOS_REPORT(m_falarmLogLikelihood,
//                                      cornerPtr->x, cornerPtr->y,
//                                      cornerPtr->m_frameNo,cornerPtr->m_cornerID)
//    );
//  }
//
//}
//
///*-------------------------------------------------------------------*
// | LOG_NORMFACTOR -- constant part of likelihood calculation
// *-------------------------------------------------------------------*/
//
//static const double LOG_NORMFACTOR =
//    /* log( 2PI^(measureVars/2) ) = */ 1.5963597;
//
//
///*-------------------------------------------------------------------*
// | CORNER_TRACK_STATE::setup() -- compute parts of Kalman filter
// |                           calculation that are independent of
// |                           reports
// *-------------------------------------------------------------------*/
//
//void CONSTVEL_STATE::setup( double processVariance, const MATRIX &R )
//{
//
//
//  /* don't do this more than once */
//  if( m_hasBeenSetup )
//  {
//    return;
//  }
//
//  m_ds = 1;
//
//  /* compute the state transition matrix and process covariance matrix
//     based on the above time step */
//
//  double ds2 = m_ds * m_ds;
//  double ds3 = ds2 * m_ds;
//
//  static MATRIX F( 4, 4 );
//  F.set(     1.,  m_ds,    0.,    0.,
//             0.,    1.,    0.,    0.,
//             0.,    0.,    1.,  m_ds,
//             0.,    0.,    0.,    1.    );
//
//
//  static MATRIX Q( 4, 4 );
//  Q.set(  ds3/3, ds2/2,    0.,    0.,
//          ds2/2,  m_ds,    0.,    0.,
//          0.,    0., ds3/3, ds2/2,
//          0.,    0., ds2/2,  m_ds  );
//  Q = Q * processVariance;
//
//  static MATRIX H(2,4);
//  H.set(1., 0., 0., 0.,
//        0., 0., 1., 0.);
//
//
//  /* fill in the rest of the variables */
//
//  MATRIX P1 = F * m_P * F.trans() + Q; // state prediction covariance
//
//  MATRIX S = H * P1 * H.trans() + R;  // innovation covariance
//
//  m_logLikelihoodCoef = -(LOG_NORMFACTOR + log( S.det() ) / 2);
//
//  m_Sinv = new MATRIX( S.inv() );
////  printf("Sinv:\n"); m_Sinv->print();
//
//  m_W = new MATRIX( P1 * H.trans() * *m_Sinv );
//
//  MATRIX tmp(4,4);
//  tmp =  *m_W * S * m_W->trans();
//
//  MATRIX tmp1(4,4);
//  tmp1 = P1-tmp;
//
//  m_nextP = new MATRIX( tmp1 );
//  m_x1 = new MATRIX( F * m_x );
//
//  m_hasBeenSetup = 1;
//
//#ifdef DEBUG1
//  printf("\nF:\n");
//    F.print();
//    printf("\nm_P:\n");
//    m_P.print();
//    printf("\nQ=\n");
//    Q.print();
//    printf("\nState Pred Cov(P1=F*m_P*F.trans +Q):\n");
//    P1.print();
//    printf("\nInnov Cov(S=H*P1*H.trans):\n");
//    S.print();
//    printf("\nS_inv:\n");
//    m_Sinv->print();
//    printf("\nPrevious State:\n");
//    m_x.print();
//    printf("LOG_NORMFACTOR =%lf log( S.det() ) / 2)=%lf\n",LOG_NORMFACTOR, log( S.det() ) / 2);
//    printf(" m_logLikelihoodCoef= %lf\n", m_logLikelihoodCoef);
//#endif
//
//}
//
///*--------------------------------------------*
// * CONSTVEL_MDL::getStateX(MDL_STATE *s)
// *--------------------------------------------*/
//
//double CONSTVEL_MDL::getStateX(MDL_STATE *s)
//{
//  return ((CONSTVEL_STATE*)s)->getX();
//}
//
//
///*--------------------------------------------*
// * CONSTVEL_MDL::getStateY(MDL_STATE *s)
// *--------------------------------------------*/
//
//double CONSTVEL_MDL::getStateY(MDL_STATE *s)
//{
//  return ((CONSTVEL_STATE*)s)->getY();
//}
//
//
//double CONSTVEL_MDL::getEndLogLikelihood( MDL_STATE *s )
//{
//  CONSTVEL_STATE *cs = (CONSTVEL_STATE*)s;
//  int m = cs->m_numSkipped;
//  double endProb = 1.0 - exp( -m / m_lambda_x);
//  endProb += (endProb == 0.0) ? EPSILON : 0.0;
//  m_endLogLikelihood = log( endProb);
//  return m_endLogLikelihood;
//}
//double CONSTVEL_MDL::getContinueLogLikelihood( MDL_STATE *s )
//{
//  CONSTVEL_STATE *cs = (CONSTVEL_STATE*)s;
//  int m = cs->m_numSkipped;
//  double endProb = 1.0 - exp( -m / m_lambda_x);
//  endProb += (endProb == 0.0) ? EPSILON : 0.0;
//  m_continueLogLikelihood = log(1.0-endProb);
//  return m_continueLogLikelihood;
//}
//
//
//
///*-------------------------------------------------------------------*
// | CONSTVEL_MDL::getSkipLogLikelihood() -- get the likelihood of
// |                                        skipping a(nother) report
// *-------------------------------------------------------------------*/
//
//double CONSTVEL_MDL::getSkipLogLikelihood( MDL_STATE *mdlState )
//{
//  CONSTVEL_STATE *state = (CONSTVEL_STATE *)mdlState;
//
//  return  m_skipLogLikelihood;
//
////  return (state->getNumSkipped() + 1) * m_skipLogLikelihood;
//}
//
//
///*------------------------------------------------------------------*
// * MDL_STATE* CONSTVEL_MDL::getNewState(...)
// *------------------------------------------------------------------*/
//
//MDL_STATE* CONSTVEL_MDL::getNewState( int stateNum,
//                                      MDL_STATE *mdlState,
//                                      MDL_REPORT *mdlReport )
//{
//
//  CONSTVEL_STATE *state = (CONSTVEL_STATE *) mdlState;
//  CONSTPOS_REPORT *report = (CONSTPOS_REPORT *) mdlReport;
//  double dx,dy;
//
//  switch(stateNum)
//  {
//    case 0:   // Continue constVel State
//    {
//      if( state != 0 && report !=0 && state->getDX() == 0
//          && state->getDY() == 0 )
//      {
//        dx = report->getX() - state->getX();
//        dy = report->getY() - state->getY();
//
//        state->setDX(dx);
//        state->setDY(dy);
//      }
//
//      CONSTVEL_STATE *newState;
//      newState=getNextState(state,report);
//      return (MDL_STATE*) newState;
//    }
//    default:
//      assert(false);//("Too many calls to CONSTVEL_MDL::getNewState()");
//  }
//
//}
//
//
//
///*-------------------------------------------------------------------*
// | CONSTVEL_MDL::getNextState() -- get the next state estimate, given
// |                                a previous state estimate and a
// |                                reported measurement
// *-------------------------------------------------------------------*/
//
//CONSTVEL_STATE* CONSTVEL_MDL::getNextState( CONSTVEL_STATE *state,
//                                            CONSTPOS_REPORT *report )
//{
//  CONSTVEL_STATE *nextState;          // new state
//  static MATRIX v( 2,1 );              // innovation
//  double distance;                   // mahalanobis distance
//
//  static MATRIX H(2,4);
//  H.set(1., 0., 0., 0.,
//        0., 0., 1., 0.);
//
//
//  if( state == 0 )
//  {
//    /* starting a new track */
//
//    double x=report->getX();
//    double y=report->getY();
//
//#ifdef DEBUG1
//    printf("\nStart a new State/Contour with Cov:");
//        m_startP.print(2);
//        printf("\nSTARTING NEW STATE WITH %lf %lf\n",x,y);
//#endif
//
//    nextState = new CONSTVEL_STATE( this,
//                                    x,
//                                    0.,
//                                    y,
//                                    0.,
//                                    m_startP,
//                                    m_startLogLikelihood,
//                                    0 );
//  }
//  else if( report == 0 )
//  {
//    /* continuing an existing CORNER_TRACK, skipping a measurement */
//
//    state->setup( m_processVariance, m_R );
//
//#ifdef DEBUG1
//    printf("Skipping meas(report=0); continued state= %lf %lf %lf %lf\n",
//               state->getX1(), state->getDX1(),
//               state->getY1(), state->getDY1() );
//#endif
//
//    nextState = new CONSTVEL_STATE( this,
//                                    state->getX1(),
//                                    state->getDX1(),
//                                    state->getY1(),
//                                    state->getDY1(),
//                                    state->getNextP(),
//                                    0.,
//                                    state->getNumSkipped() + 1 );
//  }
//  else
//  {
//    /* continuing an existing CORNER_TRACK, with a measurement */
//
//    state->setup( m_processVariance, m_R );
//    v = report->getZ() - H * state->getPrediction();
//    distance = (v.trans() * state->getSinv() * v)();
//#ifdef DEBUG1
//    printf("\nPredicted State:\n");
//        (state->getPrediction()).print(2);
//        printf("\nValidating meas:\n");
//        (report->getZ()).print(3);
//        printf("\nInnovation:\n ");
//        v.print(4);
//        printf("\nSinv:\n");
//        (state->getSinv()).print(5);
//        printf("\nMahalinobus dist(innovTrans * s_inv * innov)=%lf maxDist=%lf\n",
//               distance,m_maxDistance);
//#endif
//    if( distance > m_maxDistance )
//    {
//      nextState = 0;
//    }
//    else
//    {
//      MATRIX new_m_x =  state->getPrediction() + state->getW() * v;
//
//      nextState = new CONSTVEL_STATE( this,
//                                      new_m_x(0),
//                                      new_m_x(1),
//                                      new_m_x(2),
//                                      new_m_x(3),
//                                      state->getNextP(),
//                                      state->getLogLikelihoodCoef() -
//                                          distance / 2,
//                                      0 );
//    }
//  }
//  return nextState;
//}
//
//
//
///*-------------------------------------------------------------------*
// | CONSTVEL_MDL::beginNewStates() -- Number of new states to start.
// | Here we are limiting new track growth to only the first frame.
// | If you would like to include new track initiation at every frame
// | comment the "if(!g_is........" line and modify getNextState
// | to deal with state==0 case, or the new track state, where the
// | next/new state would be initialized with the report,startP  and
// | start Likelihood
// *-------------------------------------------------------------------*/
//
//int CONSTVEL_MDL::beginNewStates( MDL_STATE *mdlState,
//                                  MDL_REPORT *mdlReport )
//{
//
//  CONSTVEL_STATE *state = (CONSTVEL_STATE *)mdlState;
//  CONSTPOS_REPORT *report = (CONSTPOS_REPORT *)mdlReport;
//
//  return 1;
//}
//
//
///*-------------------------------------------------------------------*
// | CONSTVEL_MDL::CONSTVEL_MDL() -- constructor for the CONSTVEL_MDL
// *-------------------------------------------------------------------*/
//
//CONSTVEL_MDL::CONSTVEL_MDL( double positionMeasureVarianceX,
//                            double positionMeasureVarianceY,
//                            double gradientMeasureVariance,
//                            double intensityVariance,
//                            double processVariance,
//                            double startProb,
//                            double lambda_x,
//                            double detectProb,
//                            double stateVar,
//                            double maxDistance):
//    CORNER_TRACK_MDL(),
//    m_startLogLikelihood( log( startProb ) ),
//    m_lambda_x( lambda_x),
//    m_skipLogLikelihood( log( 1. - detectProb ) ),
//    m_detectLogLikelihood( log( detectProb ) ),
//    m_maxDistance( maxDistance ),
//    m_processVariance( processVariance ),
//    m_intensityVariance( intensityVariance ),
//    m_stateVariance( stateVar ),
//    m_R( 2, 2 ),
//    m_startP( 4, 4 )
//{
//
//  std::cout << "\nSTARTING A NEW CONSTVEL_MDL\n";
//
//  double pVx = positionMeasureVarianceX;
//  double pVy = positionMeasureVarianceY;
//  double gV = gradientMeasureVariance;
//  MATRIX Q (4, 4 );
//
//
//  m_R.set(  pVx, 0.,
//            0., pVy );
//
//  Q.set(  1./3, 1./2,   0.,   0.,
//          1./2,   1.,   0.,   0.,
//          0.,   0., 1./3, 1./2,
//          0.,   0., 1./2,   1.   );
//
//  Q = Q * m_processVariance;
//
//  m_startP.set(pVx , 0., 0., 0.,
//               0., m_stateVariance, 0., 0.,
//               0., 0., pVy, 0.,
//               0., 0., 0., m_stateVariance );
//#ifdef DEBUG1
//  std::cout << "\nstartP:\n";
//    m_startP.print();
//#endif
//
//  type = 2;
//}
//
//
///*--------------------------------------------------------*
// * getTrackColor( int trackId )
// *--------------------------------------------------------*/
//
//int getTrackColor( int trackId )
//{
//
//
//  static unsigned char color[] =
//      {
//          1,  2,  3,  4,  5,  6,  8,  9, 10, 11, 12, 13, 14, 15,
//          67, 72, 75, 81, 85, 90, 97, 101, 153, 156, 164,
//      };
//
//  return color[ trackId % sizeof( color ) ];
//}
//
//void CORNER_TRACK_MHT::describe(int spaces)
//{
//
//
//  PTR_INTO_ptrDLIST_OF< T_HYPO > tHypoPtr;
//  PTR_INTO_iDLIST_OF< GROUP > groupPtr;
//  PTR_INTO_iDLIST_OF< REPORT > reportPtr;
//  PTR_INTO_iDLIST_OF< T_TREE > tTreePtr;
//  int k;
//
//  Indent( spaces );
//  std::cout << "MHT ";
//  print();
//  std::cout << std::endl;
//  spaces += 2;
//
//  Indent( spaces );
//  std::cout << "lastTrackUsed = " << m_lastTrackIdUsed;
//  std::cout << ", time = " << m_currentTime;
//  std::cout << std::endl;
//
//  Indent( spaces );
//  std::cout << "maxDepth = " << m_maxDepth;
//  std::cout << ", logMinRatio = " << m_logMinGHypoRatio;
//  std::cout << ", maxGHypos = " << m_maxGHypos;
//  std::cout << std::endl;
//
//  Indent( spaces );
//  std::cout << "active tHypo's:";
//  k = 0;
//
//  LOOP_DLIST( tHypoPtr, m_activeTHypoList )
//  {
//    if( k++ >= 3 )
//    {
//      std::cout << std::endl;
//      Indent( spaces );
//      std::cout << "               ";
//      k = 0;
//    }
//
//    std::cout << " ";
//    (*tHypoPtr).print();
//  }
//  std::cout << std::endl;
//
//  Indent( spaces );
//  std::cout << "===== clusters";
//  std::cout << std::endl;
//  LOOP_DLIST( groupPtr, m_groupList )
//  {
//    (*groupPtr).describe( spaces + 2 );
//  }
//
//  Indent( spaces );
//  std::cout << "===== oldReports";
//  std::cout << std::endl;
//  LOOP_DLIST( reportPtr, m_oldReportList )
//  {
//    CONSTPOS_REPORT *creport=(CONSTPOS_REPORT*)(reportPtr.get());
//    (*creport).describe( spaces + 2 );
//  }
//
//  Indent( spaces );
//  std::cout << "===== newReports";
//  std::cout << std::endl;
//  LOOP_DLIST( reportPtr, m_newReportList )
//  {
//    CONSTPOS_REPORT *creport=(CONSTPOS_REPORT*)(reportPtr.get());
//    (*creport).describe( spaces + 2 );
//  }
//
//  Indent( spaces );
//  std::cout << "===== oldTrees";
//  std::cout << std::endl;
//  LOOP_DLIST( tTreePtr, m_tTreeList )
//  {
//    if( tTreePtr == m_nextNewTTree )
//    {
//      Indent( spaces );
//      std::cout << "===== newTrees";
//      std::cout << std::endl;
//    }
//
//    std::cout << std::endl;
//    (**(*tTreePtr).getTree()).describeTree( spaces + 2 );
//  }
//}
