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

#define NO_DEBUG1
#define NO_DEBUG2
#define NO_DEBUG3
//#define SUM_SQUAREDIFF
#define CORR_COEFF

double EPSILON = 0.00000000000001;
int mht::internal::g_time = 0;

namespace laser_object_tracker {
namespace tracking {
namespace mht {
ConstVelocityModel::ConstVelocityModel(double position_measure_variance_x,
                                       double position_measure_variance_y,
                                       double gradient_measure_variance,
                                       double lambda_x,
                                       double start_probability,
                                       double detection_probability,
                                       double max_mahalanobis_distance,
                                       double process_variance,
                                       double state_variance)
    : MODEL(),
      lambda_x_(lambda_x),
      start_log_likelihood_(std::log(start_probability)),
      skip_log_likelihood_(std::log(1.0 - detection_probability)),
      detect_log_likelihood_(std::log(detection_probability)),
      max_mahalanobis_distance_(max_mahalanobis_distance),
      process_variance_(process_variance),
      state_variance_(state_variance),
      measurement_covariance_(2, 2),
      initial_covariance_(4, 4) {
  MATRIX process_noise(4, 4);

  measurement_covariance_.set(position_measure_variance_x, 0.0,
                              0.0, position_measure_variance_y);

  // TODO Why??
  process_noise.set(1.0 / 3.0, 1.0 / 2.0, 0.0, 0.0,
                    1.0 / 2.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0 / 3.0, 1.0 / 2.0,
                    0.0, 0.0, 1.0 / 2.0, 1.0);

  process_noise = process_noise * process_variance_;

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

void ConstVelocityModel::endNewStates() {
  MODEL::endNewStates();
}
double ConstVelocityModel::getEndLogLikelihood(MDL_STATE* state) {
  return MODEL::getEndLogLikelihood(state);
}
double ConstVelocityModel::getContinueLogLikelihood(MDL_STATE* state) {
  return MODEL::getContinueLogLikelihood(state);
}
double ConstVelocityModel::getSkipLogLikelihood(MDL_STATE* state) {
  return MODEL::getSkipLogLikelihood(state);
}
double ConstVelocityModel::getDetectLogLikelihood(MDL_STATE* state) {
  return MODEL::getDetectLogLikelihood(state);
}

ConstVelocityState* ConstVelocityModel::getNextState(ConstVelocityState* state,
                                                     PositionReport* report) {
  return nullptr;
}

}  // namespace mht
}  // namespace tracking
}  // namespace laser_object_tracker

/*------------------------------------------------------*
 * findTrack():  look for the track with given id in the
 * cornerTrackList and return a ptr to it.  If
 * not found create one
 *------------------------------------------------------*/

CORNER_TRACK *CORNER_TRACK_MHT::findTrack( const int &id )
{

  for (std::list<CORNER_TRACK>::iterator p = m_cornerTracks.begin();
       p != m_cornerTracks.end();
       p++)
  {
    if( p->id == id )
    {
      return &(*p);
    }
  }

  m_cornerTracks.push_back( CORNER_TRACK( id, getTrackColor(id) ) );

  return &(m_cornerTracks.back());
}

/*-------------------------------------------------------------------*
 * saveFalarm(report): saves the given report in the FALARM list
 *-------------------------------------------------------------------*/
void CORNER_TRACK_MHT::saveFalarm( CONSTPOS_REPORT *report )
{
  m_falarms.push_back( FALARM( report ) );
}


/*-------------------------------------------------------------------*
 * verify(trackId, report, state) : find the CORNER_TRACK with the given
 *                    id, create a new CORNER_TRACK_ELEMENT with the
 *                    given report & state and append it to the
 *                    list of CORNER_TRACK_ELEMENTs of that CORNER_TRACK
 *-------------------------------------------------------------------*/
void CORNER_TRACK_MHT::verify( int trackId, double r_x, double r_y, double s_x, double s_y,
                               double logLikelihood,
                               int modelType, int frame, size_t id)
{
  CORNER_TRACK *track;

//  printf("Verifying trackId=%d r_x=%lf r_y=%lf s_x=%lf s_y=%lf frame=%d\n",
//			trackId,r_x,r_y,s_x,s_y,frame);
  track = findTrack( trackId );
  track->list.push_back( CORNER_TRACK_ELEMENT( s_x,s_y,r_x,r_y,logLikelihood,modelType,mht::internal::g_time,frame,id));
}

/**-------------------------------------------------------------------
 * void CORNER_TRACK_MHT::measure()
 * Take the corners of the current frame and install
 * them as reports
 *-------------------------------------------------------------------*/

void CORNER_TRACK_MHT::measure(const std::list<CORNER> &newReports)
{
  for (std::list<CORNER>::const_iterator cornerPtr = newReports.begin();
       cornerPtr != newReports.end();
       cornerPtr++)
  {
    installReport(new CONSTPOS_REPORT(m_falarmLogLikelihood,
                                      cornerPtr->x, cornerPtr->y,
                                      cornerPtr->m_frameNo,cornerPtr->m_cornerID)
    );
  }

}

/*-------------------------------------------------------------------*
 | LOG_NORMFACTOR -- constant part of likelihood calculation
 *-------------------------------------------------------------------*/

static const double LOG_NORMFACTOR =
    /* log( 2PI^(measureVars/2) ) = */ 1.5963597;


/*-------------------------------------------------------------------*
 | CORNER_TRACK_STATE::setup() -- compute parts of Kalman filter
 |                           calculation that are independent of
 |                           reports
 *-------------------------------------------------------------------*/

void CONSTVEL_STATE::setup( double processVariance, const MATRIX &R )
{


  /* don't do this more than once */
  if( m_hasBeenSetup )
  {
    return;
  }

  m_ds = 1;

  /* compute the state transition matrix and process covariance matrix
     based on the above time step */

  double ds2 = m_ds * m_ds;
  double ds3 = ds2 * m_ds;

  static MATRIX F( 4, 4 );
  F.set(     1.,  m_ds,    0.,    0.,
             0.,    1.,    0.,    0.,
             0.,    0.,    1.,  m_ds,
             0.,    0.,    0.,    1.    );


  static MATRIX Q( 4, 4 );
  Q.set(  ds3/3, ds2/2,    0.,    0.,
          ds2/2,  m_ds,    0.,    0.,
          0.,    0., ds3/3, ds2/2,
          0.,    0., ds2/2,  m_ds  );
  Q = Q * processVariance;

  static MATRIX H(2,4);
  H.set(1., 0., 0., 0.,
        0., 0., 1., 0.);


  /* fill in the rest of the variables */

  MATRIX P1 = F * m_P * F.trans() + Q; // state prediction covariance

  MATRIX S = H * P1 * H.trans() + R;  // innovation covariance

  m_logLikelihoodCoef = -(LOG_NORMFACTOR + log( S.det() ) / 2);

  m_Sinv = new MATRIX( S.inv() );
//  printf("Sinv:\n"); m_Sinv->print();

  m_W = new MATRIX( P1 * H.trans() * *m_Sinv );

  MATRIX tmp(4,4);
  tmp =  *m_W * S * m_W->trans();

  MATRIX tmp1(4,4);
  tmp1 = P1-tmp;

  m_nextP = new MATRIX( tmp1 );
  m_x1 = new MATRIX( F * m_x );

  m_hasBeenSetup = 1;

#ifdef DEBUG1
  printf("\nF:\n");
    F.print();
    printf("\nm_P:\n");
    m_P.print();
    printf("\nQ=\n");
    Q.print();
    printf("\nState Pred Cov(P1=F*m_P*F.trans +Q):\n");
    P1.print();
    printf("\nInnov Cov(S=H*P1*H.trans):\n");
    S.print();
    printf("\nS_inv:\n");
    m_Sinv->print();
    printf("\nPrevious State:\n");
    m_x.print();
    printf("LOG_NORMFACTOR =%lf log( S.det() ) / 2)=%lf\n",LOG_NORMFACTOR, log( S.det() ) / 2);
    printf(" m_logLikelihoodCoef= %lf\n", m_logLikelihoodCoef);
#endif

}

/*--------------------------------------------*
 * CONSTVEL_MDL::getStateX(MDL_STATE *s)
 *--------------------------------------------*/

double CONSTVEL_MDL::getStateX(MDL_STATE *s)
{
  return ((CONSTVEL_STATE*)s)->getX();
}


/*--------------------------------------------*
 * CONSTVEL_MDL::getStateY(MDL_STATE *s)
 *--------------------------------------------*/

double CONSTVEL_MDL::getStateY(MDL_STATE *s)
{
  return ((CONSTVEL_STATE*)s)->getY();
}


double CONSTVEL_MDL::getEndLogLikelihood( MDL_STATE *s )
{
  CONSTVEL_STATE *cs = (CONSTVEL_STATE*)s;
  int m = cs->m_numSkipped;
  double endProb = 1.0 - exp( -m / m_lambda_x);
  endProb += (endProb == 0.0) ? EPSILON : 0.0;
  m_endLogLikelihood = log( endProb);
  return m_endLogLikelihood;
}
double CONSTVEL_MDL::getContinueLogLikelihood( MDL_STATE *s )
{
  CONSTVEL_STATE *cs = (CONSTVEL_STATE*)s;
  int m = cs->m_numSkipped;
  double endProb = 1.0 - exp( -m / m_lambda_x);
  endProb += (endProb == 0.0) ? EPSILON : 0.0;
  m_continueLogLikelihood = log(1.0-endProb);
  return m_continueLogLikelihood;
}



/*-------------------------------------------------------------------*
 | CONSTVEL_MDL::getSkipLogLikelihood() -- get the likelihood of
 |                                        skipping a(nother) report
 *-------------------------------------------------------------------*/

double CONSTVEL_MDL::getSkipLogLikelihood( MDL_STATE *mdlState )
{
  CONSTVEL_STATE *state = (CONSTVEL_STATE *)mdlState;

  return  m_skipLogLikelihood;

//  return (state->getNumSkipped() + 1) * m_skipLogLikelihood;
}


/*------------------------------------------------------------------*
 * MDL_STATE* CONSTVEL_MDL::getNewState(...)
 *------------------------------------------------------------------*/

MDL_STATE* CONSTVEL_MDL::getNewState( int stateNum,
                                      MDL_STATE *mdlState,
                                      MDL_REPORT *mdlReport )
{

  CONSTVEL_STATE *state = (CONSTVEL_STATE *) mdlState;
  CONSTPOS_REPORT *report = (CONSTPOS_REPORT *) mdlReport;
  double dx,dy;

  switch(stateNum)
  {
    case 0:   // Continue constVel State
    {
      if( state != 0 && report !=0 && state->getDX() == 0
          && state->getDY() == 0 )
      {
        dx = report->getX() - state->getX();
        dy = report->getY() - state->getY();

        state->setDX(dx);
        state->setDY(dy);
      }

      CONSTVEL_STATE *newState;
      newState=getNextState(state,report);
      return (MDL_STATE*) newState;
    }
    default:
      assert(false);//("Too many calls to CONSTVEL_MDL::getNewState()");
  }

}



/*-------------------------------------------------------------------*
 | CONSTVEL_MDL::getNextState() -- get the next state estimate, given
 |                                a previous state estimate and a
 |                                reported measurement
 *-------------------------------------------------------------------*/

CONSTVEL_STATE* CONSTVEL_MDL::getNextState( CONSTVEL_STATE *state,
                                            CONSTPOS_REPORT *report )
{
  CONSTVEL_STATE *nextState;          // new state
  static MATRIX v( 2,1 );              // innovation
  double distance;                   // mahalanobis distance

  static MATRIX H(2,4);
  H.set(1., 0., 0., 0.,
        0., 0., 1., 0.);


  if( state == 0 )
  {
    /* starting a new track */

    double x=report->getX();
    double y=report->getY();

#ifdef DEBUG1
    printf("\nStart a new State/Contour with Cov:");
        m_startP.print(2);
        printf("\nSTARTING NEW STATE WITH %lf %lf\n",x,y);
#endif

    nextState = new CONSTVEL_STATE( this,
                                    x,
                                    0.,
                                    y,
                                    0.,
                                    m_startP,
                                    m_startLogLikelihood,
                                    0 );
  }
  else if( report == 0 )
  {
    /* continuing an existing CORNER_TRACK, skipping a measurement */

    state->setup( m_processVariance, m_R );

#ifdef DEBUG1
    printf("Skipping meas(report=0); continued state= %lf %lf %lf %lf\n",
               state->getX1(), state->getDX1(),
               state->getY1(), state->getDY1() );
#endif

    nextState = new CONSTVEL_STATE( this,
                                    state->getX1(),
                                    state->getDX1(),
                                    state->getY1(),
                                    state->getDY1(),
                                    state->getNextP(),
                                    0.,
                                    state->getNumSkipped() + 1 );
  }
  else
  {
    /* continuing an existing CORNER_TRACK, with a measurement */

    state->setup( m_processVariance, m_R );
    v = report->getZ() - H * state->getPrediction();
    distance = (v.trans() * state->getSinv() * v)();
#ifdef DEBUG1
    printf("\nPredicted State:\n");
        (state->getPrediction()).print(2);
        printf("\nValidating meas:\n");
        (report->getZ()).print(3);
        printf("\nInnovation:\n ");
        v.print(4);
        printf("\nSinv:\n");
        (state->getSinv()).print(5);
        printf("\nMahalinobus dist(innovTrans * s_inv * innov)=%lf maxDist=%lf\n",
               distance,m_maxDistance);
#endif
    if( distance > m_maxDistance )
    {
      nextState = 0;
    }
    else
    {
      MATRIX new_m_x =  state->getPrediction() + state->getW() * v;

      nextState = new CONSTVEL_STATE( this,
                                      new_m_x(0),
                                      new_m_x(1),
                                      new_m_x(2),
                                      new_m_x(3),
                                      state->getNextP(),
                                      state->getLogLikelihoodCoef() -
                                          distance / 2,
                                      0 );
    }
  }
  return nextState;
}



/*-------------------------------------------------------------------*
 | CONSTVEL_MDL::beginNewStates() -- Number of new states to start.
 | Here we are limiting new track growth to only the first frame.
 | If you would like to include new track initiation at every frame
 | comment the "if(!g_is........" line and modify getNextState
 | to deal with state==0 case, or the new track state, where the
 | next/new state would be initialized with the report,startP  and
 | start Likelihood
 *-------------------------------------------------------------------*/

int CONSTVEL_MDL::beginNewStates( MDL_STATE *mdlState,
                                  MDL_REPORT *mdlReport )
{

  CONSTVEL_STATE *state = (CONSTVEL_STATE *)mdlState;
  CONSTPOS_REPORT *report = (CONSTPOS_REPORT *)mdlReport;

  return 1;
}


/*-------------------------------------------------------------------*
 | CONSTVEL_MDL::CONSTVEL_MDL() -- constructor for the CONSTVEL_MDL
 *-------------------------------------------------------------------*/

CONSTVEL_MDL::CONSTVEL_MDL( double positionMeasureVarianceX,
                            double positionMeasureVarianceY,
                            double gradientMeasureVariance,
                            double intensityVariance,
                            double processVariance,
                            double startProb,
                            double lambda_x,
                            double detectProb,
                            double stateVar,
                            double maxDistance):
    CORNER_TRACK_MDL(),
    m_startLogLikelihood( log( startProb ) ),
    m_lambda_x( lambda_x),
    m_skipLogLikelihood( log( 1. - detectProb ) ),
    m_detectLogLikelihood( log( detectProb ) ),
    m_maxDistance( maxDistance ),
    m_processVariance( processVariance ),
    m_intensityVariance( intensityVariance ),
    m_stateVariance( stateVar ),
    m_R( 2, 2 ),
    m_startP( 4, 4 )
{

  std::cout << "\nSTARTING A NEW CONSTVEL_MDL\n";

  double pVx = positionMeasureVarianceX;
  double pVy = positionMeasureVarianceY;
  double gV = gradientMeasureVariance;
  MATRIX Q (4, 4 );


  m_R.set(  pVx, 0.,
            0., pVy );

  Q.set(  1./3, 1./2,   0.,   0.,
          1./2,   1.,   0.,   0.,
          0.,   0., 1./3, 1./2,
          0.,   0., 1./2,   1.   );

  Q = Q * m_processVariance;

  m_startP.set(pVx , 0., 0., 0.,
               0., m_stateVariance, 0., 0.,
               0., 0., pVy, 0.,
               0., 0., 0., m_stateVariance );
#ifdef DEBUG1
  std::cout << "\nstartP:\n";
    m_startP.print();
#endif

  type = 2;
}


/*--------------------------------------------------------*
 * getTrackColor( int trackId )
 *--------------------------------------------------------*/

int getTrackColor( int trackId )
{


  static unsigned char color[] =
      {
          1,  2,  3,  4,  5,  6,  8,  9, 10, 11, 12, 13, 14, 15,
          67, 72, 75, 81, 85, 90, 97, 101, 153, 156, 164,
      };

  return color[ trackId % sizeof( color ) ];
}

void CORNER_TRACK_MHT::describe(int spaces)
{


  PTR_INTO_ptrDLIST_OF< T_HYPO > tHypoPtr;
  PTR_INTO_iDLIST_OF< GROUP > groupPtr;
  PTR_INTO_iDLIST_OF< REPORT > reportPtr;
  PTR_INTO_iDLIST_OF< T_TREE > tTreePtr;
  int k;

  Indent( spaces );
  std::cout << "MHT ";
  print();
  std::cout << std::endl;
  spaces += 2;

  Indent( spaces );
  std::cout << "lastTrackUsed = " << m_lastTrackIdUsed;
  std::cout << ", time = " << m_currentTime;
  std::cout << std::endl;

  Indent( spaces );
  std::cout << "maxDepth = " << m_maxDepth;
  std::cout << ", logMinRatio = " << m_logMinGHypoRatio;
  std::cout << ", maxGHypos = " << m_maxGHypos;
  std::cout << std::endl;

  Indent( spaces );
  std::cout << "active tHypo's:";
  k = 0;

  LOOP_DLIST( tHypoPtr, m_activeTHypoList )
  {
    if( k++ >= 3 )
    {
      std::cout << std::endl;
      Indent( spaces );
      std::cout << "               ";
      k = 0;
    }

    std::cout << " ";
    (*tHypoPtr).print();
  }
  std::cout << std::endl;

  Indent( spaces );
  std::cout << "===== clusters";
  std::cout << std::endl;
  LOOP_DLIST( groupPtr, m_groupList )
  {
    (*groupPtr).describe( spaces + 2 );
  }

  Indent( spaces );
  std::cout << "===== oldReports";
  std::cout << std::endl;
  LOOP_DLIST( reportPtr, m_oldReportList )
  {
    CONSTPOS_REPORT *creport=(CONSTPOS_REPORT*)(reportPtr.get());
    (*creport).describe( spaces + 2 );
  }

  Indent( spaces );
  std::cout << "===== newReports";
  std::cout << std::endl;
  LOOP_DLIST( reportPtr, m_newReportList )
  {
    CONSTPOS_REPORT *creport=(CONSTPOS_REPORT*)(reportPtr.get());
    (*creport).describe( spaces + 2 );
  }

  Indent( spaces );
  std::cout << "===== oldTrees";
  std::cout << std::endl;
  LOOP_DLIST( tTreePtr, m_tTreeList )
  {
    if( tTreePtr == m_nextNewTTree )
    {
      Indent( spaces );
      std::cout << "===== newTrees";
      std::cout << std::endl;
    }

    std::cout << std::endl;
    (**(*tTreePtr).getTree()).describeTree( spaces + 2 );
  }
}
