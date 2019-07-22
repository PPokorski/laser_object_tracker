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

#include <mht/except.h>
#include <except.h>
#include <matrix.h>
#include <mdlmht.h>
#include <param.h>
#include <corner.h>
#include <math.h>
#include <cstdio>		// for  sprintf
#include <list>			// for std::list<>

static int g_numTracks;

class CONSTPOS_REPORT;
class CORNER_TRACK_MDL;
class CONSTPOS_STATE;
class CONSTPOS_MDL;
class CONSTVEL_STATE;
class CONSTVEL_MDL;
class CONSTCURV_STATE;
class CONSTCURV_MDL;
class CORNER_TRACK_MHT;


/*-------------------------------------------------------------------*
 *
 * Need to define measurement vector CONSTPOS_REPORT.
 * This must be derived from the generic base class MDL_REPORT
 * CONSTPOS_REPORT -- reported corner measurement for a CORNER_TRACK
 *
 * It is a vector containing the xy position
 *
 *
 *-------------------------------------------------------------------*/

class CONSTPOS_REPORT: public MDL_REPORT
{
  friend class CONSTVEL_STATE;
  friend class CONSTVEL_MDL;


 private:

  double m_falarmLogLikelihood;    // log of the likelihood that
  // this report is a false alarm
  // (not really part of a CORNER_TRACK)
  MATRIX m_z;                      // (x, dx, y, dy)

 public:
  int m_frameNo;
  size_t m_cornerID;
  CONSTPOS_REPORT( const double &falarmLogLikelihood,
                   const double &x, const double &y,
                   const int &f, const size_t &cornerID):
      MDL_REPORT(),
      m_falarmLogLikelihood( falarmLogLikelihood ),
      m_z( 2, 1 ),
      m_frameNo(f),
      m_cornerID(cornerID)

  {
    m_z.set( x, y);
  }

  CONSTPOS_REPORT( const CONSTPOS_REPORT &src ):
      MDL_REPORT(),
      m_falarmLogLikelihood( src.m_falarmLogLikelihood ),
      m_z( src.m_z ),
      m_frameNo(src.m_frameNo),
      m_cornerID(src.m_cornerID)
  {
  }

  virtual void describe(int spaces)
  {
    m_z.print();
  }

  virtual void print()
  {
    std::cout << "  " <<m_z(0) << " " << m_z(1);
  }

  virtual double getFalarmLogLikelihood()
  {
    return m_falarmLogLikelihood;
  }

  MATRIX &getZ()
  {
    return m_z;
  }

  double getX()
  {
    return m_z( 0 );
  }
  double getY()
  {
    return m_z( 1 );
  }
  void printMeas()
  {
    printf("%lf %lf frame=%d\n",m_z(0),m_z(1),m_frameNo);
  }
};



/*-------------------------------------------------------------------*
 | CORNER_TRACK_MDL -- model of CORNER_TRACKs
 *-------------------------------------------------------------------*/

class CORNER_TRACK_MDL:public MODEL
{
 public:
  int type;
  virtual double getStateX(MDL_STATE*)
  {
    return 0;
  }

  virtual double getStateY(MDL_STATE*)
  {
    return 0;
  }
};


/*-------------------------------------------------------------------*
 *
 * CONSTVEL_MDL -- model for cornerTracks
 *
 * CONSTVEL_MDL must be derived from MODEL
 *
 * CONSTVEL_MDL describes the user's model of a CORNER_TRACK
 * Here the model is implemented as a simple linear Kalman filter
 *
 *-------------------------------------------------------------------*/


class CONSTVEL_MDL: public CORNER_TRACK_MDL
{
 private:
  double m_lambda_x;
  double m_startLogLikelihood;     // likelihood of a CORNER_TRACK starting
  double m_endLogLikelihood;       // likelihood of a CORNER_TRACK ending
  double m_continueLogLikelihood;  // likelihood of a CORNER_TRACK not
  //   ending
  double m_skipLogLikelihood;      // likelihood of not detecting a
  //   CORNER_TRACK that hasn't ended
  double m_detectLogLikelihood;    // likelihood of detecting a
  //   CORNER_TRACK that hasn't ended

  double m_maxDistance;            // maximum mahalanobis distance
  //   allowed for validating a
  //   report to a CORNER_TRACK

  double m_processVariance;        // process noise
  double m_intensityVariance;
  double m_stateVariance;
  MATRIX m_R;                      // measurement covariance
  MATRIX m_startP;                 // covariance matrix to use at
  //   start of a CORNER_TRACK
 public:

  CONSTVEL_MDL( double positionMeasureVarianceX,
                double positionMeasureVarianceY,
                double gradientMeasureVariance,
                double intensityVariance,
                double processVariance,
                double startProb,
                double lambda_x,
                double detectProb,
                double stateVar,
                double maxDistance);

  virtual int beginNewStates( MDL_STATE *mdlState,
                              MDL_REPORT *mdlReport );
  virtual MDL_STATE *getNewState( int stateNum,
                                  MDL_STATE *mdlState,
                                  MDL_REPORT *mdlReport );
  virtual double getEndLogLikelihood( MDL_STATE * );
  virtual double getContinueLogLikelihood( MDL_STATE * );
  virtual double getSkipLogLikelihood( MDL_STATE *mdlState );
  virtual double getDetectLogLikelihood( MDL_STATE * )
  {
    return m_detectLogLikelihood;
  }
  virtual double getStateX(MDL_STATE *s);
  virtual double getStateY(MDL_STATE *s);
 private:

  CONSTVEL_STATE* getNextState( CONSTVEL_STATE *state,
                                CONSTPOS_REPORT *report );
};

/*-------------------------------------------------------------------*
 *
 * CONSTVEL_STATE -- state estimate for a CORNER_TRACK
 *
 * CONSTVEL_STATE hold the state information for the CORNER_TRACK
 * E.g. the vector (x, dx, y, dy)
 *
 *-------------------------------------------------------------------*/

class CONSTVEL_STATE: public MDL_STATE
{
  friend class ONSTPOS_REPORT;
  friend class CONSTCURV_MDL;
  friend class CONSTVEL_MDL;
  friend class CONSTPOS_MDL;

 private:

  MATRIX m_x;                      // state estimate (x, dx, y, dy)
  MATRIX m_P;                      // covariance matrix
  double m_logLikelihood;          // likelihood that this state
  //   is the true state of the
  //   CORNER_TRACK after the state
  //   that it was born from (in
  //   CONSTVEL_MDL::getNewState())

  int m_numSkipped;
  int m_hasBeenSetup;              // 0 before the following variables
  //   have been filled in, 1 after

  double m_ds;                     // "time" step until the next state
  //   (chosen so that the next state
  //   lands in a neighboring pixel)
  double m_logLikelihoodCoef;      // part of likelihood calculation
  //   that's independent of the
  //   inovation
  MATRIX *m_Sinv;                  // inverse of the innovation
  //   covariance
  MATRIX *m_W;                     // filter gain
  MATRIX *m_nextP;                 // updated state covariance
  //   (covariance for next state)
  MATRIX *m_x1;                    // state prediction

 private:

  CONSTVEL_STATE( CONSTVEL_MDL *mdl,
                  const double &x,
                  const double &dx,
                  const double &y,
                  const double &dy,
                  MATRIX &P,
                  const double &logLikelihood,
                  const int &numSkipped):
      MDL_STATE( mdl ),
      m_logLikelihood( logLikelihood ),
      m_hasBeenSetup( 0 ),
      m_numSkipped(numSkipped),
      m_x(4,1),
      m_P(P),
      m_ds( 0 ),
      m_x1( 0 ),
      m_nextP( 0 ),
      m_Sinv( 0 ),
      m_W( 0 )
  {
    m_x(0)=x;
    m_x(1)=dx;
    m_x(2)=y;
    m_x(3)=dy;
  }


  CONSTVEL_STATE( const CONSTVEL_STATE &src ):
      MDL_STATE( src.getMdl() ),
      m_x( src.m_x ),
      m_P( src.m_P ),
      m_logLikelihood( src.m_logLikelihood ),
      m_hasBeenSetup( 0 ),
      m_numSkipped(src.m_numSkipped),
      m_ds( 0 ),
      m_x1( 0 ),
      m_nextP( 0 ),
      m_Sinv( 0 ),
      m_W( 0 )
  {
  }


 private:

  void setup( double processVariance, const MATRIX &R );

  void cleanup()
  {


    if( m_hasBeenSetup )
    {
      delete m_x1;
      m_x1 = 0;
      delete m_nextP;
      m_nextP = 0;
      delete m_Sinv;
      m_Sinv = 0;
      delete m_W;
      m_W = 0;

      m_hasBeenSetup = 0;
    }
  }


  int getNumSkipped()
  {
    return m_numSkipped;
  }
  double getLogLikelihoodCoef()
  {
    checkSetup();
    return m_logLikelihoodCoef;
  }
  MATRIX &getPrediction()
  {
    checkSetup();
    return *m_x1;
  }
  MATRIX &getNextP()
  {
    checkSetup();
    return *m_nextP;
  }
  MATRIX &getSinv()
  {
    checkSetup();
    return *m_Sinv;
  }
  MATRIX &getW()
  {
    checkSetup();
    return *m_W;
  }

#ifdef TSTBUG

  void checkSetup()
    {
        assert( m_hasBeenSetup );
        //  THROW_ERR( "Trying to get derived info from a CONSTPOS state"
        //             " that hasn't been setup()" )
    }

#else
  void checkSetup() {}
#endif

 public:

  ~CONSTVEL_STATE()
  {
    cleanup();    //SHOULD THIS BE PRIVATE?
  }
  virtual double getLogLikelihood()
  {
    return m_logLikelihood;
  }

  virtual void print()
  {
    std::cout << "ConstVel State: "<< m_x(0) << " ,"
              <<m_x(2);
  }
  double getX()
  {
    return m_x( 0 );
  }
  double getDX()
  {
    return m_x( 1 );
  }
  double getY()
  {
    return m_x( 2 );
  }
  double getDY()
  {
    return m_x( 3 );
  }

  void setDX(double val)
  {
    m_x( 1 )=val;
  }
  void setDY(double val)
  {
    m_x( 3 )=val;
  }

  double getX1()
  {
    checkSetup();
    return (*m_x1)( 0 );
  }
  double getDX1()
  {
    checkSetup();
    return (*m_x1)( 1 );
  }
  double getY1()
  {
    checkSetup();
    return (*m_x1)( 2 );
  }
  double getDY1()
  {
    checkSetup();
    return (*m_x1)( 3 );
  }

  double getDS()
  {
    checkSetup();
    return m_ds;
  }
};

/*-------------------------------------------------------------------*
 *
 * FALARM -- is a structure for holding flase alarms on an
 *           "intrusive" list.
 *
 * For the definition of an intrusive list see the file list.h
 *
 *-------------------------------------------------------------------*/

class FALARM: public DLISTnode
{
 public:
  double rX, rY;
  int frameNo;
  size_t cornerID;

  FALARM( CONSTPOS_REPORT *xreport ):
      DLISTnode(),
      rX( xreport->getX() ),
      rY( xreport->getY() ),
      frameNo( xreport->m_frameNo ),
      cornerID( xreport->m_cornerID )
  {
  }

 protected:

  MEMBERS_FOR_DLISTnode( FALARM )
};


/*-------------------------------------------------------------------*
 *
 * CORNER_TRACK_ELEMENT -- is a structure for holding an individual
 * element of a CORNER_TRACK on an "intrusive" doubly linked list
 *
 * For the definition of an intrusive list see the file list.h
 *
 *-------------------------------------------------------------------*/


class CORNER_TRACK_ELEMENT: public DLISTnode
{
 public:
  int hasReport;
  double sx,sy;
  double rx,ry;
  int frameNo;
  int time;
  double logLikelihood;
  char   model[30];
  size_t cornerID;

  CORNER_TRACK_ELEMENT(double s_x, double s_y, double r_x, double r_y, double prob, int type,int t,int f,size_t id):
      DLISTnode(),
      sx(s_x),sy(s_y),rx(r_x),ry(r_y),logLikelihood(prob),time(t),frameNo(f),cornerID(id)
  {
    if (!isnan(r_x) && !isnan(r_y))
    {
      hasReport=1;
    }
    else
    {
      hasReport=0;
    }

    switch(type)
    {
      case 1:
        sprintf(model,"CONSTANT MODEL");
        break;
      case 2:
        sprintf(model,"CONSTANT VELOCITY");
        break;
      case 3:
        sprintf(model,"CONSTANT CURV");
        break;
    }
  }

 protected:

  MEMBERS_FOR_DLISTnode( CORNER_TRACK_ELEMENT )
};

/*-------------------------------------------------------------------*
 *
 * CORNER_TRACK -- is a structure to save info about a CORNER_TRACK
 * on an "intrusive" doubly linked list
 *
 * For the definition of an intrusive list see the file list.h
 *
 *-------------------------------------------------------------------*/

class CORNER_TRACK: public DLISTnode
{
 public:
  int id;
  int color;

  std::list< CORNER_TRACK_ELEMENT > list;

  CORNER_TRACK( int idArg, int colorArg ):
      DLISTnode(),
      id( idArg ),
      color( colorArg ),
      list()
  {
  }

 protected:

  MEMBERS_FOR_DLISTnode( CORNER_TRACK )
};



/*-------------------------------------------------------------------*
 *
 * CORNER_TRACK_MHT -- MDL_MHT class for CORNER_TRACK
 *
 *-------------------------------------------------------------------*/

class CORNER_TRACK_MHT: public MDL_MHT
{
 public:

  CORNER_TRACK_MHT( double fprob,int maxDepth, double minGHypoRatio, int maxGHypos,
                    ptrDLIST_OF<MODEL> mdlist ):
      MDL_MHT( maxDepth, minGHypoRatio, maxGHypos ),
      m_falarmLogLikelihood( log(fprob) ),
      m_cornerTracks(),
      m_falarms()
  {
    m_modelList.appendCopy( mdlist );
  }
  virtual void describe(int spaces=0);
  virtual std::list< CORNER_TRACK > GetTracks() const
  {
    return m_cornerTracks;
  }
  virtual std::list< FALARM > GetFalseAlarms() const
  {
    return m_falarms;
  }

 private:
  double m_falarmLogLikelihood;
  std::list< CORNER_TRACK > m_cornerTracks;
  std::list< FALARM > m_falarms;

 protected:

  virtual void measure(const std::list<CORNER> &newReports);

  virtual void startTrack( int trackId, int,
                           MDL_STATE *state, MDL_REPORT *report )
  {
    g_numTracks++;
    CONSTPOS_REPORT* r= (CONSTPOS_REPORT*)report;

    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
//      printf("Calling Verify in statrtTRack\n");
    verify( trackId, r->getX(),
            r->getY(),
            mdl->getStateX(state),
            mdl->getStateY(state),
            state->getLogLikelihood(),
            mdl->type,r->m_frameNo,r->m_cornerID);
  }

  virtual void continueTrack( int trackId, int,
                              MDL_STATE *state, MDL_REPORT *report )
  {
    CONSTPOS_REPORT* r= (CONSTPOS_REPORT*)report;
    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
//      printf("Calling Verify in continueTRack\n");
    verify( trackId, r->getX(),
            r->getY(),
            mdl->getStateX(state),
            mdl->getStateY(state),
            state->getLogLikelihood(),
            mdl->type,r->m_frameNo,r->m_cornerID);
  }

  virtual void skipTrack( int trackId, int, MDL_STATE *state )
  {
    CONSTVEL_MDL* mdl = (CONSTVEL_MDL*)(state->getMdl());
//      printf("Calling Verify in skipTRack\n");
    verify( trackId, NAN, NAN, mdl->getStateX(state), mdl->getStateY(state),
            state->getLogLikelihood(),
            mdl->type,-9,0);
  }

  virtual void endTrack( int, int )
  {
//      printf("Verifying endTrack\n");
    g_numTracks--;
  }

  virtual void falseAlarm( int, MDL_REPORT *report )
  {
    saveFalarm( (CONSTPOS_REPORT*)report );
  }

 private:
  CORNER_TRACK* findTrack( const int &id );
  void saveFalarm( CONSTPOS_REPORT *report );
  void verify( int trackID, double r_x, double r_y, double s_x,
               double s_y, double likelihood, int modelType, int frame, size_t cornerID);
};

int getTrackColor( int trackId );

#endif  // LASER_OBJECT_TRACKER_TRACKING_MHT_MOTION_MODEL_HPP
