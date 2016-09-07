
#include "chai3d.h"
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>

#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multifit_nlin.h>
#include <gsl/gsl_multiroots.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

#include "Dense"
#include <NonLinearOptimization\LevenbergMarquardt.h>
#include <NonLinearOptimization\lmpar.h>
#include <NonLinearOptimization\chkder.h>
#include <NonLinearOptimization\covar.h>
#include <NonLinearOptimization\dogleg.h>
#include <NonLinearOptimization\fdjac1.h>
#include <NonLinearOptimization\HybridNonLinearSolver.h>
#include <NonLinearOptimization\qrsolv.h>
#include <NonLinearOptimization\r1mpyq.h>
#include <NonLinearOptimization\r1updt.h>
#include <NonLinearOptimization\rwupdt.h>



using namespace chai3d;
using namespace std;

#define BEGIN_TIMING(x,y) \
  unsigned int g_beginTick##x = GetTickCount(); static unsigned int g_n##x = 0; static unsigned int g_tot##x = 0;
#define END_TIMING(x,y) \
  unsigned int g_endTick##x = GetTickCount(); g_tot##x += g_endTick##x-g_beginTick##x; \
  if(++g_n##x >= y) { \
    printf("Timing: %s time %f ms\n", #x, (float)g_tot##x/g_n##x); \
    g_tot##x = 0; \
    g_n##x = 0; \
  }


class ConcentricTubeSet {

public:
	ConcentricTubeSet();

	struct tube {
		float alpha;					// (nx1) base angle of tubes 1-n
		float OD;						// outer diameter [m]
		float ID;						// inner diameter [m]
		float E;						// Young's modulus [Pa]
		float v;						// Poisson's ratio 
		float kappa;                    // curvature [1/m]
		float kappaNew;					// curvature based on interval
		float kb;						// bending stiffness
		float kt;						// torsional stiffness
		float kbNew;					// bending stiffness depending on interval
		float ktInv;					// inverse of torsional stiffness depending on interval
		float Beta;						// Length of straight transmission behind front plate (Beta<=0)  
		float Lc;                       // Length of curved region of each tube
		float Ls;                       // Total length of straight transmission
		int   materialNum;				// Number of material of tube (0=Nitinol,1=PEBA,2=Accura)
		//Conditions for INITIAL VALUE PROBLEM
		float moment_guess;				// guess for moment (KtPsi') at s=0
	};


	struct curve {
		double r;						// radius of curvature 
		cVector3d c;					// center of circle
		double theta;					// angle of curve
		double lc;
		cVector3d bx;					
		cVector3d by;
		cVector3d normVec;
		int dir;
	};


	// Intervals and midpoints
	struct intervalAndMidpoint {
		vector <float> intervalLow;		// list of low intervals
		vector <float> intervalHigh;	// list of high intervals
		vector <float> midpoint;		// list of midpoints
	};
	intervalAndMidpoint intAndMid;		// instance of intervalAndMidpoint

	// State vector
	struct stateVec {
		float *psiVec;					// psi of tube i
		float *moment;					// moment of tube i
		float pos[3];					// position 
		float rot[9];					// rotation
	};
	stateVec stateV;					// instance of stateVec
	double *state;						// flat version of state vector

	// Derivative of state vector
	struct stateVecDot {
		float *psiDot;					// derivative of psi 
		float *momentDot;				// derivative of moment of tube i
		float posDot[3];				// derivative of position
		float rotDot[9];				// derivative of rotation
	};
	stateVecDot stateVDot;				// instance of stateVecDot
	double *stateDeriv;					// flat structure of state derivative vector

	// solved ANGLE of each tube (max of 6 as of now)
	struct angle {
		vector <double> tube1Angle;
		vector <double> tube2Angle;
		vector <double> tube3Angle;
		vector <double> tube4Angle;
		vector <double> tube5Angle;
		vector <double> tube6Angle;
	};

	// solved MOMENT of each tube (max of 6 as of now)
	struct moment {
		vector <double> tube1Moment;
		vector <double> tube2Moment;
		vector <double> tube3Moment;
		vector <double> tube4Moment;
		vector <double> tube5Moment;
		vector <double> tube6Moment;
	};

	// solved POSITION (X,Y,Z) along backbone of CTR
	struct pos {
		vector <double> x;				// vector to hold solved X POSITION along backbone of CTR
		vector <double> y;				// vector to hold solved Y POSITION along backbone of CTR
		vector <double> z;				// vector to hold solved Z POSITION along backbone of CTR
	};

	// solved ROTATION along backbone of CTR
	struct rot {
		vector <double> RB1;	
		vector <double> RB2;
		vector <double> RB3;
		vector <double> RB4;
		vector <double> RB5;
		vector <double> RB6;
		vector <double> RB7;
		vector <double> RB8;
		vector <double> RB9;
	};

	// Variables
	std::vector < tube > m_tubes;		//collection of tubes
	std::vector < curve > m_curves;		//collection of curves
	vector <float> discontinuitiesList; // list of discontinuities in ascending order
	cMatrix3d Rz;						// Rotation matrix
	cMatrix3d RzPrime;					// 3x3 derivative wrt psi of rotation matrix
	cMatrix3d uBHat;					// skew symmetric matrix
	cVector3d uB;						// 3x1 bishop curvature (function of arc length)
	cVector3d Pb_0;						// position at s=0
	cMatrix3d Rb_0;						// z rotation at s=0
	float *initConditions;				// initial conditions for solving ODE
	angle angleStored;					// vector to hold solved ANGLE along backbone of CTR
	moment momentStored;				// vector to hold solved MOMENT along backbone of CTR
	//vector <double> momentStored;
	pos positionStored;					// solved POSITION (X,Y,Z) along backbone of CTR
	rot rotationStored;					// solved ROTATION along backbone of CTR
	vector <double> sStored;			// vector to hold s values from 0 to 1 along backbone of CTR
	float *u;							// correct initial condition for moment at s=0
	bool isValidSet;					// false if solver cannot solve kinematics (get NAN)

	// For finding Jacobian
	gsl_matrix* gInv;
    gsl_matrix* Eq;
    gsl_matrix* Bq;
    gsl_matrix* Eu;
    gsl_matrix* Bu;
	gsl_matrix* Jb;
	Eigen::MatrixXf JbEig;
	Eigen::MatrixXf Jh;
	Eigen::MatrixXf JhPos;
	Eigen::MatrixXf R1;
	Eigen::MatrixXf Jpseudo;
	Eigen::MatrixXf JpseudoPos;

	// Functions
	void addTube(tube t);
	void addCurve(curve c);
	void clear();
	tube getTube(int i);
	gsl_odeiv_system odeSys;
	cVector3d computeBishopCurvature();													// compute bishop curvature (function of arc length)
	cMatrix3d computeZRotDeriv(float psi);												// compute derivative wrt psi of rotation matrix
	cMatrix3d computeVecHat(cVector3d uB);												// compute skew symmetric matrix
	std::vector<float> computeDiscontinuities();										// compute list of discontinuities in ascending order
	void computeIntervals(vector<float> discontinuitiesList);							// compute intervals and midpoints
	void computeKbKt(float midpointValue);												// compute kb and 1/kt based on intervals
	void computeKappa(float midpointValue);												// compute kappa based on intervals
	cMatrix3d computeZRot(float psi);													// compute rotation matrix
	void computeStiffnesses();															// compute stiffnesses kb and kt
	void convertStateDerivToDouble();									// convert float vectors to doubles for use in ode solver
	void convertStateToDouble();

private:
	
		
};



