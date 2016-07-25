#include "ConcentricTubeSet.h"





using namespace chai3d;
using namespace std;


//// TUBE PARAMETERS
int numTubes;
int numStates;
float *solvedState;



Eigen::VectorXf xvec(3);


//------------------------------- Functions --------------------------------------
int kinematicFunction (double s, const double y[], double dyds[], void *params);
int jac (double t, const double y[], double *dfdy, double dfdt[], void *params);
int findZeroResVec(const gsl_vector * x, void *params, gsl_vector * f);
//int findZeroResVec_df (const gsl_vector * x, void *params, gsl_matrix *J);
//int findZeroResVec_fdf (const gsl_vector *x, void *params, gsl_vector *f, gsl_matrix *J);
int solveInitConditions(ConcentricTubeSet &set);
int solveForwardKinematics(ConcentricTubeSet &set);
gsl_matrix* computeG(ConcentricTubeSet &set);
gsl_matrix* computeGInv(gsl_matrix * gBar);
void computeEq(ConcentricTubeSet &set, gsl_vector* q, gsl_vector* uBar);
void computeEu(ConcentricTubeSet &set, gsl_vector* q, gsl_vector* uBar);
void printComputedKinematics(ConcentricTubeSet set);
Eigen::MatrixXf convertGSLMatrixToEigen(gsl_matrix* gslMat);
gsl_matrix* convertEigenToGSLMatrix(Eigen::MatrixXf eigMat);
Eigen::VectorXf convertGSLVectorToEigen(gsl_vector* gslVec);
gsl_vector* convertEigenToGSLVector(Eigen::VectorXf eigVec);


struct funcToSolve
{
	ConcentricTubeSet *tubeSet;

	int operator()(const Eigen::VectorXf &xvec, Eigen::VectorXf &fvec) const 
	{ 
		// convert Eigen::VectorXf to gsl_vector
		gsl_vector *x_gsl = gsl_vector_alloc (numTubes);
		gsl_vector *f_gsl = gsl_vector_alloc (numTubes);
		for(int i=0; i<numTubes; i++) {
			gsl_vector_set (x_gsl, i, xvec(i));
			gsl_vector_set (f_gsl, i, fvec(i));
		}

		findZeroResVec(x_gsl, (void *)this->tubeSet, f_gsl);

		for(int i=0; i<numTubes; i++) {
			// convert gslvector f into fvec (eigen vector)
			fvec(i) = gsl_vector_get (f_gsl,i);
		}
		return 0;
	}

#define EPS 1e-5
	int df(const Eigen::VectorXf &xvec, Eigen::MatrixXf &fjac) const
    {
		Eigen::VectorXf epsilon1(numTubes);
		Eigen::VectorXf epsilon2(numTubes);
		Eigen::VectorXf epsilon3(numTubes);
		for(int i=0; i<numTubes; i++) {
			epsilon1(i) = 0;
			epsilon2(i) = 0;
			epsilon3(i) = 0;
		}

		epsilon1(0) = EPS;
		epsilon2(1) = EPS;
		epsilon3(2) = EPS;


	    // x
		Eigen::VectorXf fvecxa(numTubes);
		operator()(xvec + epsilon1, fvecxa);
		Eigen::VectorXf fvecxb(numTubes);
		operator()(xvec - epsilon1, fvecxb);
		Eigen::VectorXf fvecx = (fvecxa-fvecxb)/(2*EPS);

		// y
		Eigen::VectorXf fvecya(numTubes);
		operator()(xvec + epsilon2, fvecya);
		Eigen::VectorXf fvecyb(numTubes);
		operator()(xvec - epsilon2, fvecyb);
		Eigen::VectorXf fvecy = (fvecya-fvecyb)/(2*EPS);

		// z
		Eigen::VectorXf fvecza(numTubes);
		operator()(xvec + epsilon3, fvecza);
		Eigen::VectorXf fveczb(numTubes);
		operator()(xvec - epsilon3, fveczb);
		Eigen::VectorXf fvecz = (fvecza-fveczb)/(2*EPS);

		// form jacobian
		fjac.col(0) = fvecx;
		fjac.col(1) = fvecy;
		fjac.col(2) = fvecz;


		return 0;
	}

	int inputs() const { return numTubes; }	// inputs is the dimension of x.
	int values() const { return numTubes; } // "values" is the number of f_i and 

};


// -------------------------------------------------------------------------------

ConcentricTubeSet::ConcentricTubeSet() {
	
}

void ConcentricTubeSet::addTube(tube t)
{
	m_tubes.push_back(t);
}

void ConcentricTubeSet::addCurve(curve c)
{
	m_curves.push_back(c);
}

void ConcentricTubeSet::clear()
{
	m_tubes.clear();
	m_curves.clear();
}

ConcentricTubeSet::tube ConcentricTubeSet::getTube(int i)
{
	tube t = m_tubes[i];
	return t;
}

// compute rotation matrix
cMatrix3d ConcentricTubeSet::computeZRot(float psi)
{													
	Rz = cMatrix3d(cos(psi), -sin(psi), 0,
				   sin(psi), cos(psi), 0,
				   0, 0, 1);

	return Rz;
}

// compute derivative wrt psi of rotation matrix
cMatrix3d ConcentricTubeSet::computeZRotDeriv(float psi)
{
	RzPrime = cMatrix3d(-sin(psi), -cos(psi), 0,
					     cos(psi), -sin(psi), 0,
						 0, 0, 0);

	return RzPrime;
}

// compute skew symmetric matrix
cMatrix3d ConcentricTubeSet::computeVecHat(cVector3d uB)
{
	float uBx = uB(0);
	float uBy = uB(1);
	float uBz = uB(2);
	uBHat = cMatrix3d(0, -uBz, uBy,
					  uBz, 0, -uBx,
					  -uBy, uBx, 0);

	/*uBHat = cMatrix3d(0, uBz, -uBy,
					  -uBz, 0, uBx,
					  uBy, -uBx, 0);*/

	return uBHat;
}

// compute bishop curvature (function of arc length)
cVector3d ConcentricTubeSet::computeBishopCurvature()
{
	float tubeSum1 = 0;
	float tubeSum2 = 0;
	float KSum = 0;
	//stateVec state;
	for(int i=0; i<(m_tubes.size()); i++) {
		//cMatrix3d Rzi = computeZRot(state[i]);
		cMatrix3d Rzi = computeZRot(stateV.psiVec[i]);					// Rotation matrix of tube i		
		tubeSum1 = tubeSum1 + m_tubes[i].kb*Rzi(0,0)*m_tubes[i].kappa;
		tubeSum2 = tubeSum2 + m_tubes[i].kb*Rzi(1,0)*m_tubes[i].kappa;
		KSum = KSum + m_tubes[i].kb;
	}
	uB = cVector3d((1/KSum)*tubeSum1,(1/KSum)*tubeSum2,0);

	return uB;
}

// compute stiffnesses kb and kt
void ConcentricTubeSet::computeStiffnesses() 
{
	float I;
	float J;
	float G;
	for(int i=0; i<(m_tubes.size()); i++) {
		I = (C_PI/64)*(pow(m_tubes[i].OD,4)-pow(m_tubes[i].ID,4));
		J = 2*I;
		G = m_tubes[i].E/(2*(1+m_tubes[i].v));
		m_tubes[i].kb = m_tubes[i].E*I;
		m_tubes[i].kt = G*J;
	}
}

// compute list of discontinuities in ascending order
std::vector<float> ConcentricTubeSet::computeDiscontinuities() 
{
	discontinuitiesList.erase(discontinuitiesList.begin(),discontinuitiesList.end());		// clear contents of vector
	discontinuitiesList.push_back(0.0);
	for(int i=0; i<(m_tubes.size()); i++) {
		tube currentTube = m_tubes[i];
		discontinuitiesList.push_back(currentTube.Beta + currentTube.Ls);
		discontinuitiesList.push_back(currentTube.Beta + currentTube.Ls + currentTube.Lc);
	}
	std::sort(discontinuitiesList.begin(),discontinuitiesList.end());
	std::vector<float>::iterator uniqueList;
	uniqueList = std::unique(discontinuitiesList.begin(),discontinuitiesList.end());
	discontinuitiesList.resize(std::distance(discontinuitiesList.begin(),uniqueList));


	return discontinuitiesList;
	
}

// compute intervals and midpoints
void ConcentricTubeSet::computeIntervals(vector<float> discontinuitiesList) 
{
	intAndMid.intervalLow.erase(intAndMid.intervalLow.begin(), intAndMid.intervalLow.end());
	intAndMid.intervalHigh.erase(intAndMid.intervalHigh.begin(), intAndMid.intervalHigh.end());
	intAndMid.midpoint.erase(intAndMid.midpoint.begin(), intAndMid.midpoint.end());
	for(int i=0; i<discontinuitiesList.size()-1; i++) {
		intAndMid.intervalLow.push_back(discontinuitiesList[i]);
		intAndMid.intervalHigh.push_back(discontinuitiesList[i+1]);
		intAndMid.midpoint.push_back((intAndMid.intervalLow[i] + intAndMid.intervalHigh[i])/2);
	}
}

// compute kb and 1/kt based on intervals
void ConcentricTubeSet::computeKbKt(float midpointValue)
{
	float sigma;
	for(int i=0; i<(m_tubes.size()); i++) {
		sigma = midpointValue - m_tubes[i].Beta;
		if((sigma<=((m_tubes[i].Ls) + (m_tubes[i].Lc))) && (sigma>=0)) {			// if along length of the tube	
			m_tubes[i].ktInv = 1/(m_tubes[i].kt);
			m_tubes[i].kbNew = m_tubes[i].kb;
		} else {																// if outside interval of tube
			m_tubes[i].ktInv = 0;
			m_tubes[i].kbNew = 0;
		}
	}
}

// compute kappa based on intervals
void ConcentricTubeSet::computeKappa(float midpointValue) 
{
	float sigma;
	for(int i=0; i<(m_tubes.size()); i++) {

		//tube* currentTube = &(m_tubes[i]);
		sigma = midpointValue - m_tubes[i].Beta;
		if(sigma<(m_tubes[i].Ls)) {
			m_tubes[i].kappaNew = 0;
		} else {
			m_tubes[i].kappaNew = m_tubes[i].kappa;
		}
	}
}

void ConcentricTubeSet::convertStateDerivToDouble(void) {
	int ind = 0;
	for(int i=0; i<numTubes; i++) {
		stateDeriv[i] = (double)stateVDot.psiDot[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes); i++) {
		stateDeriv[i] = (double)stateVDot.momentDot[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes + 3); i++) {
		stateDeriv[i] = (double)stateVDot.posDot[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes + 12); i++) {
		stateDeriv[i] = (double)stateVDot.rotDot[i];
		ind = ind + 1;
	}
}

void ConcentricTubeSet::convertStateToDouble(void) {
	int ind = 0;
	for(int i=0; i<numTubes; i++) {
		state[i] = (double)stateV.psiVec[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes); i++) {
		state[i] = (double)stateV.moment[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes + 3); i++) {
		state[i] = (double)stateV.pos[i];
		ind = ind + 1;
	}
	for(int i=ind; i<(2*numTubes + 12); i++) {
		state[i] = (double)stateV.rot[i];
		ind = ind + 1;
	}
}




////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////// SOLVE FORWARD KINEMATICS /////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

void kinematics(ConcentricTubeSet &set) 
{
	//---------------------------------------------------------------------------------
	//-------------------------------- Set parameters ---------------------------------
	//---------------------------------------------------------------------------------
	numTubes = set.m_tubes.size();
	numStates = 2*numTubes + 12;

	set.computeStiffnesses();

	// allocate memory
	set.initConditions = new float[numStates];
	set.stateV.psiVec = new float[numTubes];
	set.stateV.moment = new float[numTubes];
	set.stateVDot.psiDot = new float[numTubes];
	set.stateVDot.momentDot = new float[numTubes];
	set.state = new double[numStates];
	set.stateDeriv = new double[numStates];
	solvedState = new float[numTubes];

	set.gInv = gsl_matrix_alloc(4,4);
	set.Eq = gsl_matrix_alloc(6,2*numTubes);
	set.Bq = gsl_matrix_alloc(numTubes,2*numTubes);
	set.Eu = gsl_matrix_alloc(6,numTubes);
	set.Bu = gsl_matrix_alloc(numTubes,numTubes);
	set.Jb = gsl_matrix_alloc(6,2*numTubes);

	// initially assume that set is valid
	set.isValidSet = true;
	
	//---------------------------------------------------------------------------------
	//---------------- First solve using a random initial guess -----------------------
	//---------------------------------------------------------------------------------
	solveInitConditions(set);

	//---------------------------------------------------------------------------------
	//---------- Then solve using correct initial guess to get actual states ----------
	//---------------------------------------------------------------------------------
	solveForwardKinematics(set);

	//for(int i=0; i<numTubes; i++) {
	//	set.m_tubes[i].moment_guess = solvedState[i];
	//}

	//---------------------------------------------------------------------------------
	//----------------------------- Compute the Jacobian ------------------------------
	//---------------------------------------------------------------------------------
	//gsl_matrix* gTemp = gsl_matrix_alloc(4,4);
	//gTemp = computeG(set);

	//set.gInv = computeGInv(gTemp);
	//gsl_vector* qBar = gsl_vector_alloc(2*numTubes);
	//gsl_vector* uBar = gsl_vector_alloc(numTubes);
	//
	//for(int i=0; i<numTubes; i++) {
	//	gsl_vector_set(qBar,i,set.m_tubes[i].alpha);
	//	gsl_vector_set(qBar,i+numTubes,set.m_tubes[i].Beta);
	//	gsl_vector_set(uBar,i,set.m_tubes[i].moment_guess);
	//}

	////printf("qBar \n");
	////printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_vector_get(qBar,0),gsl_vector_get(qBar,1),gsl_vector_get(qBar,2),gsl_vector_get(qBar,3),gsl_vector_get(qBar,4),gsl_vector_get(qBar,5));
	////printf("uBar \n");
	////printf("%f \t %f \t %f \n", gsl_vector_get(uBar,0),gsl_vector_get(uBar,1),gsl_vector_get(uBar,2));
 //
	//computeEq(set, qBar, uBar);
	//computeEu(set, qBar, uBar);

	//gsl_matrix * BuInv = gsl_matrix_alloc(numTubes,numTubes);
 //   int s;
 //   gsl_permutation * p = gsl_permutation_alloc (numTubes);
 //   gsl_linalg_LU_decomp (set.Bu, p, &s); // Bu is always nxn (factorize Bu into LU decomposition
 //   gsl_linalg_LU_invert (set.Bu, p, BuInv);
 //   
 //   gsl_permutation_free (p);
 //   gsl_matrix * prod1 = gsl_matrix_alloc(6,numTubes);
 //   gsl_matrix * prod2 = gsl_matrix_alloc(6,2*numTubes);
 //   
 //   
 //   gsl_linalg_matmult(set.Eu,BuInv,prod1);
 //   gsl_linalg_matmult(prod1,set.Bq,prod2);
 //   gsl_matrix_memcpy (set.Jb, set.Eq);     // copy values of Eq to Jb
 //   gsl_matrix_sub(set.Jb,prod2);        // NOTE: INITIALIZE JB TO BE GLOBAL VARIABLE
 //   
 //   printf("Jb \n");
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,0,0),gsl_matrix_get(set.Jb,0,1),gsl_matrix_get(set.Jb,0,2),gsl_matrix_get(set.Jb,0,3),gsl_matrix_get(set.Jb,0,4),gsl_matrix_get(set.Jb,0,5));
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,1,0),gsl_matrix_get(set.Jb,1,1),gsl_matrix_get(set.Jb,1,2),gsl_matrix_get(set.Jb,1,3),gsl_matrix_get(set.Jb,1,4),gsl_matrix_get(set.Jb,1,5));
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,2,0),gsl_matrix_get(set.Jb,2,1),gsl_matrix_get(set.Jb,2,2),gsl_matrix_get(set.Jb,2,3),gsl_matrix_get(set.Jb,2,4),gsl_matrix_get(set.Jb,2,5));
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,3,0),gsl_matrix_get(set.Jb,3,1),gsl_matrix_get(set.Jb,3,2),gsl_matrix_get(set.Jb,3,3),gsl_matrix_get(set.Jb,3,4),gsl_matrix_get(set.Jb,3,5));
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,4,0),gsl_matrix_get(set.Jb,4,1),gsl_matrix_get(set.Jb,4,2),gsl_matrix_get(set.Jb,4,3),gsl_matrix_get(set.Jb,4,4),gsl_matrix_get(set.Jb,4,5));
 //   printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Jb,5,0),gsl_matrix_get(set.Jb,5,1),gsl_matrix_get(set.Jb,5,2),gsl_matrix_get(set.Jb,5,3),gsl_matrix_get(set.Jb,5,4),gsl_matrix_get(set.Jb,5,5));
 //   


}

////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// SOLVE WITH CORRECT INITIAL GUESS /////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////

int solveForwardKinematics(ConcentricTubeSet &set) 
{
	//---------------------------------------------------------------------------------
	//---------- Solve using correct initial guess to get actual states ---------------
	//---------------------------------------------------------------------------------

	int n = set.m_tubes.size();											// number of tubes
	int numStates = 12 + 2*n;										    // number of states
	// Clear all previous states
	set.angleStored.tube1Angle.clear();
	set.angleStored.tube2Angle.clear();
	set.angleStored.tube3Angle.clear();
	set.angleStored.tube4Angle.clear();
	set.angleStored.tube5Angle.clear();
	set.angleStored.tube6Angle.clear();
	set.momentStored.tube1Moment.clear();
	set.momentStored.tube2Moment.clear();
	set.momentStored.tube3Moment.clear();
	set.momentStored.tube4Moment.clear();
	set.momentStored.tube5Moment.clear();
	set.momentStored.tube6Moment.clear();
	//set.momentStored.clear();

	set.positionStored.x.clear();
	set.positionStored.y.clear();
	set.positionStored.z.clear();
	set.rotationStored.RB1.clear();
	set.rotationStored.RB2.clear();
	set.rotationStored.RB3.clear();
	set.rotationStored.RB4.clear();
	set.rotationStored.RB5.clear();
	set.rotationStored.RB6.clear();
	set.rotationStored.RB7.clear();
	set.rotationStored.RB8.clear();
	set.rotationStored.RB9.clear();
	set.sStored.clear();

	//----------------- Compute discontinuities and intervals -------------------------
	set.discontinuitiesList = set.computeDiscontinuities();
	set.computeIntervals(set.discontinuitiesList);

	//------- For the first time through, use the following initial conditions --------
	float midpointValue = set.intAndMid.midpoint[0];
	set.computeKbKt(midpointValue);
	set.computeKappa(midpointValue);
	vector <double> sVec;
	sVec.erase(sVec.begin(),sVec.end());
	sVec.push_back(set.intAndMid.intervalLow[0]);
	sVec.push_back(set.intAndMid.intervalHigh[0]);
	//----------------------- Set initial conditions ----------------------------------
	for(int i=0; i<n;i++) {				
		set.initConditions[i+n] = set.m_tubes[i].moment_guess;
		set.initConditions[i] = (set.m_tubes[i].alpha - set.m_tubes[i].Beta*set.m_tubes[i].ktInv*set.m_tubes[i].moment_guess);	
	}
	set.Pb_0 = cVector3d(0,0,0);
	set.initConditions[2*n] = set.Pb_0(0);				// x position of robot at s=0
	set.initConditions[2*n+1] = set.Pb_0(1);			// y position of robot at s=0
	set.initConditions[2*n+2] = set.Pb_0(2);			// z position of robot at s=0
	set.Rb_0 = cMatrix3d(1,0,0,0,1,0,0,0,1);
	int iter = 0;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			set.initConditions[2*n+3+iter] = set.Rb_0(i,j);		    // rotation of robot
			iter = iter + 1;
		}
	}
	//printf ("psi init: %f psi init: %f psi init: %f\n",  set.initConditions[0], set.initConditions[1], set.initConditions[2]);
	//printf ("pos init: %f pos init: %f pos init: %f\n",  set.initConditions[3], set.initConditions[4], set.initConditions[5]);
	//printf ("mom init: %f mom init: %f mom init: %f\n",  set.initConditions[6], set.initConditions[7], set.initConditions[8]);
	//printf ("rot init: %f rot init: %f rot init: %f\n",  set.initConditions[9], set.initConditions[10], set.initConditions[11]);
	//printf ("rot init: %f rot init: %f rot init: %f\n",  set.initConditions[12], set.initConditions[13], set.initConditions[14]);
	//printf ("rot init: %f rot init: %f rot init: %f\n",  set.initConditions[15], set.initConditions[16], set.initConditions[17]);
	
	//--------------- Set up ODE to solve with initial conditions --------------------
	
	const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
	gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);

	// ************ testing ****************
	/*gsl_odeiv_control * c = gsl_odeiv_control_y_new (1e-6, 0.0);
    gsl_odeiv_evolve * e = gsl_odeiv_evolve_alloc (numStates);*/
	// *************************************

	gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};				// set up system
	double s = sVec[0], s1 = sVec[1];
    double h = 5e-4; //51e-4;
	double *y; double *y_err;
	y_err = new double[numStates];
	y = new double[numStates];
	for(int i=0; i<numStates; i++) {
		y[i] = set.initConditions[i];									// set initial conditions
	}
	double *dyds_in; double *dyds_out;
	dyds_in = new double[numStates];
	dyds_out = new double[numStates];
    GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);								// initialize dyds_in from system parameters 

	while (s < s1) {
        int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);

		// ************ testing ****************
		/*int status = gsl_odeiv_evolve_apply (e, c, step,
                                           &sys, 
                                           &s, s1,
                                           &h, y);*/
		// *************************************

		if (status != GSL_SUCCESS)
            break;

		for(int i=0; i<numStates; i++) {
			dyds_in[i] = dyds_out[i];
			//printf("y value: %f \n", y[i]);
		}

		//--------------------- STORE STATES -------------------------
		for(int i=0; i<n; i++) {							// store angle and moment states (depending on how many tubes)
			if(i<1) {
				set.angleStored.tube1Angle.push_back(y[0]);
				set.momentStored.tube1Moment.push_back(y[n]);
			} else if(i<2) {
				set.angleStored.tube2Angle.push_back(y[1]);
				set.momentStored.tube2Moment.push_back(y[n+1]);
			} else if(i<3) {
				set.angleStored.tube3Angle.push_back(y[2]);
				set.momentStored.tube3Moment.push_back(y[n+2]);
			} else if(i<4) {
				set.angleStored.tube4Angle.push_back(y[3]);
				set.momentStored.tube4Moment.push_back(y[n+3]);
			} else if(i<5) {
				set.angleStored.tube5Angle.push_back(y[4]);
				set.momentStored.tube5Moment.push_back(y[n+4]);
			} else if(i<6) {
				set.angleStored.tube6Angle.push_back(y[5]);
				set.momentStored.tube6Moment.push_back(y[n+5]);
			}
		}
		//set.momentStored.push_back(y[n]);
		//set.momentStored.push_back(y[n+1]);
		//set.momentStored.push_back(y[n+2]);

		set.positionStored.x.push_back(y[2*n]);
		set.positionStored.y.push_back(y[2*n+1]);
		set.positionStored.z.push_back(y[2*n+2]);
		set.rotationStored.RB1.push_back(y[2*n+3]);
		set.rotationStored.RB2.push_back(y[2*n+4]);
		set.rotationStored.RB3.push_back(y[2*n+5]);
		set.rotationStored.RB4.push_back(y[2*n+6]);
		set.rotationStored.RB5.push_back(y[2*n+7]);
		set.rotationStored.RB6.push_back(y[2*n+8]);
		set.rotationStored.RB7.push_back(y[2*n+9]);
		set.rotationStored.RB8.push_back(y[2*n+10]);
		set.rotationStored.RB9.push_back(y[2*n+11]);
		set.sStored.push_back(s);

		s += h;
	}

	// ************ testing ****************
	/*gsl_odeiv_evolve_free(e);
	gsl_odeiv_control_free(c);*/
    // *************************************

	gsl_odeiv_step_free (step);

	//----- For remaining intervals, use previous end state as initial condition -----
	for(int i=1; i<(set.intAndMid.midpoint.size()); i++) {

		const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
		gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);

		// ************ testing ****************
		/*gsl_odeiv_control * c = gsl_odeiv_control_y_new (1e-6, 0.0);
		gsl_odeiv_evolve * e = gsl_odeiv_evolve_alloc (numStates);*/
		// *************************************

		gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};		// set up system

		set.computeKbKt(set.intAndMid.midpoint[i]);
		set.computeKappa(set.intAndMid.midpoint[i]);
		sVec.erase(sVec.begin(),sVec.end());
		sVec.push_back(set.intAndMid.intervalLow[i]);
		sVec.push_back(set.intAndMid.intervalHigh[i]);


		delete dyds_in;
		delete dyds_out;
		dyds_in = new double[numStates];
		dyds_out = new double[numStates];

		// set initial conditions as the END of previous section
		for(int j=0; j<numStates; j++) {
			set.initConditions[j] = y[j];				// ****** does it only give the last state??******
		}
		delete y;
		delete y_err;
		y_err = new double[numStates];
		y = new double[numStates];
		for(int k=0; k<numStates; k++) {
			y[k] = set.initConditions[k];									// set initial conditions
		}

		// solve ODE
		s = sVec[0], s1 = sVec[1];
		GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);								// initialize dyds_in from system parameters 
		while (s < s1) {
			//BEGIN_TIMING(finalSolve,10);
			int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);

			// ************ testing ****************
			/*int status = gsl_odeiv_evolve_apply (e, c, step,
                                           &sys, 
                                           &s, s1,
                                           &h, y);*/
			// *************************************

			if (status != GSL_SUCCESS)
				break;

			for(int l=0; l<numStates; l++) {
				dyds_in[l] = dyds_out[l];
			}

			//--------------------- STORE STATES -------------------------
			for(int i=0; i<n; i++) {							// store angle and moment states (depending on how many tubes)
				if(i<1) {
					set.angleStored.tube1Angle.push_back(y[0]);
					set.momentStored.tube1Moment.push_back(y[n]);
				} else if(i<2) {
					set.angleStored.tube2Angle.push_back(y[1]);
					set.momentStored.tube2Moment.push_back(y[n+1]);
				} else if(i<3) {
					set.angleStored.tube3Angle.push_back(y[2]);
					set.momentStored.tube3Moment.push_back(y[n+2]);
				} else if(i<4) {
					set.angleStored.tube4Angle.push_back(y[3]);
					set.momentStored.tube4Moment.push_back(y[n+3]);
				} else if(i<5) {
					set.angleStored.tube5Angle.push_back(y[4]);
					set.momentStored.tube5Moment.push_back(y[n+4]);
				} else if(i<6) {
					set.angleStored.tube6Angle.push_back(y[5]);
					set.momentStored.tube6Moment.push_back(y[n+5]);
				}
			}
			//set.momentStored.push_back(y[n]);
			//set.momentStored.push_back(y[n+1]);
			//set.momentStored.push_back(y[n+2]);

			set.positionStored.x.push_back(y[2*n]);
			set.positionStored.y.push_back(y[2*n+1]);
			set.positionStored.z.push_back(y[2*n+2]);
			set.rotationStored.RB1.push_back(y[2*n+3]);
			set.rotationStored.RB2.push_back(y[2*n+4]);
			set.rotationStored.RB3.push_back(y[2*n+5]);
			set.rotationStored.RB4.push_back(y[2*n+6]);
			set.rotationStored.RB5.push_back(y[2*n+7]);
			set.rotationStored.RB6.push_back(y[2*n+8]);
			set.rotationStored.RB7.push_back(y[2*n+9]);
			set.rotationStored.RB8.push_back(y[2*n+10]);
			set.rotationStored.RB9.push_back(y[2*n+11]);
			set.sStored.push_back(s);


			
			s += h;
			//END_TIMING(finalSolve,10);
		}

		// ************ testing ****************
		/*gsl_odeiv_evolve_free(e);
		gsl_odeiv_control_free(c);*/
		// *************************************

		gsl_odeiv_step_free (step);


	}

	printf ("s: %f psi: %f psi: %f psi: %f\n", s, y[0], y[1], y[2]);
	printf ("moment: %f moment: %f moment: %f\n", y[3], y[4], y[5]);
	printf ("pos x: %f pos y: %f pos z: %f\n", y[6], y[7], y[8]);
	printf ("rot 1: %f rot 2: %f rot 3: %f\n", y[9], y[10], y[11]);
	printf ("rot 4: %f rot 5: %f rot 6: %f\n", y[12], y[13], y[14]);
	printf ("rot 7: %f rot 8: %f rot 9: %f\n", y[15], y[16], y[17]);
	printf("\n");


	//---------------------------------------------------------------------------------
	//--------------------- Assemble necessary output states --------------------------
	//---------------------------------------------------------------------------------
	//delete set.u;
	set.u = new float[n];							// correct initial condition for the moment at s=0
	for(int i=0; i<n; i++) {
		set.u[i] = set.m_tubes[i].moment_guess;
	}


	 return GSL_SUCCESS;
}

int solveInitConditions(ConcentricTubeSet &set) {

	//*********************** set first guess to 0 ************
	for(int i=0; i<set.m_tubes.size(); i++) {
		set.m_tubes[i].moment_guess = 0;
		
	 }

	//////////////////////////////////////////////
	//// **************** new code ****************
	//const size_t sizeResVec = set.m_tubes.size();
	//double *x_init;
	//x_init = new double[sizeResVec];
	//gsl_vector *x = gsl_vector_alloc (sizeResVec);
	//for(int i=0; i<sizeResVec; i++) {
	//	x_init[i] = set.m_tubes[i].moment_guess;
	//	gsl_vector_set (x, i, x_init[i]);
	//}

	////----------------------- Compute necessary parameters ----------------------------
	//int n = set.m_tubes.size();											// number of tubes
	//int numStates = 12 + 2*n;										    // number of states
	////----------------- Compute discontinuities and intervals -------------------------
	//set.discontinuitiesList = set.computeDiscontinuities();
	//set.computeIntervals(set.discontinuitiesList);

	////------- For the first time through, use the following initial conditions --------
	//float midpointValue = set.intAndMid.midpoint[0];
	//set.computeKbKt(midpointValue);
	//set.computeKappa(midpointValue);
	//vector <double> sVec;
	//sVec.erase(sVec.begin(),sVec.end());
	//sVec.push_back(set.intAndMid.intervalLow[0]);
	//sVec.push_back(set.intAndMid.intervalHigh[0]);

	//for(int i=0; i<n;i++) {				
	//	
	//	set.initConditions[i+n] = gsl_vector_get(x,i);
	//	set.initConditions[i] = (set.m_tubes[i].alpha - set.m_tubes[i].Beta*set.m_tubes[i].ktInv*gsl_vector_get(x,i));		

	//	//set.initConditions[i+n] = xcurr[i];
	//	//set.initConditions[i] = (set.m_tubes[i].alpha - set.m_tubes[i].Beta*set.m_tubes[i].ktInv*xcurr[i]);

	//}
	//set.Pb_0 = cVector3d(0,0,0);
	//set.initConditions[2*n] = set.Pb_0(0);				// x position of robot at s=0
	//set.initConditions[2*n+1] = set.Pb_0(1);			// y position of robot at s=0
	//set.initConditions[2*n+2] = set.Pb_0(2);			// z position of robot at s=0
	//set.Rb_0 = cMatrix3d(1,0,0,0,1,0,0,0,1);
	//int iter = 0;
	//for(int i=0; i<3; i++) {
	//	for(int j=0; j<3; j++) {
	//		set.initConditions[2*n+3+iter] = set.Rb_0(i,j);		    // rotation of robot
	//		iter = iter + 1;
	//	}
	//}

	////--------------- Set up ODE to solve with initial conditions --------------------	
	//const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
	//gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);
	//gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};		// set up system
	//double s = sVec[0], s1 = sVec[1];
 //   double h = 10e-4;//10e-4; //5e-4;												//**** trade-off between speed and accuracy... ****************
	//double *y; double *y_err;
	//y_err = new double[numStates];
	//y = new double[numStates];
	//for(int i=0; i<numStates; i++) {
	//	y[i] = set.initConditions[i];										// set initial conditions
	//}
	//double *dyds_in; double *dyds_out;
	//dyds_in = new double[numStates];
	//dyds_out = new double[numStates];
 //   GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);									// initialize dyds_in from system parameters 
	//while (s < s1) {
 //       int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);
	//	if (status != GSL_SUCCESS)
 //           break;

	//	for(int i=0; i<numStates; i++) {
	//		dyds_in[i] = dyds_out[i];
	//	}

	//	s += h;
	//}
	//gsl_odeiv_step_free (step);

	////----- For remaining intervals, use previous end state as initial condition -----
	//for(int i=1; i<(set.intAndMid.midpoint.size()); i++) {

	//	const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
	//	gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);
	//	gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};		// set up system

	//	set.computeKbKt(set.intAndMid.midpoint[i]);
	//	set.computeKappa(set.intAndMid.midpoint[i]);
	//	sVec.erase(sVec.begin(),sVec.end());
	//	sVec.push_back(set.intAndMid.intervalLow[i]);
	//	sVec.push_back(set.intAndMid.intervalHigh[i]);


	//	delete dyds_in;
	//	delete dyds_out;
	//	dyds_in = new double[numStates];
	//	dyds_out = new double[numStates];

	//	// set initial conditions as the END of previous section
	//	for(int j=0; j<numStates; j++) {
	//		set.initConditions[j] = y[j];				// ****** does it only give the last state??******
	//	}
	//	delete y;
	//	delete y_err;
	//	y_err = new double[numStates];
	//	y = new double[numStates];
	//	for(int k=0; k<numStates; k++) {
	//		y[k] = set.initConditions[k];									// set initial conditions
	//	}

	//	// solve ODE
	//	s = sVec[0], s1 = sVec[1];
	//	GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);								// initialize dyds_in from system parameters 
	//	while (s < s1) {
	//		int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);
	//		if (status != GSL_SUCCESS)
	//			break;

	//		for(int l=0; l<numStates; l++) {
	//		dyds_in[l] = dyds_out[l];
	//		}

	//		s += h;
	//	}
	//	gsl_odeiv_step_free (step);
	//}

	////////////////////////////////////////////////////
	//// ******* SET UP FUNCTION FOR SOLVER HERE *****************
	//// using output from ODE solver (y[i+n])
	////xvec.resize(numTubes);

	//Eigen::VectorXf xvec(numTubes);
	//for(int i=0; i<n; i++) {
	//	xvec(i) = y[i+n];
	//}

	//////////////////////////////////////////////////


	for(int i=0; i<numTubes; i++) {
		xvec(i) = set.m_tubes[i].moment_guess;
	}
	// xvec should be moment guess?

	

	funcToSolve func;
	func.tubeSet = &set;
	Eigen::LevenbergMarquardt<funcToSolve, float> lm(func);
	lm.minimize(xvec);



	printf("x0: %f \n", xvec(0));
	printf("x1: %f \n", xvec(1));
	printf("x2: %f \n", xvec(2));


	// Set initial moment as solved value
	for(int i=0; i<numTubes; i++){
		set.m_tubes[i].moment_guess = xvec(i);
		solvedState[i] = xvec(i);
	}

	//***** GSL code for fsolve *******
	 //const gsl_multiroot_fsolver_type *T;
	 //gsl_multiroot_fsolver *solver;
	 //int status;
	 //size_t iter = 0;	 
	 //const size_t sizeResVec = set.m_tubes.size();			
	 //gsl_multiroot_function f = {&findZeroResVec, sizeResVec, &set};

	 //double *x_init;
	 //x_init = new double[sizeResVec];
	 //gsl_vector *x = gsl_vector_alloc (sizeResVec);
	 //for(int i=0; i<sizeResVec; i++) {
		//x_init[i] = set.m_tubes[i].moment_guess;
		//gsl_vector_set (x, i, x_init[i]);
	 //}

	 //T = gsl_multiroot_fsolver_hybrids;
	 //solver = gsl_multiroot_fsolver_alloc (T, sizeResVec);
	 //gsl_multiroot_fsolver_set (solver, &f, x);



	 //do
  //   {
  //     iter++;
  //     status = gsl_multiroot_fsolver_iterate (solver);

	 //  printf("iter: %i, x: %f  %f  %f \n",iter, gsl_vector_get (solver->x, 0), gsl_vector_get (solver->x, 1),gsl_vector_get (solver->x, 2));
	 //  printf("iter: %i, f: %f  %f  %f \n",iter, gsl_vector_get (solver->f, 0), gsl_vector_get (solver->f, 1),gsl_vector_get (solver->f, 2));

	 // //  for debugging value of dx and seeing if it's NaN
	 //  printf("dx: %f \n", gsl_vector_get(solver->dx, 0));
	 //  printf("dx: %f \n", gsl_vector_get(solver->dx, 1));
	 //  printf("dx: %f \n", gsl_vector_get(solver->dx, 2));
	 //  
	 //  if(gsl_vector_get(solver->dx, 0) != gsl_vector_get(solver->dx, 0)) {
		//   printf("caught NAN case \n");
		//   set.isValidSet = false;				// set cannot be solved
		//   break;								// try using this to break out?
	 //  } else {
		//   set.isValidSet = true;
	 //  }

  //     if (status)   /* check if solver is stuck */
  //       break;

  //     status = 
  //       gsl_multiroot_test_residual (solver->f, 1e-3);				//
  //   }
	 //while (status == GSL_CONTINUE && iter < 1000);

	 //printf ("status = %s\n", gsl_strerror (status));

	 //// Set initial moment as solved value
	 //for(int i=0; i<sizeResVec; i++){
		// set.m_tubes[i].moment_guess = gsl_vector_get (solver->x, i);
		// solvedState[i] = gsl_vector_get (solver->x, i);
	 //}

	 //gsl_multiroot_fsolver_free (solver);
	 //gsl_vector_free (x);

	

	 return 0;
}

//int solveInitConditions(ConcentricTubeSet &set) {
//
//	//*********************** set first guess to 0 ************
//	for(int i=0; i<set.m_tubes.size(); i++) {
//		set.m_tubes[i].moment_guess = 0;
//		
//	 }
//
//	const gsl_multiroot_fdfsolver_type *T;
//	gsl_multiroot_fdfsolver *solver;
//
//	int status;
//	size_t iter = 0;
//	const size_t sizeResVec = set.m_tubes.size();
//	gsl_multiroot_function_fdf f = {&findZeroResVec, &findZeroResVec_df, &findZeroResVec_fdf, sizeResVec, &set};
//
//	double *x_init;
//	x_init = new double[sizeResVec];
//	gsl_vector *x = gsl_vector_alloc (sizeResVec);
//	for(int i=0; i<sizeResVec; i++) {
//		x_init[i] = set.m_tubes[i].moment_guess;
//		gsl_vector_set (x, i, x_init[i]);
//	}
//
//	T = gsl_multiroot_fdfsolver_hybridsj;
//	solver = gsl_multiroot_fdfsolver_alloc (T, sizeResVec);
//	gsl_multiroot_fdfsolver_set (solver, &f, x);
//
//	 do
//     {
//       iter++;
//       status = gsl_multiroot_fdfsolver_iterate (solver);
//
//	   printf("iter: %i, x: %f  %f  %f \n",iter, gsl_vector_get (solver->x, 0), gsl_vector_get (solver->x, 1),gsl_vector_get (solver->x, 2));
//	   printf("iter: %i, f: %f  %f  %f \n",iter, gsl_vector_get (solver->f, 0), gsl_vector_get (solver->f, 1),gsl_vector_get (solver->f, 2));
//
//       if (status)   /* check if solver is stuck */
//         break;
//
//       status = 
//         gsl_multiroot_test_residual (solver->f, 1e-6);				//
//     }
//	 while (status == GSL_CONTINUE && iter < 1000);
//
//	 printf ("status = %s\n", gsl_strerror (status));
//
//	 // Set initial moment as solved value
//	 for(int i=0; i<sizeResVec; i++){
//		 set.m_tubes[i].moment_guess = gsl_vector_get (solver->x, i);
//		 solvedState[i] = gsl_vector_get (solver->x, i);
//	 }
//
//	 gsl_multiroot_fdfsolver_free (solver);
//	 gsl_vector_free (x);
//
//
//
//	 return 0;
//}



static int numIts = 0;
//static double xprev[3] = {0};
#define MAX(x,y) ((x)>(y) ? (x) : (y))
int findZeroResVec (const gsl_vector * x, void *params, gsl_vector * f)
{
	ConcentricTubeSet set = *(ConcentricTubeSet *)params;				// recast
	
	//----------------------- Compute necessary parameters ----------------------------
	int n = set.m_tubes.size();											// number of tubes
	int numStates = 12 + 2*n;										    // number of states
	//----------------- Compute discontinuities and intervals -------------------------
	set.discontinuitiesList = set.computeDiscontinuities();
	set.computeIntervals(set.discontinuitiesList);

	//------- For the first time through, use the following initial conditions --------
	float midpointValue = set.intAndMid.midpoint[0];
	set.computeKbKt(midpointValue);
	set.computeKappa(midpointValue);
	vector <double> sVec;
	sVec.erase(sVec.begin(),sVec.end());
	sVec.push_back(set.intAndMid.intervalLow[0]);
	sVec.push_back(set.intAndMid.intervalHigh[0]);

	///////////////////////////////////////////////
	// FIX FOR GETTING PAST NAN OF FSOLVE
	//double xcurr[3] = {0};
	//if(numIts < 4) {
	//	xcurr[0] = 5*gsl_vector_get(x, 0);
	//	xcurr[1] = 5*gsl_vector_get(x, 1);
	//	xcurr[2] = 5*gsl_vector_get(x, 2);
	//} else {
	//	xcurr[0] = gsl_vector_get(x, 0);
	//	xcurr[1] = gsl_vector_get(x, 1);
	//	xcurr[2] = gsl_vector_get(x, 2);
	//	//if(abs(xcurr[0]-xprev[0])+abs(xcurr[1]-xprev[1])+abs(xcurr[2]-xprev[2]) < 1e-3)  {
	//	//	xcurr[0] = xprev[0]+10*(xcurr[0]-xprev[0]);
	//	//	xcurr[1] = xprev[1]+10*(xcurr[1]-xprev[1]);
	//	//	xcurr[2] = xprev[2]+10*(xcurr[2]-xprev[2]);
	//	//}
	//}
	////// doesn't seem like this part is being used anymore?
	////xprev[0] = gsl_vector_get(x, 0);
	////xprev[1] = gsl_vector_get(x, 1);
	////xprev[2] = gsl_vector_get(x, 2);

	//numIts++;
	//////////////////////////////////////////////////
	//----------------------- Set initial conditions ----------------------------------
	for(int i=0; i<n;i++) {				
		
		set.initConditions[i+n] = gsl_vector_get(x,i);
		set.initConditions[i] = (set.m_tubes[i].alpha - set.m_tubes[i].Beta*set.m_tubes[i].ktInv*gsl_vector_get(x,i));		

		//set.initConditions[i+n] = xcurr[i];
		//set.initConditions[i] = (set.m_tubes[i].alpha - set.m_tubes[i].Beta*set.m_tubes[i].ktInv*xcurr[i]);

	}
	set.Pb_0 = cVector3d(0,0,0);
	set.initConditions[2*n] = set.Pb_0(0);				// x position of robot at s=0
	set.initConditions[2*n+1] = set.Pb_0(1);			// y position of robot at s=0
	set.initConditions[2*n+2] = set.Pb_0(2);			// z position of robot at s=0
	set.Rb_0 = cMatrix3d(1,0,0,0,1,0,0,0,1);
	int iter = 0;
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			set.initConditions[2*n+3+iter] = set.Rb_0(i,j);		    // rotation of robot
			iter = iter + 1;
		}
	}

	//--------------- Set up ODE to solve with initial conditions --------------------	
	const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
	gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);

	// ************ testing ****************
	gsl_odeiv_control * c = gsl_odeiv_control_y_new (1e-6, 0.0);
    gsl_odeiv_evolve * e = gsl_odeiv_evolve_alloc (numStates);
	// *************************************

	gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};		// set up system
	double s = sVec[0], s1 = sVec[1];
    double h = 10e-4;//10e-4; //5e-4;												//**** trade-off between speed and accuracy... ****************
	double *y; double *y_err;
	y_err = new double[numStates];
	y = new double[numStates];
	for(int i=0; i<numStates; i++) {
		y[i] = set.initConditions[i];										// set initial conditions
	}
	/*double *dyds_in; double *dyds_out;
	dyds_in = new double[numStates];
	dyds_out = new double[numStates];
    GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);	*/								// initialize dyds_in from system parameters 


	while (s < s1) {
		//BEGIN_TIMING(kinematics,10);
        //int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);
		// ************ testing ****************
		int status = gsl_odeiv_evolve_apply (e, c, step,
                                           &sys, 
                                           &s, s1,
                                           &h, y);
		// *************************************


		if (status != GSL_SUCCESS)
            break;

		/*for(int i=0; i<numStates; i++) {
			dyds_in[i] = dyds_out[i];
		}*/

		//s += h;
		//END_TIMING(kinematics,10);
	}

	// ************ testing ****************
	gsl_odeiv_evolve_free(e);
	gsl_odeiv_control_free(c);
    // *************************************

	gsl_odeiv_step_free (step);



	//----- For remaining intervals, use previous end state as initial condition -----
	for(int i=1; i<(set.intAndMid.midpoint.size()); i++) {

		const gsl_odeiv_step_type * T  = gsl_odeiv_step_rkf45;					// Step type (rk 45)
		gsl_odeiv_step * step = gsl_odeiv_step_alloc (T, numStates);
		
		// ************ testing ****************
		gsl_odeiv_control * c = gsl_odeiv_control_y_new (1e-6, 0.0);
		gsl_odeiv_evolve * e = gsl_odeiv_evolve_alloc (numStates);
		// *************************************

		gsl_odeiv_system sys = {kinematicFunction, jac, numStates, &set};		// set up system

		set.computeKbKt(set.intAndMid.midpoint[i]);
		set.computeKappa(set.intAndMid.midpoint[i]);
		sVec.erase(sVec.begin(),sVec.end());
		sVec.push_back(set.intAndMid.intervalLow[i]);
		sVec.push_back(set.intAndMid.intervalHigh[i]);


		/*delete dyds_in;
		delete dyds_out;
		dyds_in = new double[numStates];
		dyds_out = new double[numStates];*/

		// set initial conditions as the END of previous section
		for(int j=0; j<numStates; j++) {
			set.initConditions[j] = y[j];				// ****** does it only give the last state??******
		}
		delete y;
		delete y_err;
		y_err = new double[numStates];
		y = new double[numStates];
		for(int k=0; k<numStates; k++) {
			y[k] = set.initConditions[k];									// set initial conditions
		}

		// solve ODE
		s = sVec[0], s1 = sVec[1];
		/*GSL_ODEIV_FN_EVAL(&sys, s, y, dyds_in);*/								// initialize dyds_in from system parameters 
		while (s < s1) {
			//BEGIN_TIMING(kinematics_second,10);
			//int status = gsl_odeiv_step_apply (step, s, h, y, y_err, dyds_in, dyds_out, &sys);

			// ************ testing ****************
			int status = gsl_odeiv_evolve_apply (e, c, step,
                                           &sys, 
                                           &s, s1,
                                           &h, y);
			// *************************************

			if (status != GSL_SUCCESS)
				break;

			/*for(int l=0; l<numStates; l++) {
			dyds_in[l] = dyds_out[l];
			}

			s += h;*/
			//END_TIMING(kinematics_second,10);
		}

		// ************ testing ****************
		gsl_odeiv_evolve_free(e);
		gsl_odeiv_control_free(c);
		// *************************************

		gsl_odeiv_step_free (step);
	}

	//printf("release mode test: solved through once \n");

	//// for debugging
	//printf ("ODE solution for a given initial guess \n");
	//printf ("s: %f psi: %f psi: %f psi: %f\n", s, y[0], y[1], y[2]);
	//printf ("moment: %f moment: %f moment: %f\n", y[3], y[4], y[5]);
	//printf ("pos x: %f pos y: %f pos z: %f\n", y[6], y[7], y[8]);
	//printf ("rot 1: %f rot 2: %f rot 3: %f\n", y[9], y[10], y[11]);
	//printf ("rot 4: %f rot 5: %f rot 6: %f\n", y[12], y[13], y[14]);
	//printf ("rot 7: %f rot 8: %f rot 9: %f\n", y[15], y[16], y[17]);
	//printf("\n");


	//-------------------------- Set up function for fsolver ----------------------
	for(int i=0; i<n; i++) {
		
		gsl_vector_set (f, i, y[i+n]);     // x is equal to the states (y from ODE solver) and f is the moment at end of each tube
		//printf("\n");
		//printf("moment used for solver: %f \n", y[i+n]);
		//printf("\n");
	}




	return GSL_SUCCESS;
	
}

//int findZeroResVec_df (const gsl_vector * x, void *params, gsl_matrix *J) {
//	ConcentricTubeSet set = *(ConcentricTubeSet *)params;				// recast
//	int numTubes = set.m_tubes.size();									// number of tubes
//
//	// Must solve through kinematics first
//	solveForwardKinematics(set);
//
//	// Then can compute Jacobian
//	gsl_matrix* gTemp = gsl_matrix_alloc(4,4);
//	gTemp = computeG(set);
//	set.gInv = computeGInv(gTemp);
//	gsl_vector* qBar = gsl_vector_alloc(2*numTubes);
//	gsl_vector* uBar = gsl_vector_alloc(numTubes);
//	
//	for(int i=0; i<numTubes; i++) {
//		gsl_vector_set(qBar,i,set.m_tubes[i].alpha);
//		gsl_vector_set(qBar,i+numTubes,set.m_tubes[i].Beta);
//		gsl_vector_set(uBar,i,set.m_tubes[i].moment_guess);
//	}
//
//	computeEq(set, qBar, uBar);
//	computeEu(set, qBar, uBar);
//
//	gsl_matrix * BuInv = gsl_matrix_alloc(numTubes,numTubes);
//    int s;
//    gsl_permutation * p = gsl_permutation_alloc (numTubes);
//    gsl_linalg_LU_decomp (set.Bu, p, &s); // Bu is always nxn (factorize Bu into LU decomposition
//    gsl_linalg_LU_invert (set.Bu, p, BuInv);
//    
//    gsl_permutation_free (p);
//    gsl_matrix * prod1 = gsl_matrix_alloc(6,numTubes);
//    gsl_matrix * prod2 = gsl_matrix_alloc(6,2*numTubes);
//    
//    
//    gsl_linalg_matmult(set.Eu,BuInv,prod1);
//    gsl_linalg_matmult(prod1,set.Bq,prod2);
//	gsl_matrix_sub(set.Eq,prod2);
//	for(int i=0; i<6; i++) {
//		for(int j=0; j<(2*numTubes); j++) {
//			gsl_matrix_set(J,i,j,gsl_matrix_get(set.Eq,i,j));
//		} 
//	}
//
//    //gsl_matrix_memcpy (J, set.Eq);     // copy values of Eq to Jb
//    //gsl_matrix_sub(J,prod2);        // NOTE: INITIALIZE JB TO BE GLOBAL VARIABLE
//
//	return GSL_SUCCESS;
//}
//
//int findZeroResVec_fdf (const gsl_vector *x, void *params, gsl_vector *f, gsl_matrix *J) {
//	findZeroResVec(x,params,f);
//	findZeroResVec_df(x,params,J);
//
//	return GSL_SUCCESS;
//}

// Set up ODE
int kinematicFunction (double s, const double y[], double dyds[], void *params)
{

	ConcentricTubeSet set = *(ConcentricTubeSet *)params;				// recast

	int n = set.m_tubes.size();											// number of tubes

	//printf("release mode test: in kinematicFunction \n");

	// Compute Bishop Curvature 
	float tubeSum1 = 0;
	float tubeSum2 = 0;
	float KSum = 0;
	for(int i=0; i<n; i++) {
		cMatrix3d Rzi = set.computeZRot(y[i]);							// Rotation matrix of tube i		
		tubeSum1 = tubeSum1 + (set.m_tubes[i].kbNew)*Rzi(0,0)*(set.m_tubes[i].kappaNew);		
		tubeSum2 = tubeSum2 + (set.m_tubes[i].kbNew)*Rzi(1,0)*(set.m_tubes[i].kappaNew);
		KSum = KSum + (set.m_tubes[i].kbNew);
	}
	set.uB = cVector3d((1/KSum)*tubeSum1,(1/KSum)*tubeSum2,0);
	set.uBHat = set.computeVecHat(set.uB);
	
	// for i=0 to n-1 (derivative of psi state)
	for(int i=0; i<n;i++) {
		dyds[i] = (set.m_tubes[i].ktInv)*y[n+i];
	}

	// for i=n to 2n-1 (derivative of moment state)
	for(int i=n; i<(2*n); i++) {
		cMatrix3d RzDot = set.computeZRotDeriv(y[i-n]);
		dyds[i] = -(set.m_tubes[i-n].kbNew)*(set.m_tubes[i-n].kappaNew)*(set.uB(0)*RzDot(0,0) + set.uB(1)*RzDot(1,0));
	}

	// for i=2n to 2n+2 (derivative of position state)
	dyds[2*n] = y[2*n+5];
	dyds[2*n+1] = y[2*n+8];
	dyds[2*n+2] = y[2*n+11];

	// for i=2n+3 to 2n+11 (derivative of rotation state)
	dyds[2*n+3] = y[2*n+3]*set.uBHat(0,0) + y[2*n+4]*set.uBHat(1,0) + y[2*n+5]*set.uBHat(2,0);
	dyds[2*n+4] = y[2*n+3]*set.uBHat(0,1) + y[2*n+4]*set.uBHat(1,1) + y[2*n+5]*set.uBHat(2,1);
	dyds[2*n+5] = y[2*n+3]*set.uBHat(0,2) + y[2*n+4]*set.uBHat(1,2) + y[2*n+5]*set.uBHat(2,2);
	dyds[2*n+6] = y[2*n+6]*set.uBHat(0,0) + y[2*n+7]*set.uBHat(1,0) + y[2*n+8]*set.uBHat(2,0);
	dyds[2*n+7] = y[2*n+6]*set.uBHat(0,1) + y[2*n+7]*set.uBHat(1,1) + y[2*n+8]*set.uBHat(2,1);
	dyds[2*n+8] = y[2*n+6]*set.uBHat(0,2) + y[2*n+7]*set.uBHat(1,2) + y[2*n+8]*set.uBHat(2,2);
	dyds[2*n+9] = y[2*n+9]*set.uBHat(0,0) + y[2*n+10]*set.uBHat(1,0) + y[2*n+11]*set.uBHat(2,0);
	dyds[2*n+10] = y[2*n+9]*set.uBHat(0,1) + y[2*n+10]*set.uBHat(1,1) + y[2*n+11]*set.uBHat(2,1);
	dyds[2*n+11] = y[2*n+9]*set.uBHat(0,2) + y[2*n+10]*set.uBHat(1,2) + y[2*n+11]*set.uBHat(2,2);

	//// for debugging
	//printf ("state vec dot \n");
	//printf ("psi deriv: %f psi deriv: %f psi deriv: %f\n",  dyds[0], dyds[1], dyds[2]);
	//printf ("mom deriv: %f mom deriv: %f mom deriv: %f\n",  dyds[3], dyds[4], dyds[5]);
	//printf ("pos deriv: %f pos deriv: %f pos deriv: %f\n",  dyds[6], dyds[7], dyds[8]);
	//printf ("rot deriv: %f rot deriv: %f rot deriv: %f\n",  dyds[9], dyds[10], dyds[11]);
	//printf ("rot deriv: %f rot deriv: %f rot deriv: %f\n",  dyds[12], dyds[13], dyds[14]);
	//printf ("rot deriv: %f rot deriv: %f rot deriv: %f\n",  dyds[15], dyds[16], dyds[17]);
	//printf("\n");

	return GSL_SUCCESS;

}

int jac (double t, const double y[], double *dfdy, double dfdt[], void *params)
{


	return GSL_SUCCESS;
}


//********** new code for computing jacobian ***********

// Compute g
gsl_matrix* computeG(ConcentricTubeSet &set) {
    int numPtsInnerTube = set.angleStored.tube1Angle.size();
    float tipAngle = set.angleStored.tube1Angle[numPtsInnerTube-1];      // angle of inner most tube
    
    int numPts = set.rotationStored.RB1.size();
    cMatrix3d RB = cMatrix3d(set.rotationStored.RB1[numPts-1], set.rotationStored.RB2[numPts-1], set.rotationStored.RB3[numPts-1],
                             set.rotationStored.RB4[numPts-1], set.rotationStored.RB5[numPts-1], set.rotationStored.RB6[numPts-1],
                             set.rotationStored.RB7[numPts-1], set.rotationStored.RB8[numPts-1], set.rotationStored.RB9[numPts-1]);
    cMatrix3d Rz_tube1 = set.computeZRot(tipAngle);
    cMatrix3d Rzi_tube1 = cMul(RB,Rz_tube1);
    
    cVector3d tipPos = cVector3d(set.positionStored.x[numPts-1], set.positionStored.y[numPts-1], set.positionStored.z[numPts-1]);
    
    gsl_matrix* g = gsl_matrix_alloc(4,4);
    // set rotation part of matrix
    for(int i=0; i<3; i++) {
        for(int j=0; j<3; j++) {
			gsl_matrix_set(g,i,j,Rzi_tube1.getRow(i)(j));
        }
    }
    // set translation part of matrix
    gsl_matrix_set(g,0,3,tipPos(0));
    gsl_matrix_set(g,1,3,tipPos(1));
    gsl_matrix_set(g,2,3,tipPos(2));
    // set final row of matrix
    gsl_matrix_set(g,3,0,0);
    gsl_matrix_set(g,3,1,0);
    gsl_matrix_set(g,3,2,0);
    gsl_matrix_set(g,3,3,1);
    
    return g;

	gsl_matrix_free(g);
}

// Compute gInv based on g (homogeneous transform computed based on solved kinematics)
gsl_matrix* computeGInv(gsl_matrix * gBar) {
	cMatrix3d rot = cMatrix3d(gsl_matrix_get(gBar,0,0),gsl_matrix_get(gBar,0,1),gsl_matrix_get(gBar,0,2),
							  gsl_matrix_get(gBar,1,0),gsl_matrix_get(gBar,1,1),gsl_matrix_get(gBar,1,2),
							  gsl_matrix_get(gBar,2,0),gsl_matrix_get(gBar,2,1),gsl_matrix_get(gBar,2,2));
	/*cMatrix3d rot = cMatrix3d(gsl_matrix_get(gBar,0,0),gsl_matrix_get(gBar,0,1),gsl_matrix_get(gBar,0,2),
							  gsl_matrix_get(gBar,1,0),gsl_matrix_get(gBar,1,1),gsl_matrix_get(gBar,1,2),
							  gsl_matrix_get(gBar,2,0),gsl_matrix_get(gBar,2,1),gsl_matrix_get(gBar,2,2));
*/
	rot = cTranspose(rot);
	cVector3d v = cVector3d(gsl_matrix_get(gBar,0,3),gsl_matrix_get(gBar,1,3),gsl_matrix_get(gBar,2,3));
	cVector3d v_new;
	v_new = -cMul(rot,v);

	gsl_matrix * gInv = gsl_matrix_alloc(4,4);
	// set rotation part of matrix
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++) {
			gsl_matrix_set(gInv,i,j,rot.getRow(i)(j));
		}
	}
	// set translation part of matrix
	gsl_matrix_set(gInv,0,3,v_new(0));
	gsl_matrix_set(gInv,1,3,v_new(1));
	gsl_matrix_set(gInv,2,3,v_new(2));
	// set final row of matrix
	gsl_matrix_set(gInv,3,0,0);
	gsl_matrix_set(gInv,3,1,0);
	gsl_matrix_set(gInv,3,2,0);
	gsl_matrix_set(gInv,3,3,1);


	//printf("gInv: \n");
	//printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(gInv,0,0), gsl_matrix_get(gInv,0,1), gsl_matrix_get(gInv,0,2), gsl_matrix_get(gInv,0,3));
	//printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(gInv,1,0), gsl_matrix_get(gInv,1,1), gsl_matrix_get(gInv,1,2), gsl_matrix_get(gInv,1,3));
	//printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(gInv,2,0), gsl_matrix_get(gInv,2,1), gsl_matrix_get(gInv,2,2), gsl_matrix_get(gInv,2,3));
	//printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(gInv,3,0), gsl_matrix_get(gInv,3,1), gsl_matrix_get(gInv,3,2), gsl_matrix_get(gInv,3,3));
	return gInv;

}


// inputs: qBar (joint variables used to solve), uBar (proximal moment guess used to solve), gBar (homogeneous transform)

// compute Eq
void computeEq(ConcentricTubeSet &set, gsl_vector* q, gsl_vector* uBar) {
	int n = set.m_tubes.size();
	int qSize = q->size;
	int numPts = set.momentStored.tube1Moment.size();	
	double h_alpha = .001; //1e-5; //0.001;			// for perturbing alpha [rad]
	double h_beta = 1e-5;			// for perturbing beta [m]
	gsl_vector * q_pert_pos = gsl_vector_alloc(qSize);
	gsl_vector * q_pert_neg = gsl_vector_alloc(qSize);
	gsl_matrix* g_pos = gsl_matrix_alloc(4,4);
    gsl_matrix* g_neg = gsl_matrix_alloc(4,4);
	gsl_matrix* EqiMatrix = gsl_matrix_alloc(4,4);
	gsl_vector* w = gsl_vector_alloc(3);
	gsl_vector* v = gsl_vector_alloc(3);

	ConcentricTubeSet posSet;
	ConcentricTubeSet negSet;
	posSet = set;
	negSet = set;

	
	for(int i=0; i<(2*n); i++) {
		// initialize all h to zero
		gsl_vector * h = gsl_vector_calloc(qSize);
		// set h for given parameter to h_alpha or h_beta
		gsl_vector_set(h,i,h_alpha);


		//printf("h: %4.8f \t %4.8f \t %4.8f \t %4.8f \t %4.8f \t %4.8f \n", 
		//	gsl_vector_get(h,0),gsl_vector_get(h,1),gsl_vector_get(h,2),
		//	gsl_vector_get(h,3),gsl_vector_get(h,4),gsl_vector_get(h,5));
		
		// initialize q_pert_pos and q_pert_neg to be q
		for(int k=0; k<qSize; k++) {
			gsl_vector_set(q_pert_pos,k,gsl_vector_get(q,k));	
			gsl_vector_set(q_pert_neg,k,gsl_vector_get(q,k));
		}
		// perturb q in pos direction and solve kinematics for perturbed parameters
		gsl_vector_add(q_pert_pos,h);			// function stores result in q_pert_pos
		for(int j=0; j<n; j++) {
			posSet.m_tubes[j].alpha = gsl_vector_get(q_pert_pos,j);
			posSet.m_tubes[j].Beta = gsl_vector_get(q_pert_pos,n+j);
			posSet.m_tubes[j].moment_guess = gsl_vector_get(uBar,j);
		}
		solveForwardKinematics(posSet);

		// perturb q in neg direction and solve kinematics for perturbed parameters
		gsl_vector_sub(q_pert_neg,h);			// function stores result in q_pert_neg
		for(int j=0; j<n; j++) {
			negSet.m_tubes[j].alpha = gsl_vector_get(q_pert_neg,j);
			negSet.m_tubes[j].Beta = gsl_vector_get(q_pert_neg,n+j);
			negSet.m_tubes[j].moment_guess = gsl_vector_get(uBar,j);
		}
		solveForwardKinematics(negSet);
        
        // Calculate g (homogeneous transform) resulting from solving kinematics
        g_pos = computeG(posSet);
        g_neg = computeG(negSet);

		//printf("g_pos: \n");
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,0,0), gsl_matrix_get(g_pos,0,1), gsl_matrix_get(g_pos,0,2), gsl_matrix_get(g_pos,0,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,1,0), gsl_matrix_get(g_pos,1,1), gsl_matrix_get(g_pos,1,2), gsl_matrix_get(g_pos,1,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,2,0), gsl_matrix_get(g_pos,2,1), gsl_matrix_get(g_pos,2,2), gsl_matrix_get(g_pos,2,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,3,0), gsl_matrix_get(g_pos,3,1), gsl_matrix_get(g_pos,3,2), gsl_matrix_get(g_pos,3,3));
		//printf("\n");

		//printf("g_neg: \n");
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_neg,0,0), gsl_matrix_get(g_neg,0,1), gsl_matrix_get(g_neg,0,2), gsl_matrix_get(g_neg,0,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_neg,1,0), gsl_matrix_get(g_neg,1,1), gsl_matrix_get(g_neg,1,2), gsl_matrix_get(g_neg,1,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_neg,2,0), gsl_matrix_get(g_neg,2,1), gsl_matrix_get(g_neg,2,2), gsl_matrix_get(g_neg,2,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_neg,3,0), gsl_matrix_get(g_neg,3,1), gsl_matrix_get(g_neg,3,2), gsl_matrix_get(g_neg,3,3));
		//printf("\n");

        // Calculate b (moment) resulting from solving kinematics
        gsl_vector* b_pos = gsl_vector_alloc(n);
        gsl_vector* b_neg = gsl_vector_alloc(n);


		int numPts_pos = posSet.momentStored.tube1Moment.size();
		int numPts_neg = negSet.momentStored.tube1Moment.size();
		for(int j=0; j<n; j++) {
			if(j<1) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube1Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube1Moment[numPts_neg-1]);
			} else if(j<2) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube2Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube2Moment[numPts_neg-1]);
			} else if(j<3) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube3Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube3Moment[numPts_neg-1]);
			} else if(j<4) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube4Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube4Moment[numPts_neg-1]);
			} else if(j<5) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube5Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube5Moment[numPts_neg-1]);
			} else if(j<6) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube6Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube6Moment[numPts_neg-1]);
			}
		}


		//printf("b_pos \n");
		//printf("%f \t %f \t %f \n", gsl_vector_get(b_pos,0),gsl_vector_get(b_pos,1),gsl_vector_get(b_pos,2));
		//printf("\n");
		//printf("b_neg \n");
		//printf("%f \t %f \t %f \n", gsl_vector_get(b_neg,0),gsl_vector_get(b_neg,1),gsl_vector_get(b_neg,2));
		//printf("\n");

		// Form Eq and Bq based on perturbed states
		//gsl_matrix_sub(g_pos,g_neg);	
        //gsl_linalg_matmult(set.gInv,g_pos,EqiMatrix);			// second term is g_pos-g_neg (just stored in g_pos)
		


		// testing multiplication using eigen
		Eigen::MatrixXf EqiMatrix_eig;
		Eigen::MatrixXf gInv_eig;
		Eigen::MatrixXf g_pos_eig;
		Eigen::MatrixXf g_neg_eig;
		Eigen::MatrixXf g_sub;
		gInv_eig = convertGSLMatrixToEigen(set.gInv);
		g_pos_eig = convertGSLMatrixToEigen(g_pos);
		g_neg_eig = convertGSLMatrixToEigen(g_neg);
		g_sub = g_pos_eig - g_neg_eig;
		EqiMatrix_eig = gInv_eig*g_sub;
		//EqiMatrix_eig = EqiMatrix_eig*(1/(2*gsl_vector_get(h,i)));
		EqiMatrix_eig = EqiMatrix_eig*(1/(2*h_alpha));
		EqiMatrix = convertEigenToGSLMatrix(EqiMatrix_eig);
	

		//printf("sub \n");
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,0,0),gsl_matrix_get(g_pos,0,1),gsl_matrix_get(g_pos,0,2),gsl_matrix_get(g_pos,0,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,1,0),gsl_matrix_get(g_pos,1,1),gsl_matrix_get(g_pos,1,2),gsl_matrix_get(g_pos,1,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,2,0),gsl_matrix_get(g_pos,2,1),gsl_matrix_get(g_pos,2,2),gsl_matrix_get(g_pos,2,3));
		//printf("%4.8f \t %4.8f \t %4.8f \t %4.8f \n", gsl_matrix_get(g_pos,3,0),gsl_matrix_get(g_pos,3,1),gsl_matrix_get(g_pos,3,2),gsl_matrix_get(g_pos,3,3));
		//printf("\n");

		/*printf("EqiMatrix \n");
		printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(EqiMatrix,0,0),gsl_matrix_get(EqiMatrix,0,1),gsl_matrix_get(EqiMatrix,0,2),gsl_matrix_get(EqiMatrix,0,3));
		printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(EqiMatrix,1,0),gsl_matrix_get(EqiMatrix,1,1),gsl_matrix_get(EqiMatrix,1,2),gsl_matrix_get(EqiMatrix,1,3));
		printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(EqiMatrix,2,0),gsl_matrix_get(EqiMatrix,2,1),gsl_matrix_get(EqiMatrix,2,2),gsl_matrix_get(EqiMatrix,2,3));
		printf("%f \t %f \t %f \t %f \n", gsl_matrix_get(EqiMatrix,3,0),gsl_matrix_get(EqiMatrix,3,1),gsl_matrix_get(EqiMatrix,3,2),gsl_matrix_get(EqiMatrix,3,3));
*/

		//gsl_matrix_scale(EqiMatrix, (1/(2*gsl_vector_get(h,i))));

        gsl_vector_set(w,0,gsl_matrix_get(EqiMatrix,2,1));
        gsl_vector_set(w,1,gsl_matrix_get(EqiMatrix,0,2));
        gsl_vector_set(w,2,gsl_matrix_get(EqiMatrix,1,0));

        gsl_vector_set(v,0,gsl_matrix_get(EqiMatrix,0,3));
        gsl_vector_set(v,1,gsl_matrix_get(EqiMatrix,1,3));
        gsl_vector_set(v,2,gsl_matrix_get(EqiMatrix,2,3));


		//printf("\n");
		//printf("w \n");
		//printf("%f \t %f \t %f \n",gsl_vector_get(w,0),gsl_vector_get(w,1),gsl_vector_get(w,2));

		//printf("\n");
		//printf("v \n");
		//printf("%f \t %f \t %f \n",gsl_vector_get(v,0),gsl_vector_get(v,1),gsl_vector_get(v,2));
		//printf("\n");
        
        for(int j=0; j<3; j++) {
            gsl_matrix_set(set.Eq,j,i,gsl_vector_get(v,j));
            gsl_matrix_set(set.Eq,j+n,i,gsl_vector_get(w,j));
        }
        gsl_vector* val = gsl_vector_alloc(n);
		//for(int k=0; k<n; k++) {
		//	gsl_vector_set(val,k,gsl_vector_get(b_pos,k));
		//}
		//gsl_vector_sub(val,b_neg);
		//gsl_vector_scale(val,1/(2*gsl_vector_get(h,i)));

		Eigen::VectorXf val_eig;
		Eigen::VectorXf b_pos_eig;
		Eigen::VectorXf b_neg_eig;
		b_pos_eig = convertGSLVectorToEigen(b_pos);
		b_neg_eig = convertGSLVectorToEigen(b_neg);
		val_eig = b_pos_eig - b_neg_eig;
		//val_eig = val_eig*(1/(2*gsl_vector_get(h,i)));
		val_eig = val_eig*(1/(2*h_alpha));
		val = convertEigenToGSLVector(val_eig);
		
		for(int j=0; j<n; j++) {
			gsl_matrix_set(set.Bq,j,i,gsl_vector_get(val,j));
		}
		
		
		// Free
		gsl_vector_free(h);

	}

	// Free all
	gsl_vector_free(v);
	gsl_vector_free(w);
	gsl_vector_free(q_pert_pos);
	gsl_vector_free(q_pert_neg);
	gsl_matrix_free(EqiMatrix);
	gsl_matrix_free(g_pos);
	gsl_matrix_free(g_neg);

	printf("Eq: \n");
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,0,0),gsl_matrix_get(set.Eq,0,1),gsl_matrix_get(set.Eq,0,2),gsl_matrix_get(set.Eq,0,3),gsl_matrix_get(set.Eq,0,4),gsl_matrix_get(set.Eq,0,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,1,0),gsl_matrix_get(set.Eq,1,1),gsl_matrix_get(set.Eq,1,2),gsl_matrix_get(set.Eq,1,3),gsl_matrix_get(set.Eq,1,4),gsl_matrix_get(set.Eq,1,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,2,0),gsl_matrix_get(set.Eq,2,1),gsl_matrix_get(set.Eq,2,2),gsl_matrix_get(set.Eq,2,3),gsl_matrix_get(set.Eq,2,4),gsl_matrix_get(set.Eq,2,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,3,0),gsl_matrix_get(set.Eq,3,1),gsl_matrix_get(set.Eq,3,2),gsl_matrix_get(set.Eq,3,3),gsl_matrix_get(set.Eq,3,4),gsl_matrix_get(set.Eq,3,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,4,0),gsl_matrix_get(set.Eq,4,1),gsl_matrix_get(set.Eq,4,2),gsl_matrix_get(set.Eq,4,3),gsl_matrix_get(set.Eq,4,4),gsl_matrix_get(set.Eq,4,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Eq,5,0),gsl_matrix_get(set.Eq,5,1),gsl_matrix_get(set.Eq,5,2),gsl_matrix_get(set.Eq,5,3),gsl_matrix_get(set.Eq,5,4),gsl_matrix_get(set.Eq,5,5));
	printf("\n");

	printf("Bq \n");
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Bq,0,0),gsl_matrix_get(set.Bq,0,1),gsl_matrix_get(set.Bq,0,2),gsl_matrix_get(set.Bq,0,3),gsl_matrix_get(set.Bq,0,4),gsl_matrix_get(set.Bq,0,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Bq,1,0),gsl_matrix_get(set.Bq,1,1),gsl_matrix_get(set.Bq,1,2),gsl_matrix_get(set.Bq,1,3),gsl_matrix_get(set.Bq,1,4),gsl_matrix_get(set.Bq,1,5));
	printf("%f \t %f \t %f \t %f \t %f \t %f \n", gsl_matrix_get(set.Bq,2,0),gsl_matrix_get(set.Bq,2,1),gsl_matrix_get(set.Bq,2,2),gsl_matrix_get(set.Bq,2,3),gsl_matrix_get(set.Bq,2,4),gsl_matrix_get(set.Bq,2,5));
	
}

// Compute Eu
void computeEu(ConcentricTubeSet &set, gsl_vector* q, gsl_vector* uBar) {
    int n = set.m_tubes.size();
    int uSize = uBar->size;
    float h_scaled = 1e-5;			// for perturbing u
	//int numPts = set.momentStored.size();
	int numPts = set.momentStored.tube1Moment.size();
    
    gsl_vector * u_pert_pos = gsl_vector_alloc(uSize);
    gsl_vector * u_pert_neg = gsl_vector_alloc(uSize);
	gsl_matrix* g_pos = gsl_matrix_alloc(4,4);
    gsl_matrix* g_neg = gsl_matrix_alloc(4,4);
	gsl_matrix* EuiMatrix = gsl_matrix_alloc(4,4);
	gsl_vector* w = gsl_vector_alloc(3);
	gsl_vector* v = gsl_vector_alloc(3);
    
    ConcentricTubeSet posSet;
    ConcentricTubeSet negSet;
    posSet = set;
    negSet = set;

   
    for(int i=0; i<n; i++) {
        // initialize all h to zero
		gsl_vector * h = gsl_vector_calloc(uSize);
        gsl_vector_set(h,i,set.m_tubes[i].kt*h_scaled);

		// initialize u_pert_pos and u_pert_neg to be u
		for(int k=0; k<uSize; k++) {
			gsl_vector_set(u_pert_pos,k,gsl_vector_get(uBar,k));	
			gsl_vector_set(u_pert_neg,k,gsl_vector_get(uBar,k));
		}
        
        // perturb in positive dir
		gsl_vector_add(u_pert_pos,h);			// function stores result in q_pert_pos
        for(int k=0; k<n; k++) {
            posSet.m_tubes[k].moment_guess = gsl_vector_get(u_pert_pos,k);
        }
        solveForwardKinematics(posSet);
        
        // perturb in negative dir
		gsl_vector_sub(u_pert_neg,h);			// function stores result in q_pert_pos
        for(int k=0; k<n; k++) {
            negSet.m_tubes[k].moment_guess = gsl_vector_get(u_pert_neg,k);
        }
        solveForwardKinematics(negSet);
        
        
        // Calculate g (homogeneous transform) resulting from solving kinematics
        g_pos = computeG(posSet);
        g_neg = computeG(negSet);
        // Calculate b (moment) resulting from solving kinematics
        gsl_vector* b_pos = gsl_vector_alloc(n);
        gsl_vector* b_neg = gsl_vector_alloc(n);


		int numPts_pos = posSet.momentStored.tube1Moment.size();
		int numPts_neg = negSet.momentStored.tube1Moment.size();
		for(int j=0; j<n; j++) {
			if(j<1) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube1Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube1Moment[numPts_neg-1]);
			} else if(j<2) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube2Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube2Moment[numPts_neg-1]);
			} else if(j<3) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube3Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube3Moment[numPts_neg-1]);
			} else if(j<4) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube4Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube4Moment[numPts_neg-1]);
			} else if(j<5) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube5Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube5Moment[numPts_neg-1]);
			} else if(j<6) {
				gsl_vector_set(b_pos,j,posSet.momentStored.tube6Moment[numPts_pos-1]);
				gsl_vector_set(b_neg,j,negSet.momentStored.tube6Moment[numPts_neg-1]);
			}
		}
     
        // Form Eu and Bu based on perturbed states
		//gsl_matrix_sub(g_pos,g_neg);			
        //gsl_linalg_matmult(set.gInv,g_pos,EuiMatrix);			// second term is g_pos-g_neg (just stored in g_pos)
		//gsl_matrix_scale(EuiMatrix, 1/(2*gsl_vector_get(h,i)));




		Eigen::MatrixXf EuiMatrix_eig;
		Eigen::MatrixXf gInv_eig;
		Eigen::MatrixXf g_pos_eig;
		Eigen::MatrixXf g_neg_eig;
		Eigen::MatrixXf g_sub;
		gInv_eig = convertGSLMatrixToEigen(set.gInv);
		g_pos_eig = convertGSLMatrixToEigen(g_pos);
		g_neg_eig = convertGSLMatrixToEigen(g_neg);
		g_sub = g_pos_eig - g_neg_eig;
		EuiMatrix_eig = gInv_eig*g_sub;
		EuiMatrix_eig = EuiMatrix_eig*(1/(2*h_scaled));
		EuiMatrix = convertEigenToGSLMatrix(EuiMatrix_eig);




        gsl_vector_set(w,0,gsl_matrix_get(EuiMatrix,2,1));
        gsl_vector_set(w,1,gsl_matrix_get(EuiMatrix,0,2));
        gsl_vector_set(w,2,gsl_matrix_get(EuiMatrix,1,0));
        
        gsl_vector_set(v,0,gsl_matrix_get(EuiMatrix,0,3));
        gsl_vector_set(v,1,gsl_matrix_get(EuiMatrix,1,3));
        gsl_vector_set(v,2,gsl_matrix_get(EuiMatrix,2,3));
        
        for(int j=0; j<3; j++) {
            gsl_matrix_set(set.Eu,j,i,gsl_vector_get(v,j));
            gsl_matrix_set(set.Eu,j+n,i,gsl_vector_get(w,j));
        }
       
		gsl_vector* val = gsl_vector_alloc(n);
		/*for(int k=0; k<n; k++) {
			gsl_vector_set(val,k,gsl_vector_get(b_pos,k));
		}
		gsl_vector_sub(val,b_neg);
		gsl_vector_scale(val,1/(2*gsl_vector_get(h,i)));*/

		Eigen::VectorXf val_eig;
		Eigen::VectorXf b_pos_eig;
		Eigen::VectorXf b_neg_eig;
		b_pos_eig = convertGSLVectorToEigen(b_pos);
		b_neg_eig = convertGSLVectorToEigen(b_neg);
		val_eig = b_pos_eig - b_neg_eig;
		val_eig = val_eig*(1/(2*h_scaled));
		val = convertEigenToGSLVector(val_eig);


		
		for(int j=0; j<n; j++) {
			gsl_matrix_set(set.Bu,j,i,gsl_vector_get(val,j));
		}

		// Free
		gsl_vector_free(h);

    }

	// Free all
	gsl_vector_free(v);
	gsl_vector_free(w);
	gsl_vector_free(u_pert_pos);
	gsl_vector_free(u_pert_neg);
	gsl_matrix_free(EuiMatrix);
	gsl_matrix_free(g_pos);
	gsl_matrix_free(g_neg);
    
	printf("Eu: \n");
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,0,0),gsl_matrix_get(set.Eu,0,1),gsl_matrix_get(set.Eu,0,2));
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,1,0),gsl_matrix_get(set.Eu,1,1),gsl_matrix_get(set.Eu,1,2));
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,2,0),gsl_matrix_get(set.Eu,2,1),gsl_matrix_get(set.Eu,2,2));
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,3,0),gsl_matrix_get(set.Eu,3,1),gsl_matrix_get(set.Eu,3,2));
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,4,0),gsl_matrix_get(set.Eu,4,1),gsl_matrix_get(set.Eu,4,2));
	printf("%f \t %f \t %f \n", gsl_matrix_get(set.Eu,5,0),gsl_matrix_get(set.Eu,5,1),gsl_matrix_get(set.Eu,5,2));

}
//



//
//gsl_matrix* computeGInv(gsl_matrix * gBar) {
//	cMatrix3d rot = cMatrix3d(gsl_matrix_get(gBar,0,0),gsl_matrix_get(gBar,0,1),gsl_matrix_get(gBar,0,2),
//							  gsl_matrix_get(gBar,1,0),gsl_matrix_get(gBar,1,1),gsl_matrix_get(gBar,1,2),
//							  gsl_matrix_get(gBar,2,0),gsl_matrix_get(gBar,2,1),gsl_matrix_get(gBar,2,2));
//	rot = cTranspose(rot);
//	cVector3d v = cVector3d(gsl_matrix_get(gBar,0,3),gsl_matrix_get(gBar,1,3),gsl_matrix_get(gBar,2,3));
//	cVector3d v_new;
//	v_new = -cMul(rot,v);
//
//	gsl_matrix * gInv = gsl_matrix_alloc(4,4);
//	// set rotation part of matrix
//	for(int i=0; i<3; i++) {
//		for(int j=0; j<3; j++) {
//			gsl_matrix_set(gInv,i,j,gsl_matrix_get(rot,i,j));
//		}
//	}
//	// set translation part of matrix
//	gsl_matrix_set(gInv,0,3,gsl_vector_get(v_new,0));
//	gsl_matrix_set(gInv,0,3,gsl_vector_get(v_new,1));
//	gsl_matrix_set(gInv,0,3,gsl_vector_get(v_new,2));
//	// set final row of matrix
//	gsl_matrix_set(gInv,3,0,0);
//	gsl_matrix_set(gInv,3,1,0);
//	gsl_matrix_set(gInv,3,2,0);
//	gsl_matrix_set(gInv,3,3,1);
//
//
//	return gInv;
//
//}
//
//
//// inputs: qBar (joint variables used to solve), uBar (proximal moment guess used to solve), gBar (homogeneous transform)
//
////
//void computeEq(ConcentricTubeSet &set, gsl_vector* q, gsl_vector* u) {
//	int n = set.m_tubes.size();
//	int qSize = q->size();
//	float h_alpha = 10e-5;			// for perturbing alpha [rad]
//	float h_beta = 10e-5;			// for perturbing beta [m]
//	gsl_vector * h = gsl_vector_alloc(qSize);
//	gsl_vector * q_pert_pos = gsl_vector_alloc(qSize);
//	gsl_vector * q_pert_neg = gsl_vector_alloc(qSize);
//
//	ConcentricTubeSet posSet;
//	ConcentricTubeSet negSet;
//	posSet = set;
//	negSet = set;
//
//	
//	for(int i=0; i<(2*n); i++) {
//		// initialize all h to zero
//		for(int j=0; j<qSize; j++) {
//			gsl_vector_set(h,j,0);
//		}
//		// set h for given parameter to h_alpha or h_beta
//		if(i<n) {
//			gsl_vector_set(h,i,h_alpha);
//		} else {
//			gsl_vector_set(h,i,h_beta);
//		}
//		// perturb q in pos direction and solve kinematics for perturbed parameters
//		q_pert_pos = q + h;
//		for(int j=0; j<qSize; j++) {
//			posSet.m_tubes[j].alpha = gsl_vector_get(q_pert_pos,j);
//			posSet.m_tubes[j].Beta = gsl_vector_get(q_pert_pos,qSize+j);
//			posSet.m_tubes[j].moment_guess = gsl_vector_get(u,j);
//		}
//		solveForwardKinematics(&posSet);
//
//		// perturb q in neg direction and solve kinematics for perturbed parameters
//		q_pert_neg = q - h;
//		for(int j=0; j<qSize; j++) {
//			negSet.m_tubes[j].alpha = gsl_vector_get(q_pert_neg,j);
//			negSet.m_tubes[j].Beta = gsl_vector_get(q_pert_neg,qSize+j);
//			negSet.m_tubes[j].moment_guess = gsl_vector_get(u,j);
//		}
//		solveForwardKinematics(&negSet);
//
//
//		//
//	}
//}
//








void printComputedKinematics(ConcentricTubeSet set) {
	/*char *str1 = new char[1024];
	sprintf(str1, "state = [%.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f]", y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11], y[12], y[13], y[14], y[15], y[16], y[17]);
	OutputDebugString(str1);
	delete str1;
*/

	char *str1 = new char[1024];
	sprintf(str1, "state = [%.8f; %.8f; %.8f]", set.angleStored.tube1Angle, set.angleStored.tube2Angle, set.angleStored.tube3Angle);
	OutputDebugString(str1);
	delete str1;

}

Eigen::MatrixXf convertGSLMatrixToEigen(gsl_matrix* gslMat) {
	int numRows = gslMat->size1;
	int numCols = gslMat->size2;
	Eigen::MatrixXf eigMat(numRows,numCols);
	for(int i=0; i<numRows; i++) {			// rows
		for(int j=0; j<numCols; j++) {		// cols
			eigMat(i,j) = gsl_matrix_get (gslMat,i,j);
		}
	}

	return eigMat;
}

gsl_matrix* convertEigenToGSLMatrix(Eigen::MatrixXf eigMat) {
	int numRows = eigMat.rows();
	int numCols = eigMat.cols();
	gsl_matrix* gslMat;
	gslMat = gsl_matrix_alloc(numRows,numCols);
	for(int i=0; i<numRows; i++) {			// rows
		for(int j=0; j<numCols; j++) {		// cols
			gsl_matrix_set(gslMat,i,j,eigMat(i,j));
		}
	}
	return gslMat;
}

Eigen::VectorXf convertGSLVectorToEigen(gsl_vector* gslVec) {
	int numEntries = gslVec->size;
	Eigen::VectorXf eigVec(numEntries);
	for(int i=0; i<numEntries; i++) {
		eigVec(i) = gsl_vector_get(gslVec,i);
	}
	return eigVec;
}

gsl_vector* convertEigenToGSLVector(Eigen::VectorXf eigVec) {
	int numEntries = eigVec.size();
	gsl_vector* gslVec;
	gslVec = gsl_vector_alloc(numEntries);
	for(int i=0; i<numEntries; i++) {
		gsl_vector_set(gslVec,i,eigVec(i));
	}
	return gslVec;
}