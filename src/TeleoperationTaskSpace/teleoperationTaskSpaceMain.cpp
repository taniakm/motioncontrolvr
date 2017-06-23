#define WIN32
#include "teleoperationTaskSpaceMain.h"
#include <iostream>
#include <fstream>
#include <conio.h>

using namespace chai3d;
using namespace std;

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------
cStereoMode stereoMode = C_STEREO_DISABLED;
bool fullscreen = false;				// fullscreen mode

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------
cWorld* world;							// a world that contains all objects of the virtual environment
cCamera* camera;						// a camera to render the world in the window display
cDirectionalLight *light;				// a light source to illuminate the objects in the world
cHapticDeviceHandler* handler;			// a haptic device handler
cGenericHapticDevicePtr hapticDevice;	// a pointer to the current haptic device
cVector3d hapticDevicePosition;			// a global variable to store the position [m] of the haptic device 
cShapeSphere* cursor;					// a small sphere (cursor) representing the haptic device
cToolCursor* tool;						// tool
double	toolRadius = 0.01;				// radius of cursor
bool simulationRunning = false;			// flag to indicate if the haptic simulation currently running
bool simulationFinished = false;		// flag to indicate if the haptic simulation has terminated
bool clutchPressed = false;
cPrecisionClock clock;
// for drawing tip positions
cShapeSphere* posCSphere;
cShapeSphere* posDSphere;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

//------------------------------------------------------------------------------
// DATA COLLECTION VARIABLES
//------------------------------------------------------------------------------
cThread*					dataThread;								// thread to run data collection
int							fileNum = 1;	
std::ofstream				teleopDataFile("teleopData.txt");						// set up file for storing data
void collectData(void);

//------------------------------------------------------------------------------
// TELEOPERATION VARIABLES
//------------------------------------------------------------------------------
double						epsilonPos = 0.0005;
double						vMax = 0.012; 
double						vMin = 0.001;
double						lambdaThresh = 15;
float						deltaT = 0.1; 				// assumed amount of time that speed was applied
cVector3d					currTipPos;
cVector3d					posC;

cThread*					kinematicsThread;
ConcentricTubeSet			set;
int							nTubes = 3;	
int							endIndex;
float						BetaMinVec[NUM_TUBES];
cVector3d					posD;
double						delta_pos;
cVector3d					nHat;
cVector3d					posD_dot;
Eigen::VectorXf				xD_dot(3);
Eigen::VectorXf				qD_dot(3);
Eigen::VectorXf				q(6);
Eigen::VectorXf				qPrev(6);
float						alpha[NUM_TUBES];
float						Beta[NUM_TUBES];
bool						reachedPosD = false;

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
void resizeWindow(int w, int h);							// callback when the window display is resized
void keySelect(unsigned char key, int x, int y);			// callback when a key is pressed
void updateGraphics(void);									// callback to render graphic scene
void graphicsTimer(int data);								// callback of GLUT timer
void close(void);											// function that closes the application
void updateHaptics(void);									// main haptics simulation loop
void kinematics(ConcentricTubeSet &set);
void updatePos(void);	
void runKinematics(void);									// to call the kinematics function

//------------------------------------------------------------------------------
// MAIN FUNCTION
//------------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	cout << "-----------------------------------" << endl;
    cout << "Teleoperation Task Space Control Starting" << endl;
	cout << "PRESS THE FOLLOWING KEYS:" << endl;
	cout << "-----------------------------------" << endl;


	//--------------------------------------------------------------------------
    // OPENGL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLUT
    glutInit(&argc, argv);

    // retrieve  resolution of computer display and position window accordingly
    screenW = glutGet(GLUT_SCREEN_WIDTH);
    screenH = glutGet(GLUT_SCREEN_HEIGHT);
    windowW = (int)(0.8 * screenH);
    windowH = (int)(0.5 * screenH);
    windowPosY = (screenH - windowH) / 2;
    windowPosX = windowPosY; 

	// initialize the OpenGL GLUT window
    glutInitWindowPosition(windowPosX, windowPosY);
    glutInitWindowSize(windowW, windowH);
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
    }
    else
    {
        glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);
    }

    // create display context and initialize GLEW library
    glutCreateWindow(argv[0]);
    glewInit();

    // setup GLUT options
    glutDisplayFunc(updateGraphics);
    glutKeyboardFunc(keySelect);
    glutReshapeFunc(resizeWindow);
    glutSetWindowTitle("Teleoperation Task Space");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }

	//--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------
    world = new cWorld();						// create a new world.	
    world->m_backgroundColor.setBlack();		// set the background color of the environment
	// create a camera and insert it into the virtual world
    camera = new cCamera(world);
	camera->setUseOculus(false);
    world->addChild(camera);
    // position and orient the camera
    camera->set( cVector3d (0.5, 0.0, 0.0),     // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),     // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));    // direction of the (up) vector
    camera->setClippingPlanes(0.01, 10.0);		// set the near and far clipping planes of the camera
    camera->setStereoMode(stereoMode);			// set stereo mode
    light = new cDirectionalLight(world);	    // create a directional light source
    world->addChild(light);					    // insert light source inside world
    light->setEnabled(true);				    // enable light source               
    light->setDir(-1.0, 0.0, 0.0);			    // define direction of light beam

	// create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);
    world->addChild(cursor);					// insert cursor inside world
	cursor->setShowFrame(true);
	cursor->setFrameSize(0.05);

	// create sphere that represents posC and posD
	posCSphere = new cShapeSphere(0.003);
	world->addChild(posCSphere);
	posCSphere->m_material->setBlueAqua();
	posDSphere = new cShapeSphere(0.003);
	world->addChild(posDSphere);
	posDSphere->m_material->setRedDark();

	//--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------    
	hapticDevice = std::shared_ptr < cGenericHapticDevice > ((cGenericHapticDevice *)(new cPhantomDeviceWithClutch(0)));
	if(hapticDevice->open()) {
		cout << "Opened haptic device" << endl;
	} else {
		cout << "Could not open haptic device" << endl;
	}
	cHapticDeviceInfo info = hapticDevice->getSpecifications();	// retrieve info about current device

	//--------------------------------------------------------------------------
    // SETUP CTR PARAMETERS
    //-------------------------------------------------------------------------- 
	// Tube 0 parameters
	ConcentricTubeSet::tube t0;
	t0.alpha = 0; //2.58068; 
	t0.OD = 0.0018; //0.0025; //0.0024; 
	t0.ID = 0.0013; //0.0011; //0.001; 
	t0.E = 50000000000; //2400000000;
	t0.v = 0.33;
	t0.kappa = 10; //30.817; //-25; //-50; 
	t0.Beta = -0.12; //-0.245; //-0.15; //-0.22; // min
	t0.Lc = 0.08; //0.0304616; //0.03; 
	t0.Ls = 0.14; //0.25453; //0.21; 
	t0.moment_guess = 0;
	t0.fixedStepSize = false;			// true for design interface, false for teleop
	set.addTube(t0);
	BetaMinVec[0] = -(t0.Lc + t0.Ls);

	// Tube 1 parameters
	ConcentricTubeSet::tube t1;
	t1.alpha = -90*M_PI/180; //-3.09453; //0; 
	t1.OD = 0.0025; //0.0041; //0.0038; 
	t1.ID = 0.002; //0.0033; //0.003; 
	t1.E = 50000000000; //2400000000;
	t1.v = 0.33;
	t1.kappa = 10; //88.8215; //-6.666; // -5.555
	t1.Beta = -0.09; //-0.125; //-0.085; //-0.1; // min
	t1.Lc = 0.07; //0.0150775; //0.07; 
	t1.Ls = 0.1; //0.139452; //0.095; //0.10;
	t1.moment_guess = 0;
	t1.fixedStepSize = false;			// true for design interface, false for teleop
	set.addTube(t1);
	BetaMinVec[1] = -(t1.Lc + t1.Ls);

	// Tube 2 parameters
	ConcentricTubeSet::tube t2;
	t2.alpha = 0;
	t2.OD = 0.0035; //0.0059; //0.0054; 
	t2.ID = 0.003; //0.0049; //0.0044;
	t2.E = 50000000000; //2400000000;
	t2.v = 0.33;
	t2.kappa = 9; //27.1186; //-4; //-5;  
	t2.Beta = -0.07; //-0.05; //-0.03; //-0.03;
	t2.Lc = 0.05; //0.02945; //0.04;
	t2.Ls = 0.08; //0.05; //0.02; 
	t2.moment_guess = 0;
	t2.fixedStepSize = false;			// true for design interface, false for teleop
	set.addTube(t2);
	BetaMinVec[2] = -(t2.Lc + t2.Ls);

	// Run kinematics and set up parameters
	kinematics(set);
	qPrev << t0.alpha, t1.alpha, t2.alpha, t0.Beta, t1.Beta, t2.Beta;
	posD.set(-0.014,-0.028,0.093);	// hard code posD for now

	//--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// create a thread which starts computing the kinematics
    kinematicsThread = new cThread();
	kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);

	// create a thread which starts data collection (don't start until key stroke)
	dataThread = new cThread();
	// precision clock: Start clock running
    clock.reset();
	clock.start();
    
	// start the main graphics rendering loop
	simulationRunning = true;
    glutTimerFunc(50, graphicsTimer, 0);
    glutMainLoop();
   
	// close everything
    close();
    
	// exit
    return (0);
}

//------------------------------------------------------------------------------
// Resize window
//------------------------------------------------------------------------------
void resizeWindow(int w, int h)
{
    windowW = w;
    windowH = h;
}

//------------------------------------------------------------------------------
// Key Selections
//------------------------------------------------------------------------------
void keySelect(unsigned char key, int x, int y)
{
	// UP
	if(key == '8') {
		posD.set(posD(0),posD(1),posD(2)+0.002);
		//printf("new posD: %f %f %f \n",posD(0),posD(1),posD(2));
	}

	//DOWN
	if(key == '2') {
		posD.set(posD(0),posD(1),posD(2)-0.002);
	}

	// LEFT
	if(key == '4') {
		posD.set(posD(0),posD(1)-0.002,posD(2));
	}

	// RIGHT
	if(key == '6') {
		posD.set(posD(0),posD(1)+0.002,posD(2));
	}
}

//------------------------------------------------------------------------------
// Close window
//------------------------------------------------------------------------------
void close(void)
{

	// turn off all motion controllers
	for(int i=0; i<NUM_TUBES; i++) {
		robot.m_tubeControllers[i].transController->Stop();
		robot.m_tubeControllers[i].rotController->Stop();
	}

	// stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();
}

//------------------------------------------------------------------------------
// Graphics Timer
//------------------------------------------------------------------------------
void graphicsTimer(int data)
{
	if (simulationRunning){
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------
// Update Graphics
//------------------------------------------------------------------------------
void updateGraphics(void)
{
	// render world
    camera->renderView(windowW, windowH);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);

}

//------------------------------------------------------------------------------
// Haptic loop
//------------------------------------------------------------------------------
void updateHaptics(void)
{
	// simulation is now running
    simulationRunning  = true;
    simulationFinished = false;


    // main haptic simulation loop
    while(simulationRunning)
    {
		/////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////
		// read position
		cVector3d position;
        hapticDevice->getPosition(position);

		// read rotation
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

		/////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		// update position and orientation of cursor
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		posCSphere->setLocalPos(posC);
		posDSphere->setLocalPos(posD);

		/////////////////////////////////////////////////////////////////////
        // UPDATE POS
        /////////////////////////////////////////////////////////////////////
		if(!reachedPosD) {
			updatePos();
		} else {
			printf("reached target pos \n");
		}

		/////////////////////////////////////////////////////////////////////
		// UPDATE MOTION CONTROLLERS
		/////////////////////////////////////////////////////////////////////
		for(int i=0; i<NUM_TUBES; i++) {
			// for setting RELATIVE position (delta position)
			//Beta_des[i] = (q(i+NUM_TUBES)-offsetBeta[i])*1000;		// convert to MM
			//alpha_des[i] = (q(i)-offsetAlpha[i])*180.0/M_PI;		// convert to degrees

			// update SHARED alpha and beta values (ABSOLUTE values in [m])
			//alpha[i] = q(i);									// try using actual values based on encoder readings
			//Beta[i] = q(i+NUM_TUBES);

			set.m_tubes[i].alpha = q(i);
			set.m_tubes[i].Beta = q(i+NUM_TUBES);
		}

	}
	// exit haptics thread
    simulationFinished = true;
	
}


//------------------------------------------------------------------------------
// Update information on desired position to calculate new alpha & beta values
//------------------------------------------------------------------------------
void updatePos(void) {
	
	// Start by running kinematics (for first attempt)
	/*printf("starting kinematic calc...");
	kinematics(set);
	printf("done \n");*/
	/*endIndex = set.sStored.size()-1;
	posC = cVector3d(set.positionStored.x[endIndex],set.positionStored.y[endIndex],set.positionStored.z[endIndex]);*/

	// Declare needed variables 
	Eigen::MatrixXf JhTemp(6,6);
	JhTemp = set.Jh;
	Eigen::MatrixXf JhPseudoTemp(6,6);
	Eigen::MatrixXf IEig(6,6);
	Eigen::MatrixXf JhpPseudoPosTemp(6,3);
	Eigen::MatrixXf tempMatrix(6,6);

	// Compute JhPseudoPosTemp
	IEig << 1, 0, 0, 0, 0, 0,
		    0, 1, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0,
			0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 1;

	tempMatrix = JhTemp*JhTemp.transpose() + .0000000001*IEig;
	JhPseudoTemp = JhTemp.transpose()*tempMatrix.inverse();
	//JhPseudoTemp = JhTemp.transpose()/(JhTemp*JhTemp.transpose() + .0000000001*IEig);
	JhpPseudoPosTemp << JhPseudoTemp(0,0), JhPseudoTemp(0,1), JhPseudoTemp(0,2),
						JhPseudoTemp(1,0), JhPseudoTemp(1,1), JhPseudoTemp(1,2),
						JhPseudoTemp(2,0), JhPseudoTemp(2,1), JhPseudoTemp(2,2),
						JhPseudoTemp(3,0), JhPseudoTemp(3,1), JhPseudoTemp(3,2),
						JhPseudoTemp(4,0), JhPseudoTemp(4,1), JhPseudoTemp(4,2),
						JhPseudoTemp(5,0), JhPseudoTemp(5,1), JhPseudoTemp(5,2);
	
	// Compute difference btwn current and desired tip positions	
	cVector3d delta_posVec = posD-posC;
	double delta_posVecNorm = delta_posVec.length();
	delta_posVec.divr(delta_posVecNorm,nHat);
	delta_pos = posD.distance(posC);
	//printf("delta_pos: %f \n", delta_pos);
	
	// Calculate desired linear velocity
	double v;
	if(delta_pos<=epsilonPos) {
		v = 0;
		reachedPosD = true;
	} else if((delta_pos/epsilonPos)>lambdaThresh) {
		v = vMax;
		reachedPosD = false;
	} else {
		v = (vMax-vMin)/(epsilonPos*(lambdaThresh-1))*(delta_pos-epsilonPos);
		reachedPosD = false;
	} 
	posD_dot = v*nHat;

	// Calculate desired tip velocity
	for(int i=0; i<3; i++) {
		xD_dot(i) = posD_dot(i);	// Convert cVector3d to Eigen::VectorXf (in this case length 3 bc no orientation control yet)
	}

	qD_dot = JhpPseudoPosTemp*xD_dot;	
	q = qPrev + qD_dot*deltaT;

	// Convert q to alpha and Beta
	for(int i=0; i<tubeNum; i++) {
		// check to make sure alpha value is within range [-pi, pi)
		if(q(i)>M_PI) {
			q(i) = M_PI;
		} else if(q(i)<-M_PI) {
			q(i) = -M_PI;
		} 

		// check to make sure Beta value is within range	
		if(q(i+tubeNum)<BetaMinVec[i]) {
			q(i+tubeNum) = BetaMinVec[i];
		}
	}

}



//------------------------------------------------------------------------------
// Thread to compute kinematics
//------------------------------------------------------------------------------
void runKinematics(void) {
	while(simulationRunning) {
		kinematics(set);
		endIndex = set.sStored.size()-1;
		posC = cVector3d(set.positionStored.x[endIndex],set.positionStored.y[endIndex],set.positionStored.z[endIndex]);
	}
}