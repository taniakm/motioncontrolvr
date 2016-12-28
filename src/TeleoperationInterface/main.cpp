//------------------------------------------------------------------------------
#define WIN32

//#define TESTING_TELEOP

#include "main.h"

// for finding memory leaks
#include <stdlib.h>
#include <crtdbg.h>

//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

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
bool simulationFinished = true;			// flag to indicate if the haptic simulation has terminated
bool clutchPressed = false;
cPrecisionClock clock;
bool jointSpaceControl = true;			// to change btwn joint space control and cartesian control
bool glorifiedJointSpace = false;
bool tissueDrivingTests = false;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// Drawing CTR
cShapeSphere* tipSphere;
cShapeSphere* backboneSphere[200];
cShapeSphere* CTRParentSphere;
cShapeLine* desDirLine;
cShapeBox* workspaceBox;
cMatrix3d transformDrawing;
cShapeSphere* desPosSphere;

// Kinematics Variables
ConcentricTubeSet set;
cThread* kinematicsThread;
bool setInvalidated = true;
bool readyToStart = false;

// Multithreading
cMutex *lockObject;
float alpha[NUM_TUBES];
float Beta[NUM_TUBES];
double *sVals;
double *xPos;
double *yPos;
double *zPos;
cVector3d tipPosShared;
Eigen::MatrixXf JpseudoPosShared(6,3);
Eigen::MatrixXf JhPosShared(3,6);
int *endInd;
int endIndex;
cVector3d *pos_curr;
cVector3d pos_des = cVector3d(0.0, 0.0301, 0.103); //(0.005,0.03,0.14);
cVector3d posDot_des;
Eigen::MatrixXf qDot_des(6,1);			// change depending on tube num (TubeNum,1) 
Eigen::VectorXf xDot_des(3);
Eigen::VectorXf q(6);
Eigen::VectorXf qPrev(6);
cVector3d *orient_curr;
cVector3d *orient_des;
double pos_error;
cVector3d nHat;
cVector3d delta_des;
cVector3d devPos;
cVector3d lastDevPos;

// Teleoperation variables
double epsilonPos = 0.001;  //trying 1 mm for now?
double epsilonOrient = 1;
double vMax = 0.015; //0.08 ;//.022; //.011; 
double vMin = 0.004; //0.001; 
double lambdaPos = 2;//4;
float deltaT = 0.01; // 0.1; // 0.015; //0.008;					// assumed amount of time that speed was applied
float deltaT2 = 0.035;
float offsetBeta[NUM_TUBES];
float offsetAlpha[NUM_TUBES];
cVector3d currTipPos;
cVector3d currTipPosDebug;
// Clamping values for alpha and Beta (to prevent NAN)
float alphaMin = -M_PI;
float alphaMax = M_PI;
float BetaMin;
float BetaMax;
float BetaMinVec[NUM_TUBES];
float BetaMaxVec[NUM_TUBES];
bool BetaMinHit[NUM_TUBES] = {false,false,false};
bool BetaMaxHit[NUM_TUBES] = {false,false,false};

// Motion control variables
float deltaPos_des[NUM_TUBES];
float deltaRot_des[NUM_TUBES];
float Beta_des[NUM_TUBES];
float alpha_des[NUM_TUBES];

// for debugging
bool firstTime = true;
Eigen::MatrixXf JpseudoHardCode(6,6);
Eigen::MatrixXf JHardCode(6,6);
bool hardCode = false;
cShapeSphere* tipSphereDebug;


// for joint space control
cMatrix3d deviceRot;
cVector3d lastAxis;
cVector3d axis;
double lastAngle;
double angle;
bool buttonClicked[NUM_TUBES] = {false,false,false};
bool noButtons = true;
cVector3d rotAxis;
cPhantomDeviceWithClutch *hDeviceJointSpace; 
float rotInit;
float rotInitVec[NUM_TUBES];
cVector3d devicePos;
cVector3d lastPos;
float posInit;
float posInitVec[NUM_TUBES];
bool button[NUM_TUBES] = {false,false,false};
int tubeToControl;
bool rotMode = false;
bool rotModeRecognized = false;

// For tissue driving tests
float posInitVec_tissueDriving[NUM_TUBES];
float posIncrement[NUM_TUBES];

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
void resizeWindow(int w, int h);							// callback when the window display is resized
void keySelect(unsigned char key, int x, int y);			// callback when a key is pressed
void updateGraphics(void);									// callback to render graphic scene
void graphicsTimer(int data);								// callback of GLUT timer
void close(void);											// function that closes the application
void updateHaptics(void);									// main haptics simulation loop
void runKinematics(void);									// to call the kinematics function
void kinematics(ConcentricTubeSet &set);
void drawCTR(ConcentricTubeSet set, double *s, double *x, double *y, double *z);
void calcError(cVector3d curr, cVector3d des);
void controlRobotPos(CTRControl robot, float deltaPos_des[NUM_TUBES], float deltaRot_des[NUM_TUBES]);
Eigen::MatrixXf getJpseudoPos(Eigen::MatrixXf Jpseudo);		// get just the position part of Jpseudo
void updatePos(void);	
void updatePosHardCode(ConcentricTubeSet &set);
Eigen::MatrixXf computeNewJpseudoPos(Eigen::MatrixXf JEig);
//==============================================================================


int main(int argc, char* argv[])
{
	cout << "-----------------------------------" << endl;
    cout << "Teleoperation Interface Starting" << endl;
	cout << "PRESS THE FOLLOWING KEYS:" << endl;
	cout << "s: start motion controllers -> starts as joint space control" << endl;
	cout << "j: switch to task space control (out of joint space)" << endl;
	cout << "g: glorified joint space (follow-the-leader type)" << endl;
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
    glutSetWindowTitle("Teleoperation Interface");

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

	// Create a sphere for the tip pos
	tipSphere = new cShapeSphere(0.002);
	world->addChild(tipSphere);
	tipSphere->m_material->setBlueAqua();
	tipSphere->setShowFrame(true);
	tipSphere->setFrameSize(0.05);
	// Create a sphere to be the parent of the backbone spheres
	CTRParentSphere = new cShapeSphere(0.01);
	world->addChild(CTRParentSphere);
	CTRParentSphere->setEnabled(false);
	// Create a line to go from curr tip pos to des tip pos
	desDirLine = new cShapeLine(cVector3d(0,0,0),cVector3d(0,0,0));
	world->addChild(desDirLine);
	desDirLine->m_material->setPinkDeep();
	desDirLine->setLineWidth(10);
	// Create a box around workspace (approx)
	workspaceBox = new cShapeBox(.05,-.16,-0.04);
	//workspaceBox = new cShapeBox(.04,.05,.16);
	world->addChild(workspaceBox);
	workspaceBox->setTransparencyLevel(0.1);
	workspaceBox->setLocalPos(0,-.08,0);
	//workspaceBox->setLocalPos(0,0,0.08);

	// Create a sphere for the tip pos for debugging
	/*tipSphereDebug = new cShapeSphere(0.002);
	world->addChild(tipSphereDebug);
	tipSphereDebug->m_material->setYellowGold();
	tipSphereDebug->setShowFrame(true);
	tipSphereDebug->setFrameSize(0.05);*/

	// Create a sphere for desired pos of tip
	desPosSphere = new cShapeSphere(0.002);
	world->addChild(desPosSphere);
	desPosSphere->m_material->setYellowGold();

	// set up matrix for transforming drawing to match real life
	//transformDrawing = cMatrix3d(0,0,-1,1,0,0,0,-1,0);
	transformDrawing = cMatrix3d(0,1,0,0,0,-1,-1,0,0);
	/*cVector3d col0 = cVector3d(0,0,-1);
	cVector3d col1 = cVector3d(1,0,0);
	cVector3d col2 = cVector3d(0,-1,0);
	transformDrawing.setCol0(col0);
	transformDrawing.setCol1(col1);
	transformDrawing.setCol2(col2);*/

	//--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------    
	hapticDevice = std::shared_ptr < cGenericHapticDevice > ((cGenericHapticDevice *)(new cPhantomDeviceWithClutch(0)));
	hapticDevice->open();
	cHapticDeviceInfo info = hapticDevice->getSpecifications();	// retrieve info about current device

	//hDeviceJointSpace = (cPhantomDeviceWithClutch *)hapticDevice.get();
	//--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
    // create a thread which starts the main haptics rendering loop
    cThread* hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    
	// start the main graphics rendering loop
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
	// option ESC: exit
    if ((key == 27) || (key == 'x'))
    {
        close();
        exit(0);
    }

	// Toggle fullscreen
	/*if (key == 'f') {
        if (fullscreen)
        {
            windowPosX = glutGet(GLUT_INIT_WINDOW_X);
            windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
            windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
            windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
            glutPositionWindow(windowPosX, windowPosY);
            glutReshapeWindow(windowW, windowH);
            fullscreen = false;
        }
        else
        {
            glutFullScreen();
            fullscreen = true;
        }
    }*/

	// Start teleoperation interface
	if (key == 's') {
		// Check number of tubes 	
		FILE *f = fopen("C:/Users/Tania/Documents/motioncontrolvr/src/TeleoperationGUI/numTubesFile.txt", "rb");
		if(f != NULL) {
			int val;
			fscanf(f,"%i",&val);
			fclose(f);
			tubeNum = val;
			printf("Number of tubes: %i \n", tubeNum);
		} else {
			printf("file null");
		}

		// Check whether to use default tube set	
		FILE *f2 = fopen("C:/Users/Tania/Documents/motioncontrolvr/src/TeleoperationGUI/useDefaultSet.txt", "rb");
		if(f2 != NULL) {
			int val;
			fscanf(f2,"%i",&val);
			fclose(f2);
			useDefaultSetBool = val;
			tubeNum = 3;
			printf("Use Default Set (1=yes, 0=no): %i \n", useDefaultSetBool);
		} else {
			printf("file null");
		}

		if(useDefaultSetBool) {    // load default tube set
			
			// Tube 0 parameters
			//ConcentricTubeSet::tube t0;
			//t0.alpha = 0; //M_PI/2;
			//t0.OD = 0.001514; //0.0018;
			//t0.ID = 0.000902; //0.0013;
			//t0.E = 50000000000;
			//t0.v = 0.33;
			//t0.kappa = -23.2558; //10;
			//t0.Beta = -0.22; //-0.13;
			//t0.Lc = 0.038; //0.08;
			//t0.Ls = 0.268; //0.14;
			//set.addTube(t0);

			//// Tube 1 parameters
			//ConcentricTubeSet::tube t1;
			//t1.alpha = 0; //-M_PI/2;
			//t1.OD = 0.002543; //0.0025;
			//t1.ID = 0.001829; //0.0020;
			//t1.E = 50000000000;
			//t1.v = 0.33;
			//t1.kappa = -5.9524; //10;
			//t1.Beta = -0.1; //-0.09;
			//t1.Lc = 0.042; //0.07;
			//t1.Ls = 0.137; //0.10;
			//set.addTube(t1);

			//// Tube 2 parameters
			//ConcentricTubeSet::tube t2;
			//t2.alpha = 0;
			//t2.OD = 0.003226; //0.0035;
			//t2.ID = 0.002896; //0.0030;
			//t2.E = 50000000000;
			//t2.v = 0.33;
			//t2.kappa = -6.1728; //9;
			//t2.Beta = -0.03; //-0.07;
			//t2.Lc = 0.05;
			//t2.Ls = 0.05; //0.08;
			//set.addTube(t2);
			

			// Tube 0 parameters
			ConcentricTubeSet::tube t0;
			t0.alpha = 0; 
			t0.OD = 0.0024; 
			t0.ID = 0.001; 
			t0.E = 2400000000;
			t0.v = 0.33;
			t0.kappa = -25; //-50; 
			t0.Beta = -0.15; //-0.22; // min
			t0.Lc = 0.03; 
			t0.Ls = 0.21; 
			set.addTube(t0);

			// Tube 1 parameters
			ConcentricTubeSet::tube t1;
			t1.alpha = 0; 
			t1.OD = 0.0038; 
			t1.ID = 0.003; 
			t1.E = 2400000000;
			t1.v = 0.33;
			t1.kappa = -6.666; // -5.555
			t1.Beta = -0.085; //-0.1; // min
			t1.Lc = 0.07; 
			t1.Ls = 0.095; //0.10;
			set.addTube(t1);

			// Tube 2 parameters
			ConcentricTubeSet::tube t2;
			t2.alpha = 0;
			t2.OD = 0.0054; 
			t2.ID = 0.0044;
			t2.E = 2400000000;
			t2.v = 0.33;
			t2.kappa = -4; //-5;  
			t2.Beta = -0.03; //-0.03;
			t2.Lc = 0.04;
			t2.Ls = 0.02; 
			set.addTube(t2);
		} else {			// load the entered tube parameters
			for(int i=0; i<tubeNum; i++) {
				char path[150];
				sprintf(path, "C:/Users/Tania/Documents/motioncontrolvr/src/TeleoperationGUI/tubeParameterFile%d.txt", i);
				ConcentricTubeSet::tube t;

				FILE *f = fopen(path, "rb");
				char buf[1000];
				fscanf(f,"%s",buf);
				printf("%s \n",buf);
				fclose(f);

				char tempVal[10];
				float properties[6];
				int propertyNum = 0;
				int ind = 0;
				bool startFilling = false;
				for(int j=0; j<sizeof(buf); j++) {
					if(buf[j]==':') {
						startFilling = true;
					} else if(buf[j] == ',') {
						tempVal[ind] = '\0';
						properties[propertyNum] = std::stof(tempVal);
						printf("%f \n",properties[propertyNum]);
						ind = 0;
						propertyNum++;
						startFilling = false;
					} else if(buf[j] == ';') {
						startFilling = false;
						tempVal[ind] = '\0';
						properties[propertyNum] = std::stof(tempVal);
						printf("%f \n",properties[propertyNum]);
						printf("done with all \n");
					} else if(startFilling) {
						tempVal[ind] = buf[j];
						ind++;
					}
				}

				// Save values read into tube structure
				t.OD = properties[0];
				t.ID = properties[1];
				t.kappa = properties[2];
				t.Ls = properties[3];
				t.Lc = properties[4];
				t.materialNum = properties[5];	
				if(t.materialNum == 0) {	// Niti
					t.E = 50000000000;
					t.v = 0.33;
				} else if(t.materialNum == 1) {	// PEBA
					t.E = 75;
					t.v = 0.4;
				} else if(t.materialNum == 2) { // Accura
					t.E = 1625;
					t.v = 0.4;
				}
				// Assumtions for alpha and Beta values to start (can change later if we want this as an input to the GUI)
				t.alpha = 0;
				if(i==2) {	// for tube 2
					t.Beta = -0.03; //-0.004;
				} else if(i==1) {
					t.Beta = -0.1; //-0.01;
				} else if(i==0) {
					t.Beta = -0.22; //-0.022;
				}
				set.addTube(t);
			}
		}

		// initialize device positions
		devPos.zero(); 
		lastDevPos.zero();

		//// create a thread which starts the kinematics loop
		//kinematicsThread = new cThread();
		//kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_GRAPHICS);

		lockObject = new cMutex();
		/*alpha = new float[tubeNum];
		Beta = new float[tubeNum];*/
		

		// Commenting out for debugging... these don't seem to be doing anything??
		/*sVals = new double[1000];
		xPos = new double[1000];
		yPos = new double[1000];
		zPos = new double[1000];*/
		

		// Set alpha and Beta values
		for(int i=0; i<NUM_TUBES; i++) {
			alpha[i] = set.m_tubes[i].alpha;		// set initial alpha and Beta value
			Beta[i] = set.m_tubes[i].Beta;
			offsetAlpha[i] = set.m_tubes[i].alpha;
			offsetBeta[i] = set.m_tubes[i].Beta;
			// set initial q values
			qPrev(i) = set.m_tubes[i].alpha;			
			qPrev(i+tubeNum) = set.m_tubes[i].Beta;
			q(i) = qPrev(i);
			q(i+tubeNum) = qPrev(i+tubeNum);
		}

		// Set beta min and max values
		BetaMinVec[2] = -0.03;  //-0.035;
		BetaMinVec[1] = -0.100; //-0.105
		BetaMinVec[0] = -0.220; //-0.225
		BetaMaxVec[2] = -0.005;
		BetaMaxVec[1] = -0.05;
		BetaMaxVec[0] = -0.09;

		// Initialize delta_des
		delta_des.zero();

		// for debugging
		if(hardCode) {
			// set J
			JHardCode << -0.003993, -0.007946, -0.010319, 0.0, 0.0, 0.0,
						0.0, 0.0, 0.0, -0.104859, -0.437415, -0.214496,
						0.0, 0.0, 0.0, 0.957421, -0.150511, -0.153253,
						0.0, 0.0, 0.0, -20.222355, 17.537556, 2.68528,
						-0.30416, -0.316079, -0.136531, 0.0, 0.0, 0.0,
						0.496501, 0.244926, -0.087747, 0.0, 0.0, 0.0;
			// set Jpseudo
			JpseudoHardCode << -0.996418, 0.0, 0.0, 0.0, 0.62817, 1.997,
								1.52232, 0.0, 0.0, 0.0, -2.40524, -0.86005,
								-1.65457, 0.0, 0.0, 0.0, -3.02309, -2.37023,
								0.0, -0.665727, 0.755361, -0.0101856, 0.0, 0.0,
								0.0, -0.167568, 1.33665, 0.0642079, 0.0, 0.0,
								0.0, -3.91928, -3.04137, -0.123655, 0.0, 0.0; 
			// set tip pos
			currTipPosDebug.set(0.0, 0.022257, 0.097838);
			readyToStart = true;
		}

		// testing starting motion control devices
		robot.Init();
		for(int i=0; i<NUM_TUBES; i++) {
			deltaPos_des[i] = 0;
			deltaRot_des[i] = 0;
		}

		//testing taking this out
		//readyToStart = true;

		// create a thread which starts the kinematics loop
		if(!jointSpaceControl) {
			kinematicsThread = new cThread();
			kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_GRAPHICS);
		} else {
			readyToStart = true;
			robot.m_tubeControllers[2].transController->Stop();
			robot.m_tubeControllers[2].rotController->Stop();
		}
		
	}

	if(key=='p') {
		long currPos = robot.m_tubeControllers[0].transController->GetPosition();
		float currPosMM = robot.m_tubeControllers[0].transController->ConvertPositionToMM(currPos, robot.m_tubeControllers[0].transController->devParams);
		printf("currPos trans: %f \t currPosMM: %f \n",(float)currPos, currPosMM);
		currPos = robot.m_tubeControllers[0].rotController->GetPosition();
		currPosMM = robot.m_tubeControllers[0].rotController->ConvertPositionToAngle(currPos, robot.m_tubeControllers[0].rotController->devParams);
		printf("currPos rot: %f \t currPosMM: %f \n",(float)currPos, currPosMM);
	}

	// for setting alpha positions
	if(key=='m') {
		// define all desired rotations
		float rot_desDeg[NUM_TUBES] = {-30,-60,0};
		for(int i=0; i<NUM_TUBES; i++) {
			// Convert desired rot from deg to encoder ticks
			int rot_des = robot.m_tubeControllers[i].rotController->ConvertAngleToPosition(rot_desDeg[i], robot.m_tubeControllers[i].rotController->devParams);
			// Set position of rotation motor
			robot.m_tubeControllers[i].rotController->EnableDevice();
			robot.m_tubeControllers[i].rotController->SetPosition(rot_des);
			//robot.m_tubeControllers[i].rotController->Stop();
		}
	}

	// Try setting desired delta for tip to move
	if(key=='f') {
		//delta_des.set(0.0, 0.0, -0.015); // used to be 'd' down
		delta_des.set(0.0, 0.0, 0.003);
		pos_des.add(delta_des(0),delta_des(1),delta_des(2));
		printf("increased desired pos \n");
	}

	if(key=='b') {
		//delta_des.set(0.0, 0.0, 0.015); // used to be 'u' up on screen
		delta_des.set(0.001, 0.0, 0.0);
		pos_des.add(delta_des(0),delta_des(1),delta_des(2));
	}

	if(key=='o') {
		//delta_des.set(0.0, 0.015, 0.0);  // used to be 'r'
		delta_des.set(0.0, 0.001, 0.0);
		pos_des.add(delta_des(0),delta_des(1),delta_des(2));
	}

	/*if(key=='l') {
		delta_des.set(0.015, 0.0, 0.0);
		pos_des.add(delta_des(0),delta_des(1),delta_des(2));
	}*/

	if(key=='r') {
		delta_des.set(-0.015, 0.0, 0.0);
		pos_des.add(delta_des(0),delta_des(1),delta_des(2));
	}


	// Look for memory leak
	if(key=='l') {
		_CrtDumpMemoryLeaks();
	}

	// Change to joint space control
	if(key=='j') {
		// start joint space control
		if(jointSpaceControl) {
			jointSpaceControl = false;
			printf("******************************** \n");
			printf("Starting Cartesian Space Control \n");
			printf("******************************** \n");
		} else {
			jointSpaceControl = true;
			printf("******************************** \n");
			printf("Starting Joint Space Control \n");
			printf("******************************** \n");
			readyToStart = true;
		}
	}

	// Change to glorified joint space control (for follow the leader)
	if(key=='g') {
		if(glorifiedJointSpace) {
			glorifiedJointSpace = false;
		} else {
			glorifiedJointSpace = true;
			printf("******************************** \n");
			printf("Starting Approximate Follow-the-leader Control \n");
			printf("Press button 0 (bottom) to translate INNERMOST TUBE \n");
			printf("Press button 1 (upper) to translate INNERMOST AND MIDDLE TUBES \n");
			printf("Press both buttons to translate ALL 3 TUBES \n");
			printf("Put Omni pen inside starting hole to activate rotation \n");
			printf("******************************** \n");
		}
	}

	// For testing: tissue driving tests
	if(key=='t') {
		tissueDrivingTests = true;
		//jointSpaceControl = false;
		glorifiedJointSpace = false;
		printf("******************************** \n");
		printf("STARTING TISSUE DRIVING TEST MODE \n");
		printf("******************************** \n");

		// read current position based on encoder readings
		for(int i=0; i<NUM_TUBES; i++) {
			posInitVec_tissueDriving[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
			Beta_des[i] = posInitVec_tissueDriving[i];
			robot.m_tubeControllers[i].transController->Stop();

			alpha_des[i] = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
			robot.m_tubeControllers[i].rotController->Stop();				
		}
		// set everything to 0;
		posIncrement[0] = 0; // set to 10 mm for now
		posIncrement[1] = 0;
		posIncrement[2] = 0;
	}

	// To move tube 0
	if(key=='0') {
		printf("Move Tube 0 \n");
		// read current position based on encoder readings
		for(int i=0; i<NUM_TUBES; i++) {
			posInitVec_tissueDriving[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
		}
		// Set increment to insert tube
		posIncrement[0] = 10; // set to 10 mm for now
		posIncrement[1] = 0;
		posIncrement[2] = 0;

		robot.m_tubeControllers[0].transController->EnableDevice();
		robot.m_tubeControllers[1].transController->Stop();
		robot.m_tubeControllers[2].transController->Stop();
	}

	// To move tube 0 & 1
	if(key=='1') {
		printf("Move Tube 0 & 1 \n");
		// read current position based on encoder readings
		for(int i=0; i<NUM_TUBES; i++) {
			posInitVec_tissueDriving[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
		}
		// Set increment to insert tube
		posIncrement[0] = 10; // set to 10 mm for now
		posIncrement[1] = 10;
		posIncrement[2] = 0;

		robot.m_tubeControllers[0].transController->EnableDevice();
		robot.m_tubeControllers[1].transController->EnableDevice();
		robot.m_tubeControllers[2].transController->Stop();
	}

	// To move tube 0 & 1 & 2
	if(key=='2') {
		printf("Move Tube 0 & 1 & 2 \n");
		// read current position based on encoder readings
		for(int i=0; i<NUM_TUBES; i++) {
			posInitVec_tissueDriving[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
		}
		// Set increment to insert tube
		posIncrement[0] = 10; // set to 10 mm for now
		posIncrement[1] = 10;
		posIncrement[2] = 10;

		robot.m_tubeControllers[0].transController->EnableDevice();
		robot.m_tubeControllers[1].transController->EnableDevice();
		robot.m_tubeControllers[2].transController->EnableDevice();
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
		if(readyToStart) {
			devPos = position;
		}
		// read rotation
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

		// for testing
		//printf("devPos: %f %f %f \n", devPos(0), devPos(1), devPos(2));

		/////////////////////////////////////////////////////////////////////
        // READ BUTTONS
        /////////////////////////////////////////////////////////////////////
		bool button0;
		bool button1;
		hapticDevice->getUserSwitch(0, button0);
		hapticDevice->getUserSwitch(1, button1);
		// determine which tube to control based on button presses
		if(!glorifiedJointSpace) {		// for regular joint space control
			if(button0 && !button1) {
				button[0] = true;
				button[1] = false;
				button[2] = false;
				tubeToControl = 0;
				//printf("controlling tube 0 \n");
			} else if(button1 && !button0) {
				button[0] = false;
				button[1] = true;
				button[2] = false;
				tubeToControl = 1;
				//printf("controlling tube 1 \n");
			} else if(button0 && button1) {
				button[0] = false;
				button[1] = false;
				button[2] = true;
				tubeToControl = 2;
				//printf("controlling tube 2 \n");
			} else {
				button[0] = false;
				button[1] = false;
				button[2] = false;
			}
		} else {	// for glorified joint space control
			if(button0 && !button1) {			// control just tube 0
				noButtons = false;
				button[0] = true;
				button[1] = false;
				button[2] = false;
				//printf("Tube 0 \n");
			} else if(button1 && !button0) {	// control tube 0,1
				noButtons = false;
				button[0] = true;
				button[1] = true;
				button[2] = false;
				//printf("Tube 0 and 1 \n");
			} else if(button0 && button1) {		// control tube 0,1,2
				noButtons = false;
				button[0] = true;
				button[1] = true;
				button[2] = true;
				//printf("Tube 0, 1, and 2 \n");
			} else {
				button[0] = false;
				button[1] = false;
				button[2] = false;
				noButtons = true;
			}
		}

		if(!jointSpaceControl) {	// FOR CARTESIAN CONTROL USE BUTTON AS CLUTCH
			if(!button0) {
				clutchPressed=true;
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->clutchPressed();
				pos_des.set(tipPosShared(0),tipPosShared(1),tipPosShared(2));
			} else if(button0 && clutchPressed) {
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->releaseClutch();
			} 
			/*if(button0) {
				clutchPressed = true;
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->clutchPressed();
			} else if(!button0 && clutchPressed) {
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->releaseClutch();
			}*/
		} else if(jointSpaceControl && !glorifiedJointSpace && !tissueDrivingTests){	// FOR JOINT SPACE CONTROL USE BUTTONS TO SIGNAL WHICH TUBE TO CONTROL
			if(button[tubeToControl] && !buttonClicked[tubeToControl]) {
				buttonClicked[tubeToControl] = true;

				// read device rotation
				hapticDevice->getRotation(deviceRot);
				deviceRot.toAxisAngle(rotAxis, lastAngle);	// get rotation axis (axis that we're rotating about)
				lastAxis = deviceRot.getCol2();				// get axis to determine angle of change
				// read current angle based on encoder readings
				rotInit = robot.m_tubeControllers[tubeToControl].rotController->GetAngle(robot.m_tubeControllers[tubeToControl].rotController->devParams);

				// read current device position
				hapticDevice->getPosition(lastPos);
				// read current position based on encoder readings
				posInit = robot.m_tubeControllers[tubeToControl].transController->GetMM(robot.m_tubeControllers[tubeToControl].transController->devParams);
				printf("tube: %i posInit: %f \n", tubeToControl, posInit);
				// save positions of all motors to use for maintaining position
				for(int i=0; i<NUM_TUBES; i++) {
					Beta_des[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
					alpha_des[i] = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
					// for testing/debugging
					robot.m_tubeControllers[i].transController->Stop();
					robot.m_tubeControllers[i].rotController->Stop();
				}
				// for testing/debugging
				robot.m_tubeControllers[tubeToControl].transController->EnableDevice();
				robot.m_tubeControllers[tubeToControl].rotController->EnableDevice();
			} else if(!button[tubeToControl] && !tissueDrivingTests) {
				buttonClicked[0] = false;
				buttonClicked[1] = false;
				buttonClicked[2] = false;
				if(readyToStart) {
					//Beta_des[tubeToControl] = robot.m_tubeControllers[tubeToControl].transController->GetMM(robot.m_tubeControllers[tubeToControl].transController->devParams);
					//alpha_des[tubeToControl] = robot.m_tubeControllers[tubeToControl].rotController->GetAngle(robot.m_tubeControllers[tubeToControl].rotController->devParams);
					//controlRobotPos(robot, Beta_des, alpha_des);
					robot.m_tubeControllers[tubeToControl].transController->Stop();
					robot.m_tubeControllers[tubeToControl].rotController->Stop();
					
				}
			} 
		} else if(jointSpaceControl && glorifiedJointSpace) {	// FOR GLORIFIED JOINT SPACE CONTROL (FOLLOW THE LEADER)
			// read current device position
			cVector3d tempPos;
			hapticDevice->getPosition(tempPos);
			//printf("pos: %f %f %f \n", tempPos(0),tempPos(1),tempPos(2));
			// check to see if omni is in "home" position
			if((tempPos(0)<0) && (tempPos(0)>-0.11)) {
				if((tempPos(1)<0.0001) && (tempPos(1)>-0.011)) {
					if((tempPos(2)<0) && (tempPos(2)>-0.1)) {
						rotMode = true;
					} else {
						rotMode = false;
					}
				} else {
					rotMode = false;
				} 
			} else {
				rotMode = false;
			}

			// for each motor...
			if(rotMode && !rotModeRecognized) {
				printf("rotation mode \n");
				rotModeRecognized = true;
				// read device rotation
				hapticDevice->getRotation(deviceRot);
				deviceRot.toAxisAngle(rotAxis, lastAngle);	// get rotation axis (axis that we're rotating about)
				lastAxis = deviceRot.getCol1();				// get axis to determine angle of change				
				if(readyToStart) {
					for(int i=0; i<NUM_TUBES; i++) {
						// read current angle based on encoder readings
						rotInitVec[i] = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
						robot.m_tubeControllers[i].rotController->EnableDevice();
					}
				}
			} else if (!rotMode) {
				for(int i=0; i<NUM_TUBES; i++) {
					if(button[i] && !buttonClicked[i]) {
						buttonClicked[i] = true;
						rotModeRecognized = false;
						// read device rotation
						hapticDevice->getRotation(deviceRot);
						deviceRot.toAxisAngle(rotAxis, lastAngle);	// get rotation axis (axis that we're rotating about)
						// to deal with paradoxical translation
						//printf("LastAngle: %f \n", lastAngle);
						//printf("rotAxis: %f %f %f \n",rotAxis(0),rotAxis(1),rotAxis(2));
			
						lastAxis = deviceRot.getCol2();				// get axis to determine angle of change
						// read current device position
						hapticDevice->getPosition(lastPos);
						// read current position based on encoder readings
						posInitVec[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
						// enable motors
						robot.m_tubeControllers[i].transController->EnableDevice();
		
					} else if(!button[i]) {
						buttonClicked[i] = false;
						rotModeRecognized = false;
						if(readyToStart) {
							/*robot.m_tubeControllers[i].transController->Stop();
							robot.m_tubeControllers[i].rotController->Stop();*/


							//******* TESTING ********************
							// read current position based on encoder readings
							posInitVec[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
							// enable motors
							robot.m_tubeControllers[i].transController->EnableDevice();
						}
					}
				}
			}
		}

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		// update position and orientation of cursor
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		/////////////////////////////////////////////////////////////////////
        // SAVE NEWEST VALUES TO COMPUTE KINEMATICS
        /////////////////////////////////////////////////////////////////////
		if(readyToStart) {
			/////////////////////////////////////////////////////////////////////
			// FOR CARTESIAN CONTROL
			/////////////////////////////////////////////////////////////////////
			if(!jointSpaceControl) {
				if(lockObject->tryAcquire()) {
				
					updatePos();
					/////////////////////////////////////////////////////////////////////
					// UPDATE MOTION CONTROLLERS
					/////////////////////////////////////////////////////////////////////
					for(int i=0; i<tubeNum; i++) {
						// for setting RELATIVE position (delta position)
						Beta_des[i] = (q(i+NUM_TUBES)-offsetBeta[i])*1000;		// convert to MM
						alpha_des[i] = (q(i)-offsetAlpha[i])*180.0/M_PI;		// convert to degrees

						// update SHARED alpha and beta values (ABSOLUTE values in [m])
						alpha[i] = qPrev(i);									// try using actual values based on encoder readings
						Beta[i] = qPrev(i+NUM_TUBES);
					}
					/*printf("--------------------------------------------------- \n");
					printf("BetaDes: %f %f %f \n",Beta_des[0],Beta_des[1],Beta_des[2]);
					printf("AlphaDes: %f %f %f \n",alpha_des[0],alpha_des[1],alpha_des[2]);
					printf("--------------------------------------------------- \n");*/
					
					// for debugging
					//printf("--------------------------------------------------- \n");
					//float currPosMM;
					//float currRotDeg;
					//for(int i=0; i<tubeNum; i++) {
					//	currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
					//	currRotDeg = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
					//	//qPrev(i+tubeNum) = currPosMM/1000+offsetBeta[i];
					//}
					//printf("encoder pos: %f %f %f \n", currPosMM/1000+offsetBeta[0],currPosMM/1000+offsetBeta[1],currPosMM/1000+offsetBeta[2]);

					controlRobotPos(robot, Beta_des, alpha_des);

				
					cVector3d transformedTipPos = transformDrawing*currTipPos;
					/*tipSphere->setLocalPos(transformedTipPos(0),transformedTipPos(1),transformedTipPos(2));
					tipSphere->setLocalRot(transformDrawing);*/

					cTransform tipTrans = cTransform(transformedTipPos,transformDrawing);
					tipSphere->setLocalTransform(tipTrans);

					//tipSphere->setLocalPos(currTipPos(0),currTipPos(1),currTipPos(2));
					//drawCTR(set, sVals, xPos, yPos, zPos);

					lockObject->release();		// must release object only if it was acquired 
				}
			} else if(jointSpaceControl && !glorifiedJointSpace && !tissueDrivingTests) { 
				/////////////////////////////////////////////////////////////////////
				// FOR JOINT SPACE CONTROL
				/////////////////////////////////////////////////////////////////////
				hapticDevice->getRotation(deviceRot);
				if(button[tubeToControl]) {
					// calculate angle
					axis = deviceRot.getCol2();
					float theta = acos(axis.dot(lastAxis)/((axis.length())*(lastAxis.length())));
					theta = theta*180.0/M_PI;	

					// determine direction
					cVector3d crossProd;
					axis.crossr(lastAxis,crossProd);					
					if(crossProd.dot(rotAxis)<0) {
						theta = -theta;
					}
					// add increment to original angle
					alpha_des[tubeToControl] = rotInit + theta;
					if(alpha_des[tubeToControl]>(alphaMax*180.0/M_PI)) {
						alpha_des[tubeToControl] = alphaMax*180.0/M_PI;
						printf("max alpha value reached \n");
					} else if(alpha_des[tubeToControl]<(alphaMin*180.0/M_PI)) {
						alpha_des[tubeToControl] = alphaMin*180.0/M_PI;
						printf("min alpha value reached \n");
					}
					//printf("angle: %f \n", alpha_des[0]);

					// position calculations
					hapticDevice->getPosition(devicePos);
					cVector3d diff = devicePos - lastPos;
					double projectedDist = (diff.dot(rotAxis))/rotAxis.length();
					Beta_des[tubeToControl] = posInit - (projectedDist*1000);	// converted to mm
					//printf("dist: %f \n", Beta_des[0]);

					// send command to motor
					controlRobotPos(robot, Beta_des, alpha_des);
				}
			} else if(jointSpaceControl && glorifiedJointSpace) {
				/////////////////////////////////////////////////////////////////////
				// FOR GLORIFIED JOINT SPACE CONTROL (FOLLOW-THE-LEADER APPROX)
				/////////////////////////////////////////////////////////////////////
				// position calculations
				hapticDevice->getPosition(devicePos);
				cVector3d diff = devicePos - lastPos;
				//double projectedDist = (diff.dot(rotAxis))/rotAxis.length();
				double projectedDist = (diff.dot(deviceRot.getCol0()))/rotAxis.length();
				// rotation
				hapticDevice->getRotation(deviceRot);
				// 
				for(int i=0; i<NUM_TUBES; i++) {
					if(!rotMode) {
						if(noButtons) {		// no buttons pressed, so try stopping all motors!
							robot.m_tubeControllers[i].transController->Stop();
							robot.m_tubeControllers[i].rotController->Stop();
						} else if(buttonClicked[i]) {
							Beta_des[i] = posInitVec[i] - ((projectedDist/4)*1000);	// converted to mm
						} else {
							Beta_des[i] = posInitVec[i];
						}
					} else {
						// calculate angle
						axis = deviceRot.getCol1();
						float theta = acos(axis.dot(lastAxis)/((axis.length())*(lastAxis.length())));
						theta = theta*180.0/M_PI;	
						// determine direction
						cVector3d crossProd;
						axis.crossr(lastAxis,crossProd);					
						if(crossProd.dot(rotAxis)<0) {
							theta = -theta;
						}
						//printf("theta: %f \n", theta);

						// add increment to original angle
						alpha_des[i] = rotInitVec[i] + theta;
						if(alpha_des[i]>(alphaMax*180.0/M_PI)) {
							alpha_des[i] = alphaMax*180.0/M_PI;
							printf("max alpha value reached \n");
						} else if(alpha_des[i]<(alphaMin*180.0/M_PI)) {
							alpha_des[i] = alphaMin*180.0/M_PI;
							printf("min alpha value reached \n");
						}
					}
				}
				// send command to motor
				controlRobotPos(robot, Beta_des, alpha_des);
			} else if(tissueDrivingTests) {
				/////////////////////////////////////////////////////////////////////
				// FOR DRIVING THROUGH TISSUE
				/////////////////////////////////////////////////////////////////////
				for(int i=0; i<NUM_TUBES; i++) {
					Beta_des[i] = posInitVec_tissueDriving[i] + posIncrement[i];	// converted to mm
				}
				// send command to motor
				//printf("Beta_des: %f %f %f \n", Beta_des[0], Beta_des[1], Beta_des[2]);
				controlRobotPos(robot, Beta_des, alpha_des);
			}
		}

	}
	// exit haptics thread
    simulationFinished = true;
	
}

//------------------------------------------------------------------------------
// Update information on desired position to calculate new alpha & beta values
//------------------------------------------------------------------------------
void updatePos(void) {

	Eigen::MatrixXf JpseudoPosTemp(6,3);
	Eigen::MatrixXf JhPosTemp(3,6);

	// Grab SHARED variables
	JpseudoPosTemp = JpseudoPosShared;
	JhPosTemp = JhPosShared;
	
	//endIndex = set.sStored.size()-1;
	/*for(int i=0;i<=endIndex;i++) {
		sVals[i] = set.sStored[i];
		xPos[i] = set.positionStored.x[i];
		yPos[i] = set.positionStored.y[i];
		zPos[i] = set.positionStored.z[i];
	}*/
	//int tempInd = *endInd;

	// Compute difference btwn current and desired tip positions	
	//currTipPos.set(xPos[tempInd],yPos[tempInd],zPos[tempInd]);				// define current position of tip
	//currTipPos.set(set.positionStored.x[endIndex],set.positionStored.y[endIndex],set.positionStored.z[endIndex]);
	currTipPos.set(tipPosShared(0),tipPosShared(1),tipPosShared(2));
			
	//// ***** try using omni pos as RELATIVE pos ****
	//devPos->subr(lastDevPos->x(),lastDevPos->y(),lastDevPos->z(),delta_des);
	//currTipPos.addr(delta_des(0),delta_des(1),delta_des(2),pos_des);
	//// *********************************************
	//// set last dev pos to current dev pos
	//lastDevPos->set(devPos->x(),devPos->y(),devPos->z());
	delta_des = devPos - lastDevPos;
	#ifdef TESTING_TELEOP
		delta_des = cVector3d(0,0,0);
	#endif
	pos_des = pos_des + cVector3d(-delta_des(2),delta_des(0),-delta_des(1));
	// for testing!!
	// trying to add delta to current tip pos rather than old desired pos
	//pos_des = currTipPos + 50*cVector3d(-delta_des(2),delta_des(0),-delta_des(1));
	lastDevPos = devPos;


	//printf("pos_des: %f %f %f \n", pos_des(0),pos_des(1),pos_des(2));
	//printf("pos_cur: %f %f %f \n", xPos[endIndex],yPos[endIndex],zPos[endIndex]);

	calcError(currTipPos,pos_des);												// calculate diff btwn curr and des pos

	// try drawing line from curr tip pos to des tip pos
	/*desDirLine->m_pointA.set(currTipPos(0),currTipPos(1),currTipPos(2));
	desDirLine->m_pointB.set(pos_des(0),pos_des(1),pos_des(2));*/
	//desPosSphere->setLocalPos(pos_des(0),pos_des(1),pos_des(2));
	cVector3d transformedTipPos = transformDrawing*currTipPos;
	cVector3d transformedDesPos = transformDrawing*pos_des;
	/*desDirLine->m_pointA.set(transformedTipPos(0),transformedTipPos(1),transformedTipPos(2));
	desDirLine->m_pointB.set(transformedDesPos(0),transformedDesPos(1),transformedDesPos(2));*/
	desPosSphere->setLocalPos(transformedDesPos(0),transformedDesPos(1),transformedDesPos(2));

	/*printf("des pos: %f %f %f \n", transformedDesPos);
	printf("tip pos: %f %f %f \n", transformedTipPos);*/
	//printf("------- \n");

	// Calculate desired linear velocity
	double v;
	if(pos_error<=epsilonPos) {
		v = 0;
	} else if((pos_error/epsilonPos)>lambdaPos) {
		v = vMax;
	} else {
		v = (vMax-vMin)/(epsilonPos*(lambdaPos-1))*(pos_error-epsilonPos);
	} 
	posDot_des = v*nHat;
	// Calculate desired tip velocity
	for(int i=0; i<3; i++) {
		xDot_des(i) = posDot_des(i);	// Convert cVector3d to Eigen::VectorXf (in this case length 3 bc no orientation control yet)
	}

	//qDot_des = set.JpseudoPos*xDot_des;
	qDot_des = JpseudoPosTemp*xDot_des;
	// Calculate new actuator values 
	/*for(int i=0; i<tubeNum; i++) {
		qPrev(i) = q(i); 
		qPrev(i+tubeNum) = q(i+tubeNum); 
	}*/
	q = qPrev + qDot_des*deltaT;

	// Convert q to alpha and Beta
	for(int i=0; i<tubeNum; i++) {
		// check to make sure alpha value is within range [-pi, pi)
		if(q(i)<alphaMin) {
			q(i) = alphaMin;
		} else if(q(i)>alphaMax) {
			q(i) = alphaMax;
		} 

		// check to make sure Beta value is within range	
		BetaMin = BetaMinVec[i];
		BetaMax = BetaMaxVec[i];
		if(q(i+tubeNum)<BetaMin) {
			q(i+tubeNum) = BetaMin;
			//printf("Tube %i cannot move back \n", i);
			BetaMinHit[i] = true;
		} else if(q(i+tubeNum)>BetaMax) {
			q(i+tubeNum) = BetaMax;
			//printf("Tube %i cannot move forward \n", i);
			BetaMaxHit[i] = true;
			//for debugging (and avoiding jumping to max insertion of tube 0)
			//if((i==0)&&((q(i+tubeNum)-qPrev(i+tubeNum))>0.01)){
			//	//Beta[i] = qPrev(i+tubeNum)+0.01;
			//	q(i+tubeNum) = qPrev(i+tubeNum)+0.01;
			//	printf("jumping here \n");
			//}
		} else {
			BetaMinHit[i] = false;
			BetaMaxHit[i] = false;
		}

		// check compared to other tubes
		/*if(i>0) {
			if((q(i+tubeNum)+set.m_tubes[i].Lc+set.m_tubes[i].Ls)>(q(i+tubeNum-1)+set.m_tubes[i-1].Lc+set.m_tubes[i-1].Ls)) {
				q(i+tubeNum) = q(i+tubeNum-1) + set.m_tubes[i-1].Lc + set.m_tubes[i-1].Ls - set.m_tubes[i].Lc -set.m_tubes[i].Ls - 0.002;
			}
		}*/
	}

	// check for Beta min/max being hit
	//if(BetaMinHit[0] && BetaMaxHit[2]) {
	//	//printf("limit hit \n");
	//	Eigen::MatrixXf newJh;
	//	newJh = JhPosTemp;
	//	newJh.block(0,3,3,1) << 0,0,0;
	//	Eigen::MatrixXf newJpseudoPos(6,3);
	//	newJpseudoPos = computeNewJpseudoPos(newJh);

	//	qDot_des = newJpseudoPos*xDot_des;
	//	q = qPrev + qDot_des*deltaT;
	//	for(int i=0; i<tubeNum; i++) {
	//		BetaMin = BetaMinVec[i];
	//		BetaMax = BetaMaxVec[i];
	//		if(q(i+tubeNum)<BetaMin) {
	//			q(i+tubeNum) = BetaMin;
	//		} else if(q(i+tubeNum)>BetaMax) {
	//			q(i+tubeNum) = BetaMax;
	//		}
	//	}
	//} else 
	if((BetaMaxHit[1] && BetaMinHit[2]) || (BetaMaxHit[1] && BetaMaxHit[2]) || BetaMinHit[2]) {
		//printf("limits hit \n");
		// set new JhPos with columns zeroed
		Eigen::MatrixXf newJh;
		newJh = JhPosTemp;
		newJh.block(0,5,3,1) << 0,0,0;
		Eigen::MatrixXf newJpseudoPos(6,3);
		newJpseudoPos = computeNewJpseudoPos(newJh);

		qDot_des = newJpseudoPos*xDot_des;
		q = qPrev + qDot_des*deltaT;
		for(int i=0; i<tubeNum; i++) {
			BetaMin = BetaMinVec[i];
			BetaMax = BetaMaxVec[i];
			if(q(i+tubeNum)<BetaMin) {
				q(i+tubeNum) = BetaMin;
			} else if(q(i+tubeNum)>BetaMax) {
				q(i+tubeNum) = BetaMax;
			}
		}
	}

	//printf("q: %f %f %f \n", q(0),q(1),q(2));
	/*char *str1 = new char[1024];
	sprintf(str1, "q = [%.8f; %.8f; %.8f; %.8f; %.8f; %.8f] \n", q(0),q(1),q(2),q(3),q(4),q(5));
	OutputDebugString(str1);
	delete str1;

	char *str5 = new char[1024];
	sprintf(str5, "pos_des = [%.8f; %.8f; %.8f] \n", pos_des(0), pos_des(1), pos_des(2));
	OutputDebugString(str5);
	delete str5;*/

	char *str1 = new char[1024];
	sprintf(str1, "%.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f; %.8f \n", q(0),q(1),q(2),q(3),q(4),q(5),pos_des(0), pos_des(1), pos_des(2), v, pos_error);
	OutputDebugString(str1);
	delete str1;


	//printf("q: %f %f %f \n", q(0),q(1),q(2));
	/*char *str2 = new char[1024];
	sprintf(str2, "qPrev = [%.8f; %.8f; %.8f] \n", qPrev(0),qPrev(1),qPrev(2));
	OutputDebugString(str2);
	delete str2;*/


	// check for Beta min/max being hit
	//if((BetaMaxHit[1] && BetaMinHit[2]) || (BetaMaxHit[1] && BetaMaxHit[2])) {
	//	printf("need to get unstuck \n");
	//	// set new JhPos with columns zeroed
	//	Eigen::MatrixXf newJh;
	//	//newJh = set.JhPos;
	//	newJh = JhPosTemp;
	//	newJh.block(0,4,3,2) << 0,0,0,0,0,0;
	//	Eigen::MatrixXf newJpseudoPos(6,3);
	//	newJpseudoPos = computeNewJpseudoPos(newJh);

	//	qDot_des = newJpseudoPos*xDot_des;
	//	q = qPrev + qDot_des*deltaT;
	//	for(int i=0; i<tubeNum; i++) {
	//		BetaMin = BetaMinVec[i];
	//		BetaMax = BetaMaxVec[i];
	//		//alpha[i] = q(i);
	//		if(q(i+tubeNum)<BetaMin) {
	//			//Beta[i] = BetaMin;
	//			q(i+tubeNum) = BetaMin;
	//			printf("Tube %i cannot move back \n", i);
	//		} else if(q(i+tubeNum)>BetaMax) {
	//			//Beta[i] = BetaMax;
	//			q(i+tubeNum) = BetaMax;
	//			printf("Tube %i cannot move forward \n", i);
	//			//for debugging (and avoiding jumping to max insertion of tube 0)
	//			if((i==0)&&(abs(q(i+tubeNum)-qPrev(i+tubeNum))>0.08)){
	//				//Beta[i] = qPrev(i+tubeNum)+0.01;
	//				q(i+tubeNum) = qPrev(i+tubeNum)+0.01;
	//				printf("jumping here \n");
	//			}
	//		}else {
	//			//Beta[i] = q(i+tubeNum);
	//		}
	//	}
	//	printf("move 0 forward? \n");
	//} 

	// *********** for debugging ***************************
	//char *str1 = new char[1024];
	//sprintf(str1, "alpha = [%.8f; %.8f; %.8f] \n", alpha[0], alpha[1], alpha[2]);
	//OutputDebugString(str1);
	//delete str1;

	//char *str2 = new char[1024];
	//sprintf(str2, "Beta = [%.8f; %.8f; %.8f] \n", Beta[0], Beta[1], Beta[2]);
	//OutputDebugString(str2);
	//delete str2;

	//char *str5 = new char[1024];
	//sprintf(str5, "pos_des = [%.8f; %.8f; %.8f] \n", pos_des(0), pos_des(1), pos_des(2));
	//OutputDebugString(str5);
	//delete str5;

	/*char *str3 = new char[1024];
	sprintf(str3, "delta_q = [%.8f; %.8f; %.8f; %.8f; %.8f; %.8f] \n", (qDot_des*deltaT)(0)*180/M_PI, (qDot_des*deltaT)(1)*180/M_PI, (qDot_des*deltaT)(2)*180/M_PI, (qDot_des*deltaT)(3)*1000, (qDot_des*deltaT)(4)*1000, (qDot_des*deltaT)(5)*1000);
	OutputDebugString(str3);
	delete str3;
	
	char *str4 = new char[1024];
	sprintf(str4, "Jh = [%.8f %.8f %.8f %.8f %.8f %.8f; %.8f %.8f %.8f %.8f %.8f %.8f; %.8f %.8f %.8f %.8f %.8f %.8f] \n", set.Jh(0,0), set.Jh(0,1), set.Jh(0,2), set.Jh(0,3), set.Jh(0,4), set.Jh(0,5), 
		set.Jh(1,0), set.Jh(1,1), set.Jh(1,2), set.Jh(1,3), set.Jh(1,4), set.Jh(1,5), set.Jh(2,0), set.Jh(2,1), set.Jh(2,2), set.Jh(2,3), set.Jh(2,4), set.Jh(2,5));
	OutputDebugString(str4);
	delete str4;*/



}

void updatePosHardCode(ConcentricTubeSet &set) {
	// ***************************************************************************************
	// calculate delta x based on hard coded jacobian
	Eigen::VectorXf qDelta(6);
	qDelta = q - qPrev;
	Eigen::VectorXf xDelta(6);
	xDelta = set.Jh*qDelta;
	//printf("deltax: %f %f %f \n", xDelta(0), xDelta(1), xDelta(2));
	currTipPosDebug.set(currTipPosDebug(0)+xDelta(0),currTipPosDebug(1)+xDelta(1),currTipPosDebug(2)+xDelta(2));
	calcError(currTipPosDebug,pos_des);
	// ***************************************************************************************

	//// calculate delta x based on hard coded jacobian
	//Eigen::VectorXf qDelta(6);
	//qDelta = q - qPrev;
	//Eigen::VectorXf xDelta(6);
	//xDelta = JHardCode*qDelta;
	//currTipPos.set(currTipPos(0)+xDelta(0),currTipPos(1)+xDelta(1),currTipPos(2)+xDelta(2));


/*	printf("pos_des: %f %f %f \n", pos_des(0),pos_des(1),pos_des(2));
	printf("pos_cur: %f %f %f \n", currTipPos(0),currTipPos(1),currTipPos(2));
	calcError(currTipPos,pos_des);	*/											// calculate diff btwn curr and des pos

	// try drawing line from curr tip pos to des tip pos
	desDirLine->m_pointA.set(currTipPosDebug(0),currTipPosDebug(1),currTipPosDebug(2));
	desDirLine->m_pointB.set(pos_des(0),pos_des(1),pos_des(2));

	// Calculate desired linear velocity
	double v;
	if((pos_error/epsilonPos)>lambdaPos) {
		v = vMax;
	} else {
		v = (vMax-vMin)/(epsilonPos*(lambdaPos-1))*(pos_error-epsilonPos);
	}
	posDot_des = v*nHat;
	// Calculate desired tip velocity
	for(int i=0; i<3; i++) {
		xDot_des(i) = posDot_des(i);	// Convert cVector3d to Eigen::VectorXf (in this case length 3 bc no orientation control yet)
	}
	//Eigen::MatrixXf JpseudoPos = getJpseudoPos(JpseudoHardCode);	// get just position part of Jpseudo
	Eigen::MatrixXf JpseudoPos = getJpseudoPos(set.Jpseudo);
	qDot_des = JpseudoPos*xDot_des;
	// Calculate new actuator values 
	for(int i=0; i<tubeNum; i++) {
		qPrev(i) = set.m_tubes[i].alpha;			// ***** or should this be alpha[i] and Beta[i] ???
		qPrev(i+tubeNum) = set.m_tubes[i].Beta;
	}
	//q = qPrev + qDot_des*deltaT;
	// testing
	Eigen::VectorXf qDotT(6);
	qDotT << qDot_des(0)*deltaT2, qDot_des(1)*deltaT2, qDot_des(2)*deltaT2,
		     qDot_des(3)*deltaT,qDot_des(4)*deltaT,qDot_des(5)*deltaT;
	q = qPrev + qDotT;
	// Convert q to alpha and Beta
	for(int i=0; i<tubeNum; i++) {
		// check to make sure alpha value is within range
		/*if(q(i)>alphaMax) {
			alpha[i] = alphaMax;
		} else if(q(i)<alphaMin) {
			alpha[i] = alphaMin;
		} else {
			alpha[i] = q(i);
		}*/
		alpha[i] = q(i);
		// check to make sure Beta value is within range
		BetaMin = BetaMinVec[i];
		BetaMax = BetaMaxVec[i];
		if(q(i+tubeNum)<BetaMin) {
			Beta[i] = BetaMin;
			printf("Tube %i cannot move back \n", i);
		} else if(q(i+tubeNum)>BetaMax) {
			Beta[i] = BetaMax;
			printf("Tube %i cannot move forward \n", i);
		}else {
			Beta[i] = q(i+tubeNum);
		}
	}
}
//------------------------------------------------------------------------------
// Thread to compute kinematics
//------------------------------------------------------------------------------
void runKinematics(void) {
	while(simulationRunning) {
		//ConcentricTubeSet setTemp;
		if(lockObject->acquire()) {
			// 1. grab values of alpha and beta
			for(int i=0; i<tubeNum; i++) {
				set.m_tubes[i].alpha = alpha[i];
				set.m_tubes[i].Beta = Beta[i];
			}
			lockObject->release();
		} 

		// 2. compute kinematics and jacobian
		//setInvalidated = true;
		kinematics(set);
		//setInvalidated = false;
		printf("kinematics computed \n");

		// for debugging
		/*char *str3 = new char[1024];
		sprintf(str3, "new jacobian \n");
		OutputDebugString(str3);
		delete str3;*/

		// testing
		// 3. Update tip position and jacobian (shared variables)
		if(lockObject->acquire()) {
			endIndex = set.sStored.size()-1;
			tipPosShared.set(set.positionStored.x[endIndex],set.positionStored.y[endIndex],set.positionStored.z[endIndex]);

			// testing to see if we can set initial position of desired pos sphere
			if(firstTime) {
				cVector3d transformedDesPos = transformDrawing*tipPosShared;
				desPosSphere->setLocalPos(transformedDesPos(0),transformedDesPos(1),transformedDesPos(2));

				hapticDevice->getPosition(devPos);
				hapticDevice->getPosition(lastDevPos);

				// testing putting this in here instead b/c think other stuff in here wasn't getting called before
				firstTime = false;
				readyToStart = true;
			}

			JpseudoPosShared = set.JpseudoPos;
			JhPosShared = set.JhPos;

			lockObject->release();
		}


		// testing commenting this out for now and moved it inside loop above
		//if(firstTime) {
		//	firstTime = false;
		//	//testing
		//	readyToStart = true;
		//}

	}
}


void drawCTR(ConcentricTubeSet set, double *s, double *x, double *y, double *z) {
	printf("drawing \n");
	cVector3d tipPos = cVector3d(x[endIndex],y[endIndex],z[endIndex]);
	//cVector3d tipPos = 10*cVector3d(x[endIndex],y[endIndex],z[endIndex]);
	tipSphere->setLocalPos(tipPos(0),tipPos(1),tipPos(2));


	// try drawing backbone too
	/*CTRParentSphere->deleteAllChildren();
	for(int i=0; i<endIndex; i++) {
		backboneSphere[i] = new cShapeSphere(0.002);
		CTRParentSphere->addChild(backboneSphere[i]);
		backboneSphere[i]->m_material->setBlue();
		backboneSphere[i]->setLocalPos(x[i],y[i],z[i]);
	}*/
	
	

}

void calcError(cVector3d curr, cVector3d des) {
	cVector3d diff;
	des.subr(curr,diff);
	pos_error = sqrt(diff.dot(diff));		// position error
	//pos_error = diff.length();
	if(pos_error!=0) {
		// ***** for testing ******
		/*if(abs(diff(1))>abs((1.5*diff(2)))) {
			diff.mul(1,2,1);
		}*/
		// **************************
		nHat = diff/(diff.length());			// nHat
	} else {
		nHat = cVector3d(0,0,0);
	}
}

Eigen::MatrixXf computeNewJpseudoPos(Eigen::MatrixXf JEig) {

	//std::cout << "JEig: \n" << JEig << endl;

	Eigen::MatrixXf JTEig(6,3);
	Eigen::MatrixXf IEig(3,3);
	Eigen::MatrixXf JJTEig(3,3);
	Eigen::MatrixXf JInvTemp(3,3);
	Eigen::MatrixXf JPseud(6,3);
	float p1 = 1e-11;

	JTEig = JEig.transpose();
	JJTEig = JEig*JTEig;

	IEig << 1, 0, 0,
		    0, 1, 0, 
			0, 0, 1;
	IEig = p1*IEig;
	//std::cout << "JJTEig: \n" << JJTEig << endl;
	JInvTemp = JJTEig + IEig;
	JInvTemp = JInvTemp.inverse();
	//std::cout << "JInvTemp: \n" << JInvTemp << endl;
	JPseud = JTEig*JInvTemp;

	return JPseud;
}


//------------------------------------------------------------------------------
// Position control for robot
//------------------------------------------------------------------------------
void controlRobotPos(CTRControl robot, float desPos[NUM_TUBES], float desRot[NUM_TUBES]) {
	for(int i=0; i<NUM_TUBES; i++){
		float currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
		float currRotDeg = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
		// for debugging
		qPrev(i+NUM_TUBES) = currPosMM/1000+offsetBeta[i];	// convert encoder reading to absolute q in m (with offset added back in) 
		qPrev(i) = currRotDeg*M_PI/180.0+offsetAlpha[i];

		// Move insertion motor
		if(desPos[i]!=currPosMM) {
			float pos_desMM = desPos[i];
			// Convert desired pos from mm to encoder ticks
			int pos_des = robot.m_tubeControllers[i].transController->ConvertMMToPosition(pos_desMM, robot.m_tubeControllers[i].transController->devParams);
			// Set position of insertion motor
			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
			//if(!jointSpaceControl) {
			//	if(currPosMM<pos_desMM) {
			//		if(abs(currPosMM-pos_desMM)<15) {
			//			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
			//		} else {
			//			// only move 10 mm more forward
			//			robot.m_tubeControllers[i].transController->SetPosition(robot.m_tubeControllers[i].transController->ConvertMMToPosition(currPosMM+10, robot.m_tubeControllers[i].transController->devParams));
			//			printf("new des pos: %f \n", currPosMM+10);
			//		}
			//	} else {
			//		if(abs(pos_desMM-currPosMM)) {
			//			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
			//		} else {
			//			// only move 10 mm more backward
			//			robot.m_tubeControllers[i].transController->SetPosition(robot.m_tubeControllers[i].transController->ConvertMMToPosition(currPosMM-10, robot.m_tubeControllers[i].transController->devParams));
			//			printf("new des pos: %f \n", currPosMM-10);
			//		}
			//	}
			//} else {   // FOR JOINT SPACE CONTROL
			//	robot.m_tubeControllers[i].transController->SetPosition(pos_des);
			//}
		} 
		// Move rotation motor
		if(desRot[i]!=currRotDeg) {
			float rot_desDeg = desRot[i];
			// Convert desired rot from deg to encoder ticks
			int rot_des = robot.m_tubeControllers[i].rotController->ConvertAngleToPosition(rot_desDeg, robot.m_tubeControllers[i].rotController->devParams);
			//printf("rot_ticks: %f \n",(float)rot_des);
			// Set position of rotation motor
			robot.m_tubeControllers[i].rotController->SetPosition(rot_des);
		}
	}
	//printf("qPrev: %f %f %f \n",qPrev(3),qPrev(4),qPrev(5));
}

// for controlling delta
//void controlRobotPos(CTRControl robot, float deltaPos[NUM_TUBES], float deltaRot[NUM_TUBES]) {
//	for(int i=0; i<NUM_TUBES; i++){
//		float currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
//		float currRotDeg = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
//		float pos_desMM = currPosMM + deltaPos[i];
//		float rot_desDeg = currRotDeg + deltaRot[i];
//		// Move insertion motor
//		if(pos_desMM!=currPosMM) {
//			// Convert desired pos from mm to encoder ticks
//			int pos_des = robot.m_tubeControllers[i].transController->ConvertMMToPosition(pos_desMM, robot.m_tubeControllers[i].transController->devParams);
//			//printf("pos_ticks: %f \n",(float)pos_des);
//			// Set position of insertion motor
//			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
//		}
//		// Move rotation motor
//		if(rot_desDeg!=currRotDeg) {
//			// Convert desired rot from deg to encoder ticks
//			int rot_des = robot.m_tubeControllers[i].rotController->ConvertAngleToPosition(rot_desDeg, robot.m_tubeControllers[i].rotController->devParams);
//			//printf("rot_ticks: %f \n",(float)rot_des);
//			// Set position of rotation motor
//			robot.m_tubeControllers[i].rotController->SetPosition(rot_des);
//		}
//	}
//}

//void controlRobotPos(CTRControl robot, float deltaPos[NUM_TUBES], float deltaRot[NUM_TUBES]) {
//	for(int i=0; i<NUM_TUBES; i++){
//		// Move insertion motor
//		if(deltaPos[i]!=0) {
//			// Get current pos in mm and add it to desired delta
//			float pos_desMM = deltaPos[i] + robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
//			long currPos = robot.m_tubeControllers[i].transController->GetPosition();
//			printf("currPos: %f \n",(float)currPos);
//			// Convert desired pos from mm to encoder ticks
//			int pos_des = robot.m_tubeControllers[i].transController->ConvertMMToPosition(pos_desMM, robot.m_tubeControllers[i].transController->devParams);
//			// Set position of insertion motor
//			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
//			// Set desired delta back to 0
//			deltaPos_des[i] = 0;
//		}
//		// Move rotation motor
//		if(deltaRot[i]!=0) {
//			// Get current rot in deg and add it to desired delta
//			float rot_desDeg = deltaRot[i] + robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
//			long currRot = robot.m_tubeControllers[i].rotController->GetPosition();
//			printf("currRot: %f \n",(float)currRot);
//			// Convert desired rot from deg to encoder ticks
//			int rot_des = robot.m_tubeControllers[i].rotController->ConvertAngleToPosition(rot_desDeg, robot.m_tubeControllers[i].rotController->devParams);
//			// Set position of rotation motor
//			robot.m_tubeControllers[i].rotController->SetPosition(rot_des);
//			// Set desired delta back to 0
//			deltaRot_des[i] = 0;
//		}
//	}
//}

//------------------------------------------------------------------------------
// Get just position part of jacobian pseudo inverse
//------------------------------------------------------------------------------
Eigen::MatrixXf getJpseudoPos(Eigen::MatrixXf Jpseudo) {
	int numRows = Jpseudo.rows();
	Eigen::MatrixXf JpseudoPos(numRows,3);
	for(int i=0; i<numRows; i++) {
		for(int j=0; j<3; j++) {
			JpseudoPos(i,j) = Jpseudo(i,j);
		}
	}

	return JpseudoPos;
}