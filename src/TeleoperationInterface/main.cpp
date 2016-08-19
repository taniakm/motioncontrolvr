//------------------------------------------------------------------------------
#define WIN32

#include "main.h"
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

// Kinematics Variables
ConcentricTubeSet set;
cThread* kinematicsThread;
bool setInvalidated = true;
bool readyToStart = false;

// Multithreading
cMutex *lockObject;
float *alpha;
float *Beta;
double *sVals;
double *xPos;
double *yPos;
double *zPos;
int endIndex;
cVector3d *pos_curr;
cVector3d pos_des = cVector3d(0,0,0);
cVector3d posDot_des;
Eigen::MatrixXf qDot_des(6,1);			// change depending on tube num (TubeNum,1) 
Eigen::VectorXf xDot_des(3);
Eigen::VectorXf q(6);
Eigen::VectorXf qLast(6);
cVector3d *orient_curr;
cVector3d *orient_des;
double pos_error;
cVector3d nHat;
cVector3d delta_des;
cVector3d *devPos;
cVector3d *lastDevPos;

// Teleoperation variables
double epsilonPos = 0.001;  //trying 1 mm for now?
double epsilonOrient = 1;
double vMax = .01;
double vMin = 0.0005;
double lambdaPos = 4;
float deltaT = 2.0;						// assumed amount of time that speed was applied
float offsetBeta[NUM_TUBES];
float offsetAlpha[NUM_TUBES];
// Clamping values for alpha and Beta (to prevent NAN)
float alphaMin = -M_PI;
float alphaMax = M_PI;
float BetaMin = -0.002;
float BetaDeltaMin = -0.002;

// Motion control variables
float deltaPos_des[NUM_TUBES];
float deltaRot_des[NUM_TUBES];
float Beta_des[NUM_TUBES];
float alpha_des[NUM_TUBES];

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
void updatePos(ConcentricTubeSet &set);	
//==============================================================================


int main(int argc, char* argv[])
{
	cout << "-----------------------------------" << endl;
    cout << "Teleoperation Interface Starting" << endl;
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
	tipSphere = new cShapeSphere(0.01);
	world->addChild(tipSphere);
	tipSphere->m_material->setBlueAqua();
	// Create a sphere to be the parent of the backbone spheres
	CTRParentSphere = new cShapeSphere(0.01);
	world->addChild(CTRParentSphere);
	CTRParentSphere->setEnabled(false);
	// Create a line to go from curr tip pos to des tip pos
	desDirLine = new cShapeLine(cVector3d(0,0,0),cVector3d(0,0,0));
	world->addChild(desDirLine);
	desDirLine->m_material->setPinkDeep();
	desDirLine->setLineWidth(10);


	//--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------    
	hapticDevice = std::shared_ptr < cGenericHapticDevice > ((cGenericHapticDevice *)(new cPhantomDeviceWithClutch(0)));
	hapticDevice->open();
	cHapticDeviceInfo info = hapticDevice->getSpecifications();	// retrieve info about current device


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
	if (key == 'f') {
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
    }

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
			ConcentricTubeSet::tube t0;
			t0.alpha = M_PI/2;
			t0.OD = 0.0018;
			t0.ID = 0.0013;
			t0.E = 50000000000;
			t0.v = 0.33;
			t0.kappa = 10;
			t0.Beta = -0.13;
			t0.Lc = 0.08;
			t0.Ls = 0.14;
			set.addTube(t0);

			// Tube 1 parameters
			ConcentricTubeSet::tube t1;
			t1.alpha = -M_PI/2;
			t1.OD = 0.0025;
			t1.ID = 0.0020;
			t1.E = 50000000000;
			t1.v = 0.33;
			t1.kappa = 10;
			t1.Beta = -0.09;
			t1.Lc = 0.07;
			t1.Ls = 0.10;
			set.addTube(t1);

			// Tube 2 parameters
			ConcentricTubeSet::tube t2;
			t2.alpha = 0;
			t2.OD = 0.0035;
			t2.ID = 0.0030;
			t2.E = 50000000000;
			t2.v = 0.33;
			t2.kappa = 9;
			t2.Beta = -0.07;
			t2.Lc = 0.05;
			t2.Ls = 0.08;
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
					t.Beta = -0.004;
				} else if(i==1) {
					t.Beta = -0.01;
				} else if(i==0) {
					t.Beta = -0.022;
				}
				set.addTube(t);
			}
		}

		// initialize device positions
		devPos = new cVector3d(0,0,0);
		lastDevPos = new cVector3d(0,0,0);

		// create a thread which starts the kinematics loop
		kinematicsThread = new cThread();
		kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_GRAPHICS);

		lockObject = new cMutex();
		alpha = new float[tubeNum];
		Beta = new float[tubeNum];
		sVals = new double[1000];
		xPos = new double[1000];
		yPos = new double[1000];
		zPos = new double[1000];
		

		// Set alpha and Beta values
		for(int i=0; i<NUM_TUBES; i++) {
			alpha[i] = set.m_tubes[i].alpha;		// set initial alpha and Beta value
			Beta[i] = set.m_tubes[i].Beta;
			offsetAlpha[i] = set.m_tubes[i].alpha;
			offsetBeta[i] = set.m_tubes[i].Beta;
		}
		// Initialize delta_des
		delta_des.zero();

		// testing starting motion control devices
		robot.Init();
		for(int i=0; i<NUM_TUBES; i++) {
			deltaPos_des[i] = 0;
			deltaRot_des[i] = 0;
		}


		readyToStart = true;
	}

	if(key=='0') {
		deltaPos_des[0] = 10;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='1') {
		deltaRot_des[0] = 60;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='2') {
		deltaPos_des[1] = 10;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='3') {
		deltaRot_des[1] = 60;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='4') {
		deltaPos_des[2] = 10;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='5') {
		deltaRot_des[2] = 60;
		controlRobotPos(robot, deltaPos_des, deltaRot_des);
	}

	if(key=='p') {
		long currPos = robot.m_tubeControllers[0].transController->GetPosition();
		float currPosMM = robot.m_tubeControllers[0].transController->ConvertPositionToMM(currPos, robot.m_tubeControllers[0].transController->devParams);
		printf("currPos trans: %f \t currPosMM: %f \n",(float)currPos, currPosMM);
		currPos = robot.m_tubeControllers[0].rotController->GetPosition();
		currPosMM = robot.m_tubeControllers[0].rotController->ConvertPositionToAngle(currPos, robot.m_tubeControllers[0].rotController->devParams);
		printf("currPos rot: %f \t currPosMM: %f \n",(float)currPos, currPosMM);
	}

	// Try setting desired delta for tip to move
	if(key=='d') {
		//delta_des.set(0.002, 0.001, 0.002);
		pos_des.set(xPos[endIndex]+.001,yPos[endIndex]+.001,zPos[endIndex]+.002);
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
			devPos->set(position(0),position(1),position(2));
		}
		//read rotation
        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		// update position and orientation of cursor
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		/////////////////////////////////////////////////////////////////////
        // UPDATE MOTION CONTROLLERS
        /////////////////////////////////////////////////////////////////////	
		//controlRobotPos(robot, deltaPos_des, deltaRot_des);

		/////////////////////////////////////////////////////////////////////
        // SAVE NEWEST VALUES TO COMPUTE KINEMATICS
        /////////////////////////////////////////////////////////////////////
		if(readyToStart) {
			if(lockObject->tryAcquire()) {
				if(setInvalidated) {
					for(int i=0; i<tubeNum; i++) { 
						alpha[i] = set.m_tubes[i].alpha;
						Beta[i] = set.m_tubes[i].Beta;

						//devPos->set(position(0),position(1),position(2));
					}
				} else {
					updatePos(set);
					/////////////////////////////////////////////////////////////////////
					// UPDATE 
					/////////////////////////////////////////////////////////////////////
					for(int i=0; i<tubeNum; i++) {
						Beta_des[i] = Beta[i]-offsetBeta[i];
						alpha_des[i] = (alpha[i]-offsetAlpha[i])*180.0/M_PI;
					}
					printf("--------------------------------------------------- \n");
					printf("BetaDes: %f %f %f \n",Beta_des[0]*1000,Beta_des[1]*1000,Beta_des[2]*1000);
					printf("AlphaDes: %f %f %f \n",alpha_des[0],alpha_des[1],alpha_des[2]);
					printf("--------------------------------------------------- \n");
					//controlRobotPos(robot, Beta_des, alpha_des);



					drawCTR(set, sVals, xPos, yPos, zPos);
					//lastDevPos->set(devPos->x(),devPos->y(),devPos->z());
				}
				lockObject->release();
			}

			
		}

		

	}
	// exit haptics thread
    simulationFinished = true;
	
}

void updatePos(ConcentricTubeSet &set) {
	endIndex = set.sStored.size()-1;
	for(int i=0;i<=endIndex;i++) {
		sVals[i] = set.sStored[i];
		xPos[i] = set.positionStored.x[i];
		yPos[i] = set.positionStored.y[i];
		zPos[i] = set.positionStored.z[i];
	}

	// Compute difference btwn current and desired tip positions
	cVector3d currTipPos;	
	currTipPos.set(xPos[endIndex],yPos[endIndex],zPos[endIndex]);				// define current position of tip
			
	// ***** try using omni pos as RELATIVE pos ****
	devPos->subr(lastDevPos->x(),lastDevPos->y(),lastDevPos->z(),delta_des);
	currTipPos.addr(delta_des(0),delta_des(1),delta_des(2),pos_des);
	// *********************************************
	// set last dev pos to current dev pos
	lastDevPos->set(devPos->x(),devPos->y(),devPos->z());

	printf("pos_des: %f %f %f \n", pos_des(0),pos_des(1),pos_des(2));
	printf("pos_cur: %f %f %f \n", xPos[endIndex],yPos[endIndex],zPos[endIndex]);
	//printf("dev_pos: %f %f %f \n", devPos->x(), devPos->y(), devPos->z());
	calcError(currTipPos,pos_des);												// calculate diff btwn curr and des pos

	// try drawing line from curr tip pos to des tip pos
	desDirLine->m_pointA.set(currTipPos(0),currTipPos(1),currTipPos(2));
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
	Eigen::MatrixXf JpseudoPos = getJpseudoPos(set.Jpseudo);	// get just position part of Jpseudo
	qDot_des = JpseudoPos*xDot_des;
	// Calculate new actuator values 
	for(int i=0; i<tubeNum; i++) {
		qLast(i) = set.m_tubes[i].alpha;			// ***** or should this be alpha[i] and Beta[i] ???
		qLast(i+tubeNum) = set.m_tubes[i].Beta;
	}
	q = qLast + qDot_des*deltaT;
	// Convert q to alpha and Beta
	for(int i=0; i<tubeNum; i++) {
		// check to make sure alpha value is within range
		if(q(i)>alphaMax) {
			alpha[i] = alphaMax;
		} else if(q(i)<alphaMin) {
			alpha[i] = alphaMin;
		} else {
			alpha[i] = q(i);
		}
		// check to make sure Beta value is within range
		if(q(i+tubeNum)>(BetaMin+(tubeNum-i)*BetaDeltaMin)) {
			Beta[i] = BetaMin+(tubeNum-i)*BetaDeltaMin;
		} else if(q(i+tubeNum)<(-(set.m_tubes[i].Ls + set.m_tubes[i].Lc))) {
			Beta[i] = -(set.m_tubes[i].Ls + set.m_tubes[i].Lc) + 0.001;
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
		if(lockObject->acquire()) {
			// save values
			for(int i=0; i<tubeNum; i++) {
				set.m_tubes[i].alpha = alpha[i];
				set.m_tubes[i].Beta = Beta[i];
			}
			lockObject->release();
		}
		
		setInvalidated = true;
		kinematics(set);
		setInvalidated = false;
		printf("kinematics computed \n");

		// **** not sure where to put this??? ******
		readyToStart = true;
		// *****************************************


		// ************************ TRY PUTTING THIS IN HAPTIC LOOP ***************************
		//if(lockObject->acquire()) {
		//	endIndex = set.sStored.size()-1;
		//	for(int i=0;i<=endIndex;i++) {
		//		sVals[i] = set.sStored[i];
		//		xPos[i] = set.positionStored.x[i];
		//		yPos[i] = set.positionStored.y[i];
		//		zPos[i] = set.positionStored.z[i];
		//	}
		//	//lockObject->release();

		//	// Compute difference btwn current and desired tip positions
		//	cVector3d currTipPos;	
		//	currTipPos.set(xPos[endIndex],yPos[endIndex],zPos[endIndex]);				// define current position of tip
		//	//currTipPos.addr(delta_des(0),delta_des(1),delta_des(2),pos_des);			// define desired position based on delta
		//	
		//	//// **** try using omni pos as des pos *****
		//	//pos_des = cVector3d(devPos->x(), devPos->y(), devPos->z());
		//	//// ****************************************
		//	// ***** try using omni pos as RELATIVE pos ****
		//	devPos->subr(lastDevPos->x(),lastDevPos->y(),lastDevPos->z(),delta_des);
		//	currTipPos.addr(delta_des(0),delta_des(1),delta_des(2),pos_des);
		//	// *********************************************
		//	// set last dev pos to current dev pos
		//	lastDevPos->set(devPos->x(),devPos->y(),devPos->z());

		//	printf("pos_des: %f %f %f \n", pos_des(0),pos_des(1),pos_des(2));
		//	printf("pos_cur: %f %f %f \n", xPos[endIndex],yPos[endIndex],zPos[endIndex]);
		//	//printf("dev_pos: %f %f %f \n", devPos->x(), devPos->y(), devPos->z());
		//	calcError(currTipPos,pos_des);												// calculate diff btwn curr and des pos

		//	// try drawing line from curr tip pos to des tip pos
		//	desDirLine->m_pointA.set(currTipPos(0),currTipPos(1),currTipPos(2));
		//	desDirLine->m_pointB.set(pos_des(0),pos_des(1),pos_des(2));



		//	// Calculate desired linear velocity
		//	double v;
		//	if((pos_error/epsilonPos)>lambdaPos) {
		//		v = vMax;
		//	} else {
		//		v = (vMax-vMin)/(epsilonPos*(lambdaPos-1))*(pos_error-epsilonPos);
		//	}
		//	posDot_des = v*nHat;
		//	// Calculate desired tip velocity
		//	for(int i=0; i<3; i++) {
		//		xDot_des(i) = posDot_des(i);	// Convert cVector3d to Eigen::VectorXf (in this case length 3 bc no orientation control yet)
		//	}
		//	Eigen::MatrixXf JpseudoPos = getJpseudoPos(set.Jpseudo);	// get just position part of Jpseudo
		//	qDot_des = JpseudoPos*xDot_des;
		//	// Calculate new actuator values 
		//	for(int i=0; i<tubeNum; i++) {
		//		qLast(i) = set.m_tubes[i].alpha;			// ***** or should this be alpha[i] and Beta[i] ???
		//		qLast(i+tubeNum) = set.m_tubes[i].Beta;
		//	}
		//	q = qLast + qDot_des*deltaT;
		//	// Convert q to alpha and Beta
		//	for(int i=0; i<tubeNum; i++) {
		//		set.m_tubes[i].alpha = q(i);
		//		set.m_tubes[i].Beta = q(i+tubeNum);
		//		alpha[i] = q(i);
		//		Beta[i] = q(i+tubeNum);
		//	}

		//	// Release lock object
		//	lockObject->release();
		//}
		// *******************************************************************************************
			
	}
}


void drawCTR(ConcentricTubeSet set, double *s, double *x, double *y, double *z) {
	printf("drawing \n");
	cVector3d tipPos = cVector3d(x[endIndex],y[endIndex],z[endIndex]);
	//cVector3d tipPos = 10*cVector3d(x[endIndex],y[endIndex],z[endIndex]);
	tipSphere->setLocalPos(tipPos(0),tipPos(1),tipPos(2));


	// try drawing backbone too
	CTRParentSphere->deleteAllChildren();
	for(int i=0; i<endIndex; i++) {
		backboneSphere[i] = new cShapeSphere(0.005);
		CTRParentSphere->addChild(backboneSphere[i]);
		backboneSphere[i]->m_material->setBlue();
		backboneSphere[i]->setLocalPos(x[i],y[i],z[i]);
	}
	
	

}

void calcError(cVector3d curr, cVector3d des) {
	cVector3d diff;
	des.subr(curr,diff);
	pos_error = sqrt(diff.dot(diff));		// position error
	if(pos_error!=0) {
		nHat = diff/(diff.length());			// nHat
	} else {
		nHat = cVector3d(0,0,0);
	}
}


//------------------------------------------------------------------------------
// Position control for robot
//------------------------------------------------------------------------------
void controlRobotPos(CTRControl robot, float desPos[NUM_TUBES], float desRot[NUM_TUBES]) {
	for(int i=0; i<NUM_TUBES; i++){
		float currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
		float currRotDeg = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
		// Move insertion motor
		if(desPos[i]!=currPosMM) {
			float pos_desMM = desPos[i];
			// Convert desired pos from mm to encoder ticks
			int pos_des = robot.m_tubeControllers[i].transController->ConvertMMToPosition(pos_desMM, robot.m_tubeControllers[i].transController->devParams);
			printf("pos_ticks: %f \n",(float)pos_des);
			// Set position of insertion motor
			robot.m_tubeControllers[i].transController->SetPosition(pos_des);
		}
		// Move rotation motor
		if(desRot[i]!=currRotDeg) {
			float rot_desDeg = desRot[i];
			// Convert desired rot from deg to encoder ticks
			int rot_des = robot.m_tubeControllers[i].rotController->ConvertAngleToPosition(rot_desDeg, robot.m_tubeControllers[i].rotController->devParams);
			printf("rot_ticks: %f \n",(float)rot_des);
			// Set position of rotation motor
			robot.m_tubeControllers[i].rotController->SetPosition(rot_des);
		}
	}
}

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