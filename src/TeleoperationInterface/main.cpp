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


// Kinematics Variables
ConcentricTubeSet set;
cThread* kinematicsThread;
bool setInvalidated = false;
bool readyToStart = false;


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

	//--------------------------------------------------------------------------
    // HAPTIC DEVICE
    //--------------------------------------------------------------------------
    
	//cHapticDeviceInfo info = hapticDevice->getSpecifications();	// retrieve info about current device
	//
	//camera->addChild(cursor);
	//cursor->setShowFrame(true);
	//cursor->setFrameSize(0.05);

	//// create a tool (cursor) and insert into the world
 //   tool = new cToolCursor(world);
	//camera->addChild(tool);
	//tool->setHapticDevice(hapticDevice);						// connect haptic device to virtual tool
	//tool->setWorkspaceRadius(0.7);								// map physical workspace of haptic device to virtual workspace
	//tool->setRadius(toolRadius);								// define radius for sphere (virtual tool)
	//tool->setShowEnabled(true);
	////tool->setShowFrame(true);									// display reference frame
	////tool->setFrameSize(0.05);									// set the size of the reference frame
	//tool->start();												// start haptic tool

	
	// *** for testing whether haptic device is connected ***
    handler = new cHapticDeviceHandler();			// create a haptic device handler
    handler->getDevice(hapticDevice, 0);			// get a handle to the first haptic device
	int numDevices = handler->getNumDevices();
	printf("num devices: %i", numDevices);



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

		// load tube parameters
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
		// create a thread which starts the kinematics loop
		kinematicsThread = new cThread();
		readyToStart = true;
	}

}

//------------------------------------------------------------------------------
// Close window
//------------------------------------------------------------------------------
void close(void)
{
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
	// simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////
		// read user-switch status 
		//bool button0 = false;
		//bool button1 = false;
		//hapticDevice->getUserSwitch(0, button0);
  //      hapticDevice->getUserSwitch(1, button1);
		// read position
		/*cVector3d position;
        hapticDevice->getPosition(position);

        cMatrix3d rotation;
        hapticDevice->getRotation(rotation);*/

		//cVector3d position;
		//position = tool->getDeviceLocalPos();
		//cMatrix3d rotation;
		//rotation = tool->getDeviceLocalRot();

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		// update position and orientation of cursor
        /*cursor->setLocalPos(position);
        cursor->setLocalRot(rotation);*/

		//cursor->setLocalPos(position);
		//cursor->setLocalRot(rotation);


		/////////////////////////////////////////////////////////////////////
        // COMPUTE FORWARD KINEMATICS TO DETERMINE CURRENT TIP POS
        /////////////////////////////////////////////////////////////////////
		if(readyToStart) {
			//kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
		}

	}
	// exit haptics thread
    simulationFinished = true;
	
}


//------------------------------------------------------------------------------
// Thread to compute kinematics
//------------------------------------------------------------------------------
void runKinematics(void) {
	kinematics(set);	
	setInvalidated = true;
	printf("kinematics computed \n");
}
