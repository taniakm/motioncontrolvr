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
cStereoMode stereoMode = C_STEREO_PASSIVE_LEFT_RIGHT;
bool fullscreen = false;				// fullscreen mode
bool mirroredDisplay = false;			// mirrored display

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


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
void resizeWindow(int w, int h);							// callback when the window display is resized
void keySelect(unsigned char key, int x, int y);			// callback when a key is pressed
void updateGraphics(void);									// callback to render graphic scene
void graphicsTimer(int data);								// callback of GLUT timer
void close(void);											// function that closes the application
void updateHaptics(void);									// main haptics simulation loop
//void loadTube(const char *path, ConcentricTubeSet::tube *t);
//void loadTubeParameters(ConcentricTubeSet::tube *t);
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
    world->addChild(camera);
    // position and orient the camera
    camera->set( cVector3d (0.5, 0.0, 0.0),     // camera position (eye)
                 cVector3d (0.0, 0.0, 0.0),     // look at position (target)
                 cVector3d (0.0, 0.0, 1.0));    // direction of the (up) vector
    camera->setClippingPlanes(0.01, 10.0);		// set the near and far clipping planes of the camera
    camera->setStereoMode(stereoMode);			// set stereo mode
    camera->setStereoEyeSeparation(0.01);		// set stereo eye separation (applies only if stereo is enabled)
    camera->setStereoFocalLength(0.5);			// set stereo focal length (applies only if stereo is enabled)
    camera->setMirrorVertical(mirroredDisplay); // set vertical mirrored display mode
    light = new cDirectionalLight(world);	    // create a directional light source
    world->addChild(light);					    // insert light source inside world
    light->setEnabled(true);				    // enable light source               
    light->setDir(-1.0, 0.0, 0.0);			    // define direction of light beam

	// create a sphere (cursor) to represent the haptic device
    cursor = new cShapeSphere(0.01);
    world->addChild(cursor);					// insert cursor inside world



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
			set.addTube(t);
		}

	}

}

void close(void)
{

}

void graphicsTimer(int data)
{
	if (simulationRunning){
        glutPostRedisplay();
    }

    glutTimerFunc(50, graphicsTimer, 0);
}

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

void updateHaptics(void)
{


	
}






	

	//// Call function to set up all tube parameters
	//setTubeParams(set);

	//for(int i=0; i<nTubes; i++) {
	//	ConcentricTubeSet::tube t;
	//	
	//	t.alpha = alpha_init[i];
	//	t.OD = OD_init[i];
	//	t.ID = ID_init[i];
	//	t.E = E_init[i];
	//	t.v = v_init[i];
	//	t.kappa = kappa_init[i];
	//	t.Beta = Beta_init[i];
	//	t.Lc = Lc_init[i];
	//	t.Ls = Ls_init[i];
	//	t.materialNum = 0;		// set initial material to Nitinol

	//	t.moment_guess = 0;

	//	set.addTube(t);
	//}
	//// Call function to set up material display properties
	//for(int i=0; i<nTubes; i++) {
	//	updateTubeMaterial(set, i, 0);
	//}


	//char path[150];
	//int i = 0;
	//for(i=0; i<set.n_tubes; i++) {
	//	sprintf(path, "C:/Users/Tania/Documents/motioncontrolvr/src/TeleoperationGUI/tubeParameterFile%d.txt", i);
	//	ConcentricTubeSet::tube t;
	//	loadTube(path, &t);
	//	set.m_tubes[i] = t;
	//}
