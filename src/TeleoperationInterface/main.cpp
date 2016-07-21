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


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------
void resizeWindow(int w, int h);							// callback when the window display is resized
void keySelect(unsigned char key, int x, int y);			// callback when a key is pressed
void updateGraphics(void);									// callback to render graphic scene
void graphicsTimer(int data);								// callback of GLUT timer
void close(void);											// function that closes the application
void updateHaptics(void);									// main haptics simulation loop

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