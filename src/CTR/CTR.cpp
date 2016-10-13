//==============================================================================
/* Based on example 1 */
//==============================================================================

//------------------------------------------------------------------------------
#define OVR_OS_WIN32


#include "Service_NetClient.h"
#include "Service_NetSessionCommon.h"

#include "OVR_CAPI.h"		// Include the OculusVR SDK
#include "OVR_Stereo.h"
#include "OVR_Profile.h"
#include "Util_LatencyTest2Reader.h"
#include "Util_LatencyTest2State.h"
#include "Util_Render_Stereo.h"
#include "OVR_Math.h"
#include "CPhantomDeviceWithClutch.h"
#include "ConcentricTubeSet.h"

#include "Leap.h"
#include "LeapListener.h"
#include "CTR.h"
#include "SerialClass.h"
#include <math.h> 
#include <iostream>
#include <fstream>
#include <conio.h>




//------------------------------------------------------------------------------
using namespace chai3d;
using namespace OVR;
using namespace OVR::Util::Render;
using namespace Leap;
using namespace std;
//------------------------------------------------------------------------------

#include "GL/glut.h"

//#define TESTBED
#define DEBOUNCE_TIME 0.20

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_PASSIVE_LEFT_RIGHT;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------
cWorld* world;							// a world that contains all objects of the virtual environment
cCamera* camera;						// a camera to render the world in the window display
cDirectionalLight *light;				// a light source to illuminate the objects in the world
cHapticDeviceHandler* handler;			// a haptic device handler
cGenericHapticDevicePtr hapticDevice;	// a pointer to the current haptic device
cLabel* labelHapticDeviceModel;			// a label to display the haptic device model
cLabel* labelHapticDevicePosition;		// a label to display the position [m] of the haptic device
cVector3d hapticDevicePosition;			// a global variable to store the position [m] of the haptic device
cLabel* labelHapticRate;				// a label to display the rate [Hz] at which the simulation is running
cShapeSphere* cursor;					// a small sphere (cursor) representing the haptic device 
cShapeLine* velocity;					// a line representing the velocity vector of the haptic device
cThread* hapticsThread;					// thread to run haptics rendering loop
bool useDamping = false;				// flag for using damping (ON/OFF)
bool useForceField = true;				// flag for using force field (ON/OFF)
bool simulationRunning = false;			// flag to indicate if the haptic simulation currently running
bool simulationFinished = true;			// flag to indicate if the haptic simulation has terminated
cFrequencyCounter frequencyCounter;		// frequency counter to measure the simulation haptic rate
cVector3d cursorOffset = cVector3d(0,-0.05,-0.15);
cToolCursor* tool;
cVector3d clutchOffset = cVector3d(0,0,0);
cVector3d clutchOffsetLocal = cVector3d(0,0,0);
double	toolRadius = 0.01;
double maxStiffness;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

int labelX = 10;
int labelY = windowH;
int labelZ = 0;

//------------------------------------------------------------------------------
// GRAPHICS VARIABLES 
//------------------------------------------------------------------------------
cMultiMesh*					a_mesh;										// Mesh for imported obj file	
cMatrix3d					meshRotMatrix;								// Rotation matrix of imported mesh
cShapeSphere*				posSphere;	
cShapeSphere*				testSphere;
cShapeSphere*				originSphere;	
cShapeSphere*				rotateSphere;
cMultiMesh*					kidney_mesh;
cMultiMesh*					liver_mesh;
cMultiMesh*					tube_mesh;
cMultiMesh*					right_kidney_mesh;
cShapeSphere*				tubeStartSphere;
cShapeSphere*				originSphere2;
cMultiMesh*					stone_mesh;
cMultiMesh*					skin_mesh;
cMultiMesh*					renal_pelvis_mesh;
float						insertPointX = 0;
float						insertPointY = 0;
float						insertPointZ = -0.2;

cVector3d					meshCenter;									// For translating meshes to be centered at origin
Tube						tube;										// Struct containing spheres to draw tube
int							n;											// Number of points to draw for tube
cLabel*						commandLabel;								// Label for displaying command type
string						commandName = "";							// For use in the label
cLabel*						materialLabel;								// Label for displaying material type
string						materialName = "";							// For use in material label
string						materialNameVec[3];

cLevel*						materialOptionDisplay[6];
cLevel*						materialOptionDisplay1;
cLevel*						materialOptionDisplay2;
cLevel*						materialOptionDisplay3;
cLevel*						materialOptionDisplay4;
cLevel*						materialOptionDisplay5;
cLevel*						materialOptionDisplay6;
cLabel*						materialOptionLabel[3];
cLabel*						materialOptionLabel1;
cLabel*						materialOptionLabel2;
cLabel*						materialOptionLabel3;
// Variables for haptic interactions
bool						gripped = false;
bool						distalGripped = false;
cVector3d					contactVecInit;
cVector3d					tubeContactVecInit;
cVector3d					lastContactVec;
cVector3d					lastTubeContactVec;
cVector3d					lastContactPoint;
cVector3d					lastTubeContactPoint;
cVector3d					normVec = cVector3d(1,0,0);
cVector3d					tubeNormVec;
cMatrix3d					clutchedRot;
cMatrix3d					rotModel;
cMatrix3d					newRot;
double						theta = 0.0;
float						tubeTheta;
cShapeLine*					tanLine; 
cShapeLine*					globalTanLine;
bool						hapticsOn = true;
bool						kappaChange = false;
cVector3d					unitVecOrig;
float						dotProd;
float						kappaLast;
int							lastTube;
bool						lengthChange = false;
int							sphereInd = 0;
int							lastSphereInd = 0;
bool						lineGripped[6];
bool						lineGrippedForLengthChange[6];
float						testIncrement = 0;
float						deltaLength = 0;
double						angle;
double						lastAngle;
bool						grippedToRot;
bool						readyToChangeMaterial = false;
bool						changeMaterial = false;
int							grippedToRotInd;
int							readyToChangeMaterialInd;
long double					angleChange;
cMatrix3d					deviceRot;
cLabel*						alphaLabel;
string						alphaValue = "";
cVector3d					pA;
cVector3d					pB;
// Collision detection variables
cToolCursor*				CTRTool[100];
int							numCTRSpheres;
cCollisionAABB*				CTRCollisionDetect[1000];
cToolCursor*				testTool;
cCollisionRecorder			CTRCollisionRecord;
cCollisionSettings			CTRCollisionSetting;

//------------------------------------------------------------------------------
// KINEMATICS VARIABLES 
//------------------------------------------------------------------------------
bool setInvalidated = false;
ConcentricTubeSet set;
void kinematics(ConcentricTubeSet &set);
void setTubeParams(ConcentricTubeSet set);
cThread* kinematicsThread;
// TUBE PARAMETERS
float						hsize = 0.0005;
int							nTubes = 3;							// Number of tubes in CTR *****CHANGE THIS AS NEEDED*******
int							nStates = 2*nTubes + 12;
float						*alpha_init;
float						*OD_init;
float						*ID_init;
float						*E_init;
float						*v_init;
float						*kappa_init;
float						*Beta_init;
float						*Lc_init;
float						*Ls_init;
cMaterial					tubeColor[6];						// Color of each concentric tube (max of 6 right now)
cColorf						lineColor[6];
float						EVec[3];							// Vector of E values for different possible materials
float						vVec[3];							// Vector of v values for different possible materials
float						maxStrainVec[3];					// Vector of max strain values for different materials
float						maxLength[3];
float						minWallThickness[3];
float						maxStrain;
cVector3d					a1;
cVector3d					a2;
cVector3d					a3;
cVector3d					a4;
cShapeSphere*				proximalSphere;
cShapeSphere*				distalSphere;
cShapeSphere*				lengthSphere[6];
cShapeLine*					centerLine[6];	
cShapeLine*					debugLine;
int							tubeStartInd[6];
int							tubeMidInd[6];
int							tubeEndInd[6];
int							tubeStartIndReduced[6];
int							tubeEndIndReduced[6];
int							tubeMidIndReduced[6];
int							lengthSphereInd[6];
cLabel*						warningLabel;						// For warning if parameters are out of range based on material
string						warningName = "";
cLabel*						waitLabel;
string						waitName = "Computing configurations. Please wait.";
// SIMULATION PARAMETERS
ConcentricTubeSet*			simulatedSets = NULL;
int							numValidSets = 0;		
std::vector<std::vector<float>> simulatedBeta;
std::vector<std::vector<float>> simulatedAlpha;
bool						calculated = false;
bool						readyToDraw = false;
bool						pauseSimPressed = false;
int							currSetDrawing = 1000;
int							currIter = 0;
// INITIALIZATION PARAMETERS
cVector3d					cSolved;							// solved value of center of circle
double						rSolved;							// solved value of radius of circle
cVector3d					*points;							// set up variable to hold a1 through an
double						*rValues;							// store computed r values
cVector3d					*cValues;							// store computed c values
cVector3d					*normals;							// store computed normals to each curve
cVector3d					v2;
cVector3d					v2_init;
cVector3d					v2_temp;
cVector3d					globalTanVec;
cShapeSphere*				pointSphere[10];					// spheres for showing points identified by user for defining initial curves (set to some random size here)
bool						CTRInitialized = false;				// starts as false, and only changed to true once initial points are identified and initial CTR computed
int							pointNum = 0;
bool						readyToDrawInitCTR = false;
double						initAngle;
double						*savedAlphas;						// for testing
cPrecisionClock				otherClock;							// for timing other tasks that need to happen
cPrecisionClock				debounceClock;						// for timing other tasks that need to happen

Eigen::VectorXf circVec(4);

// Initial tube parameters in [m]
float						length_straight = 0.10;	
float						length_curved = 0.040;
float						curvature_radius = 0.020;
float						diameter = 0.003;


//------------------------------------------------------------------------------
// OCULUS VARIABLES 
//------------------------------------------------------------------------------
ovrHmd						pHMD;										// The handle of the headset
HMDInfo						pHMDInfo;									// Information about HMD
HmdRenderInfo				pHMDRenderInfo;								// Information needed for rendering
Sizei						textureSizeRec;								// Recommended texture size
bool						rendertargetSharedByBothEyes;				// true:size of the combined buffer, false:size of each individual buffer
StereoConfig				stereo;
Ptr<Profile>				pDefaultProfile;

// Left eye rendering parameters
StereoEyeParamsWithOrtho	leftEyeParam;								// parameters of left eye
Recti						leftVP;										// viewport
Matrix4f					leftProjection;								// Projection matrix used with this eye.
Matrix4f					leftTranslation;							// Translation to be applied to view matrix.
DistortionRenderDesc		leftDistortion;								// Distortion on the physical display 
Recti						leftDistortionVP;							// Viewport on the physical display
FovPort						leftFov;									// The FOVs of this scene.
// Right eye rendering parameters
StereoEyeParamsWithOrtho	rightEyeParam;								// parameters of right eye
Recti						rightVP;									// viewport
Matrix4f					rightProjection;							// Projection matrix used with this eye.
Matrix4f					rightTranslation;							// Translation to be applied to view matrix.
DistortionRenderDesc		rightDistortion;							// Distortion on the physical display 
Recti						rightDistortionVP;							// Viewport on the physical display
FovPort						rightFov;									// The FOVs of this scene.
// For sending oculus info to camera
float						projectionLeftVectorized[16];
float						translationLeftVectorized[16];
float						projectionRightVectorized[16];
float						translationRightVectorized[16];
int							projLeft = 0;
int							transLeft = 1;
int							projRight = 2;
int							transRight = 3;
int							leftViewportParams[4];
int							rightViewportParams[4];
bool						usingOculus = false;
cMatrix3d					baseRotation;
cMatrix3d					startingRotation;
cMatrix3d					zoomedRotation;
cMatrix3d					headRotation;
// For head tracking
Tracking::TrackingState		ts;											// tracking state
PoseStatef					HeadPose;
Posef						CameraPose;
float						eyePitch;
float						eyeRoll;
float						eyeYaw;
cVector3d					eyePos;
cTransform					headTransform;
float						oculusOffset = 0.4;   //0.6
float						defaultOculusOffset = 0.4;
cMatrix3d					defaultRotation;
cMatrix3d					lastRot;
int							rotNum = 0;


float						userEyeSeparation = 0.005;
float						userEyeFocalLength = 0.5;

#ifdef USE_LEAP 
//------------------------------------------------------------------------------
// LEAP VARIABLES
//------------------------------------------------------------------------------
Controller					controller;
cShapeSphere*				leftCenterSphere;						// a small sphere representing the center of the hand
cShapeSphere*				rightCenterSphere;
Frame						frame;									// Current frame
HandList					pHandList;								// List of all hands in view
Hand						pHand;									// Hand in view
int							handID;									// Hand id
int							handCount;								// Number of hands in view
Vector						handCenterTemp;							// Temporary variable for center pos of hand
cVector3d					handCenter;								// Hand pos
FingerList					pFingerList;							// List of all fingers in view
Finger						pFinger;								// Finger
int							fingerCount;							// Number of fingers in view
cMatrix3d					leapToOculusRot;						// Rotation from leap to oculus coord frame
InteractionBox				interactionBox;							// Interaction box needed to normalize coords
Vector						handCenterNorm;							// Center of hand normalized
Bone						pBone;									// Bone
Bone::Type					pBoneType;								// Type of bone
Vector						boneStart;								// Coord of start of bone
Vector						boneEnd;								// Coord of end of bone
float						boneWidth;								// Width of bone
LeftSkeletonHand			leftSkeletonHand;						// Struct with data on left skeleton hand to track/draw
LeftSkeletonFinger			leftSkeletonFinger;						// Struct with data on left skeleton finger to track/draw
Vector						skeletonBoneStartNorm;					// Start pos of bone normalized
Vector						skeletonBoneEndNorm;					// End pos of bone normalized
cVector3d					skeletonBoneStart;						// Start pos of bone
cVector3d					skeletonBoneEndTemp;					// Temporary variable for end of bone
cVector3d					skeletonBoneEnd;						// End pos of bone
RightSkeletonHand			rightSkeletonHand;						// Struct with data on right skeleton hand to track/draw
RightSkeletonFinger			rightSkeletonFinger;					// Struct with data on right skeleton finger to track/draw
LeftJointSpheres			leftJoints;								// Spheres to draw joints of left hand
RightJointSpheres			rightJoints;							// Spheres to draw joints of right hand
LeftFingerLines				leftFingers;
RightFingerLines			rightFingers;
float						handScale = 3;	
Vector						leftPalmNormal;
Vector						rightPalmNormal;

float						leapScaleFactor;						
GestureList					pGestureList;							// List of gestures
Gesture						pGesture;								// Gesture
int							gestureCount;							// Number of gestures
int32_t						gestureOfInterest;						// Specific gesture in a given frame
int32_t						lastGestureOfInterest;					// Gesture in last frame
SwipeGesture				swipeGesture;							// Swipe gesture
Vector						swipeDirection;							// Direction of swipe gesture
bool						verticalSwipe = false;					// Vertical swipe (true if swipe motion was vertical)
bool						horizontalSwipe = false;				// Horizontal swipe (true if swipe motion was horizontal)
cBitmap*					leapBitmap;								// Bitmap for leap image
cImage*						leapIm;									// Image from the leap
const float					camera_offset = 20;						// x-axis offset of cameras in millimeters
Vector						boxCenter;
float						boxHeight;								// along leap y [mm]
float						boxWidth;								// along leap x [mm]
float						boxDepth;	
cShapeBox*					leapBox;
double						ratio;									// To scale leap image
double						ratioW;
double						ratioH;
double						bitmapH;
double						bitmapW;
bool						gotBitmapInfo = false;

bool zoomIn = false;
std::vector<float>			horizHandHistory;						// keep track of horizontal position of the hand for hovering
std::vector<float>			vertHandHistory;						// keep track of vertical position of the hand for hovering
float						horizError;
float						vertError;
cVector3d					globalHandCenter;
cGenericCollision*			lineDetector[6];
cCollisionRecorder			lineRecorder[6];
#endif

//------------------------------------------------------------------------------
// ARDUINO COMMUNICATION VARIABLES
//------------------------------------------------------------------------------
char						incomingData[1] = "";			// pre-allocate memory
int							dataLength = 1;
int							readResult = 0;
int							lastReadResult = 0;
cThread*					serialThread;
int							mode = 1;
int							lastMode = 0;
Serial*						SP;

cLabel*						modeLabel;								// Label for displaying command type
string						modeName = "";							// For use in the label
bool						cameraModeOn = false;					// to see if in camera mode
bool						designModeOn = true;					// to see if in design mode
bool						increaseLength = false;
bool						decreaseLength = false;
bool						rotateModeOn = false;
bool						simulationModeOn = false;
bool						redrawTubes = false;
bool						rotating = false;

//------------------------------------------------------------------------------
// DATA COLLECTION VARIABLES
//------------------------------------------------------------------------------
cThread*					dataThread;								// thread to run data collection
cPrecisionClock				clock;
int							fileNum = 1;	
std::ofstream				myFile("test.txt");						// set up file for storing data
void collectData(void);

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS (existing)
//------------------------------------------------------------------------------
void resizeWindow(int w, int h);							// callback when the window display is resized
void keySelect(unsigned char key, int x, int y);			// callback when a key is pressed
void updateGraphics(void);									// callback to render graphic scene
void graphicsTimer(int data);								// callback of GLUT timer
void close(void);											// function that closes the application
void updateHaptics(void);									// main haptics simulation loop
void runKinematics(void);									// to call the kinematics function
void runSerialComm(void);									// to call serial read function

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS FOR OCULUS
//------------------------------------------------------------------------------
void vectorizeOculusMatrix(int inputMatrix);
void vectorizeViewportParameters(Recti leftVP, Recti rightVP);
void updateOculusSensorReadings(void);							// update sensor readings from oculus


static void createHmdInfo(HMDInfo *hinfo)
{
    OVR::Service::NetClient* CAPI_pNetClient = OVR::Service::NetClient::GetInstance();
	OVR::Service::HMDNetworkInfo netInfo;
	int index = 0;
	CAPI_pNetClient->Hmd_Create(index, &netInfo);
	int netId = 0;
	CAPI_pNetClient->Hmd_GetHmdInfo(0, hinfo);
}


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS FOR LEAP
//------------------------------------------------------------------------------
void updateLeap(void);
void eraseRightHand(void);
void eraseLeftHand(void);
void showRightHand(void);
void showLeftHand(void);
void drawLeftHand(LeftSkeletonHand leftSkeletonHand);
void drawRightHand(RightSkeletonHand rightSkeletonHand);
void respondToSwipe(void);
void showInteractionBox(InteractionBox interactionBox);

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS FOR CTR
//------------------------------------------------------------------------------
void drawCTR(float length_straight, float length_curved, float curvature_radius, float diameter);
void drawTubes(ConcentricTubeSet set);
void drawSimulation(ConcentricTubeSet set);
void updateInsertionPoint(ConcentricTubeSet set, float insertPointX, float insertPointY, float insertPointZ);
void updateOrientation(ConcentricTubeSet set, cVector3d &axisToRotateAbout);
void updateTubeLength(ConcentricTubeSet &set, int tubeIndex, float deltaLs, float deltaLc);
void updateTubeCurvature(ConcentricTubeSet &set, int tubeIndex, cVector3d a1, cVector3d a2, cVector3d a3, cVector3d a4, float dotProd);
void updateTubeMaterial(ConcentricTubeSet &set, int tubeIndex, int materialNumber);
void updateTubeMaterialAndConfig(ConcentricTubeSet &set);
void updateTubeAlpha(ConcentricTubeSet &set, int tubeIndex, double angleChange);
void calculateConfig(ConcentricTubeSet &set);
void simulate(void);
cMatrix3d calculateRotMat(cVector3d pA, cVector3d pB);

int findCurves(vector<cVector3d> &circParam);
int curveFunc(const gsl_vector * x, void *circParam, gsl_vector * f);
cVector3d computeCurveParam(ConcentricTubeSet &set,  cVector3d a1, cVector3d a2, cVector3d v);
void computeTubeParam(ConcentricTubeSet &set, void *circParam);
cVector3d computeEndPoint(cVector3d tubeStartPoint, cVector3d tubeEndPoint, cVector3d curveMidPoint, float lineLength);
void drawInitCTR(ConcentricTubeSet &set);
void updateInitOrientation(ConcentricTubeSet set, cVector3d &axisToRotateAbout, double angle);
void computeInitOrientation(ConcentricTubeSet set);
void computeOptPath(ConcentricTubeSet &set, void *circParam);
void saveTube(const char *path, const ConcentricTubeSet::tube *t);
void loadTube(const char *path, ConcentricTubeSet::tube *t);
void printTube(ConcentricTubeSet set);
void reinitialize(void);
void getClutchOffset(void);

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS FOR OTHER
//------------------------------------------------------------------------------
cVector3d difference;
void checkGrip(void);
void setTransparent(cMultiMesh* transparentMesh);
void rotateLeftRight(int dir, cShapeSphere* objectToRotate);										// Rotate object left or right
void rotateUpDown(int dir, cShapeSphere* objectToRotate);											// Rotate object up or down
void rotateCwCcw(int dir, cShapeSphere* objectToRotate);											// Rotate object clockwise or counterclockwise
void rotateAll(void);																				// Rotate both the mesh and the tube
void colorPixels(unsigned int x, unsigned int y);
void zoom(float deltaOffset);
void checkIndexCollision(void);
void showDefaultView(cMatrix3d defaultRot, float defaultOff);
void setShowMaterialLabels(void);

//==============================================================================
/*
    CTR.cpp

    This application will render a patient-specific environment. The user can 
	then view and interact with a concentric tube robot design within this
	specific environment. The kinematics will be computed in a separate loop
	as the tube parameters are changed. 

*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 01-mydevice" << endl;
    cout << "Copyright 2003-2014" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[x] - Exit application" << endl;
	cout << "[r] - Rotate model right" << endl;
	cout << "[l] - Rotate model left" << endl;
	cout << "[u] - Rotate model up" << endl;
	cout << "[d] - Rotate model down" << endl;
	cout << "[c] - Rotate model clockwise" << endl;
	cout << "[w] - Rotate model counter clockwise" << endl;
	cout << "[1] - Rotate tube right" << endl;
	cout << "[2] - Rotate tube left" << endl;
	cout << "[3] - Rotate tube up" << endl;
	cout << "[4] - Rotate tube down" << endl;
	cout << "[5] - Rotate tube clockwise" << endl;
	cout << "[6] - Rotate tube counter clockwise" << endl;
    cout << endl << endl;

	//--------------------------------------------------------------------------
    // SETUP FOR SERIAL COMMUNICATION WITH ARDUINO
    //--------------------------------------------------------------------------
	SP = new Serial("\\\\.\\COM50");    // adjust as needed

	if (SP->IsConnected()) {
		printf("Arduino connected \n");
	}


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


	//--------------------------------------------------------------------------
	// SET UP OCCULUS RIFT
	//--------------------------------------------------------------------------
	// Initialize LibOVR, and the Rift
	ovr_Initialize();
	pHMD = ovrHmd_Create(0);

	if(pHMD)
	{
		cout << "Rift detected" << endl;
		// Get more details about HMD
		ovrSizei resolution = pHMD ->Resolution;
		ovrHmdType Type =  pHMD ->Type;
		

	} else {
		cout << "No rift detected" << endl;
	}
	// Start the sensor which provides the Rift’s pose and motion.
	ovrHmd_ConfigureTracking(pHMD, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | 
								  ovrTrackingCap_Position, 0);


	// Direct rendering from a window handle to the Hmd.
	// Not required if ovrHmdCap_ExtendDesktop flag is set.
	if(pHMD->HmdCaps & ovrHmdCap_ExtendDesktop) {
		printf("running in \"extended desktop\" mode\n");
	} else {
		ovrHmd_AttachToWindow(pHMD, GetActiveWindow(), NULL, NULL);
		printf("attached to window \n");
	}


	//----------------Configure Stereo settings----------------------
	stereo.SetStereoMode(StereoConfig::StereoMode::Stereo_LeftRight_Multipass);
	// Get info for rendering to HMD
	createHmdInfo(&pHMDInfo);
	pDefaultProfile = *ProfileManager::GetInstance()->GetDefaultUserProfile(&pHMDInfo);
	pHMDRenderInfo = GenerateHmdRenderInfoFromHmdInfo (pHMDInfo, pDefaultProfile, Distortion_CatmullRom10, EyeCup_LAST);
	stereo.SetHmdRenderInfo(pHMDRenderInfo);
	rendertargetSharedByBothEyes = true;
	// Calculate recommended texture size
	textureSizeRec = CalculateRecommendedTextureSize(pHMDRenderInfo,rendertargetSharedByBothEyes, 1.0f);	
	// Set actual texture size
	stereo.SetRendertargetSize(textureSizeRec,rendertargetSharedByBothEyes);
	// Get full set of stereo rendering parameters for each eye
	leftEyeParam = stereo.GetEyeRenderParams(StereoEye_Left);
	rightEyeParam = stereo.GetEyeRenderParams(StereoEye_Right);
	// Left eye rendering parameters
	leftVP = leftEyeParam.StereoEye.RenderedViewport;						//Rendered VP
	leftProjection = leftEyeParam.StereoEye.RenderedProjection;				//Projection matrix
	leftTranslation = leftEyeParam.StereoEye.HmdToEyeViewOffset;			//translation to apply to view matrix
	leftDistortion = leftEyeParam.StereoEye.Distortion;						//distortion on physical display
	leftDistortionVP = leftEyeParam.StereoEye.DistortionViewport;			//VP on physical display
	leftFov = leftEyeParam.StereoEye.Fov;
	// Right eye rendering parameters
	rightVP = rightEyeParam.StereoEye.RenderedViewport;
	rightProjection = rightEyeParam.StereoEye.RenderedProjection;
	rightTranslation = rightEyeParam.StereoEye.HmdToEyeViewOffset;
	rightDistortion = rightEyeParam.StereoEye.Distortion;
	rightDistortionVP = rightEyeParam.StereoEye.DistortionViewport;
	rightFov = rightEyeParam.StereoEye.Fov;
	//Vectorized the matrices so they can be sent to the cCamera class
	vectorizeOculusMatrix(projLeft);
	vectorizeOculusMatrix(transLeft);
	vectorizeOculusMatrix(projRight);
	vectorizeOculusMatrix(transRight);
	vectorizeViewportParameters(leftVP, rightVP);

	simulationRunning = true;

	
#ifdef USE_LEAP
	//--------------------------------------------------------------------------
	// SET UP LEAP MOTION CONTROL
	//--------------------------------------------------------------------------
	if(controller.isConnected()){
		printf("Leap controller connected \n");
	}
	if(controller.isServiceConnected()){
		printf("Leap controller service connected \n");
	}
	controller.setPolicy(Controller::POLICY_OPTIMIZE_HMD);
	controller.setPolicy(Controller::POLICY_IMAGES);
	
	controller.enableGesture(Gesture::TYPE_SWIPE);						// Enable Swipe Gesture
	controller.config().setFloat("Gesture.Swipe.MinLength", 120.0);		// Set parameters of swipe
	controller.config().setFloat("Gesture.Swipe.MinVelocity", 300);
	controller.config().save();
	controller.enableGesture(Gesture::TYPE_CIRCLE);
#endif

	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------
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
    glutSetWindowTitle("CHAI3D");

    // set fullscreen mode
    if (fullscreen)
    {
        glutFullScreen();
    }


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set( cVector3d (0.0, 0.0, 0.10),    // camera position (eye)					
                 cVector3d (0.0, 0.0, 0.0),    // look at position (target)
                 cVector3d (0.0, 1.0, 0.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
    camera->setStereoMode(stereoMode);

	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------
	handler = new cHapticDeviceHandler();						// create haptic device handler
	int numDevices = handler->getNumDevices();
	printf("num devices: %i \n", numDevices);

	//handler->getDevice(hapticDevice, 0);						// get a handle to first haptic device
	hapticDevice = std::shared_ptr < cGenericHapticDevice > ((cGenericHapticDevice *)(new cPhantomDeviceWithClutch(0)));
	hapticDevice->open();
	cHapticDeviceInfo info = hapticDevice->getSpecifications();	// retrieve info about current device
	
	// create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
	camera->addChild(tool);
	tool->setHapticDevice(hapticDevice);						// connect haptic device to virtual tool
	tool->setWorkspaceRadius(0.7);								// map physical workspace of haptic device to virtual workspace
	tool->setRadius(toolRadius);								// define radius for sphere (virtual tool)
	tool->setShowEnabled(true);
	tool->start();						// start haptic tool

	// device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    maxStiffness = info.m_maxLinearStiffness / workspaceScaleFactor;
	printf("maxStiffness: %f \n", maxStiffness);
	//--------------------------------------------------------------------------
	// CAMERA FOR OCULUS 
	//--------------------------------------------------------------------------
	camera->setUseOculus(false);
	camera->setOculusMatrices(projectionLeftVectorized, translationLeftVectorized, projectionRightVectorized, translationRightVectorized);
	camera->setOculusViewports(leftViewportParams, rightViewportParams);
	camera->setUseOculus(true);
	baseRotation = camera->getLocalRot();
	startingRotation = baseRotation;
	zoomedRotation = startingRotation;
	usingOculus = true;

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(userEyeSeparation);
	camera->setStereoFocalLength(userEyeFocalLength);

	// Set up lighting conditions
    light = new cDirectionalLight(world);					// create a directional light source
    world->addChild(light);									// insert light source inside world
    light->setEnabled(true);								// enable light source                 
    light->setDir(-1.0, 0.0, 0.0);							// define direction of light beam

	// Create objects to display
    cursor = new cShapeSphere(toolRadius);					// create a sphere (cursor) to represent the haptic device
    camera->addChild(cursor);								// insert cursor inside world
	//cursor->setLocalPos(0,0,0);
	cursor->setEnabled(false);
    velocity = new cShapeLine(cVector3d(0,0,0),				// create small line to illustrate the velocity of the haptic device
                              cVector3d(0,0,0));			
    world->addChild(velocity);								// insert line inside world

	// For testing tangent line 
	tanLine = new cShapeLine(cVector3d(0,0,0), cVector3d(1,0,0));
	cursor->addChild(tanLine);
	tanLine->m_colorPointA.setPinkDeep();
	tanLine->m_colorPointB.setPinkDeep();
	tanLine->setLineWidth(10);
	tanLine->setEnabled(false);

	globalTanLine = new cShapeLine(cVector3d(0,0,0), cVector3d(1,0,0));
	world->addChild(globalTanLine);
	globalTanLine->m_colorPointA.setWhite();
	globalTanLine->m_colorPointB.setWhite();
	globalTanLine->setLineWidth(10);
	globalTanLine->setEnabled(false);


	//--------------------------------------------------------------------------
	// ----------------- Create mesh with 3D models ----------------------------
	//--------------------------------------------------------------------------
	// Add sphere to act as the overall parent and origin
	originSphere = new cShapeSphere(0.05);
	world->addChild(originSphere);
	cMaterial originColor;
	originColor.setPinkHot();
	originSphere->setMaterial(originColor);
	originSphere->setEnabled(false);
	originSphere->setLocalPos(0,0,-0.2);
		//***** testing for setting default view ******
	defaultRotation = originSphere->getLocalRot();

	// Add sphere for rotating model
	rotateSphere = new cShapeSphere(0.1);
	world->addChild(rotateSphere);
	rotateSphere->setEnabled(false);
	rotateSphere->setLocalPos(originSphere->getLocalPos());
	cGenericEffect* rotateEffect;
	rotateEffect = new cEffectSurface(rotateSphere);                // create a haptic effect
	rotateSphere->addEffect(rotateEffect);                          // add effect to object

	// Add sphere to act as parent for hands
	originSphere2 = new cShapeSphere(0.05);
	camera->addChild(originSphere2);			// Make camera the parent
	originSphere2->setMaterial(originColor);
	originSphere2->setEnabled(false);
	originSphere2->setLocalPos(0,0,0);       //******************UNSURE WHAT TO SET THIS AS...***********************

	//----------------------------------
	//------ Add bone mesh -------------
	//----------------------------------
	a_mesh = new cMultiMesh();
	originSphere->addChild(a_mesh);	
	a_mesh->setEnabled(true);
	const string bone_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/BoneReduced2/BoneReduced2.obj";
	if(cLoadFileOBJ(a_mesh, bone_fileName	 )){
		printf("--------> bone file loaded!! <-------- \n");
	}
	a_mesh->setShowEnabled(true);
	a_mesh->setUseTransparency(false);
	a_mesh->setUseVertexColors(true);
	a_mesh->setUseMaterial(true);
	cColorf vertColor; 
	vertColor.setGray();
	a_mesh->setVertexColor(vertColor);	
	meshCenter = a_mesh->getBoundaryCenter();
	a_mesh->translate(-meshCenter);									// Center the center of the mesh at the global origin
	// Add haptic effect to mesh
	//a_mesh->computeBoundaryBox(true);
	//a_mesh->createAABBCollisionDetector(toolRadius);
	//a_mesh->setStiffness(0.3*maxStiffness, true);

	//----------------------------------
	//-------- Add kidney mesh ---------
	//----------------------------------
	kidney_mesh = new cMultiMesh();
	originSphere->addChild(kidney_mesh);	
	kidney_mesh->setEnabled(true);
	const string kidney_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/KidneyReduced2b/KidneyReduced2b.obj";
	if(cLoadFileOBJ(kidney_mesh, kidney_fileName	 )){
		printf("--------> kidney file loaded!! <-------- \n");
	}
	kidney_mesh->setShowEnabled(true);
	kidney_mesh->setUseTransparency(false);
	kidney_mesh->setUseVertexColors(true);
	vertColor.setPurpleAmethyst();
	kidney_mesh->setVertexColor(vertColor);
	kidney_mesh->translate(-meshCenter);
	// Add haptic effect to mesh
	/*kidney_mesh->computeBoundaryBox(true);
	kidney_mesh->createAABBCollisionDetector(toolRadius);
	kidney_mesh->setStiffness(0.3*maxStiffness, true);*/

	//----------------------------------
	//--------- Add liver mesh ---------
	//----------------------------------
	liver_mesh = new cMultiMesh();
	originSphere->addChild(liver_mesh);	
	liver_mesh->setEnabled(true);
	const string liver_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/LiverReduced2b/LiverReduced2b.obj";
	if(cLoadFileOBJ(liver_mesh, liver_fileName	 )){
		printf("--------> liver file loaded!! <-------- \n");
	}
	liver_mesh->setShowEnabled(true);
	liver_mesh->setUseVertexColors(true);
	liver_mesh->setUseMaterial(true);
	vertColor.setGreenMediumSpring();
	liver_mesh->setVertexColor(vertColor);
	liver_mesh->translate(-meshCenter);

	//----------------------------------
	//--------- Add stone mesh ---------
	//----------------------------------
	stone_mesh = new cMultiMesh();
	originSphere->addChild(stone_mesh);	
	stone_mesh->setEnabled(true);
	const string stone_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/kidney_Stone/kidney_Stone.obj";
	if(cLoadFileOBJ(stone_mesh, stone_fileName	 )){
		printf("--------> stone file loaded!! <-------- \n");
	}
	stone_mesh->translate(0,-.02,.065);
	stone_mesh->setShowEnabled(true);
	stone_mesh->setUseVertexColors(true);
	stone_mesh->setUseMaterial(true);
	vertColor.setYellowGold();
	stone_mesh->setVertexColor(vertColor);
	stone_mesh->translate(-meshCenter);

	//----------------------------------
	//------ Add renal pelvis mesh -----
	//----------------------------------
	renal_pelvis_mesh = new cMultiMesh();
	originSphere->addChild(renal_pelvis_mesh);	
	renal_pelvis_mesh->setEnabled(true);
	const string renal_pelvis_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/renalPelvisReduced2/renalPelvisReduced2.obj";
	if(cLoadFileOBJ(renal_pelvis_mesh, renal_pelvis_fileName)){
		printf("--------> renal pelvis file loaded!! <-------- \n");
	}
	renal_pelvis_mesh->setShowEnabled(true);
	renal_pelvis_mesh->setUseVertexColors(true);
	renal_pelvis_mesh->setUseMaterial(true);
	vertColor.setBlue();
	renal_pelvis_mesh->setVertexColor(vertColor);
	renal_pelvis_mesh->translate(-meshCenter);

	//----------------------------------
	//------ Add right kidney mesh -----
	//----------------------------------
	right_kidney_mesh = new cMultiMesh();
	originSphere->addChild(right_kidney_mesh);	
	right_kidney_mesh->setEnabled(true);
	const string right_kidney_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/right_kidney2/right_kidney2.obj";
	if(cLoadFileOBJ(right_kidney_mesh, right_kidney_fileName	 )){
		printf("--------> right kidney file loaded!! <-------- \n");
	}
	right_kidney_mesh->setShowEnabled(true);
	right_kidney_mesh->setUseVertexColors(true);
	right_kidney_mesh->setUseMaterial(true);
	vertColor.setPinkDeep();
	right_kidney_mesh->setVertexColor(vertColor);
	right_kidney_mesh->setTransparencyLevel(0.3);
	right_kidney_mesh->translate(-meshCenter);

	//----------------------------------
	//---------- Add skin mesh ---------
	//----------------------------------
	skin_mesh = new cMultiMesh();
	originSphere->addChild(skin_mesh);	
	skin_mesh->setEnabled(true);
	const string skin_fileName	= "C:/Users/Tania/Documents/FilesForVisualization/SkinV3Reduced2/SkinV3Reduced2.obj";
	if(cLoadFileOBJ(skin_mesh, skin_fileName	 )){
		printf("--------> skin file loaded!! <-------- \n");
	}
	skin_mesh->setShowEnabled(true);
	skin_mesh->setUseVertexColors(true);
	skin_mesh->setUseMaterial(true);
	vertColor.setYellowPeachPuff();
	skin_mesh->setVertexColor(vertColor);
	skin_mesh->setTransparencyLevel(0.3);
	skin_mesh->translate(-meshCenter);

	// for debugging
	// test turning off all haptic effects related to origin sphere (by using false,true)
	originSphere->setHapticEnabled(false,true);
	originSphere->setShowEnabled(false,false);
	originSphere->setEnabled(true,true);


	//CTRCollisionSetting.m_checkHapticObjects = false;
	

	//----------------------------------
	//----- Add sphere for testing -----
	//----------------------------------
	posSphere = new cShapeSphere(0.01);
	world->addChild(posSphere);
	posSphere->m_material->setYellow();
	posSphere->setEnabled(false);


	//----------------------------------
	//- Add sphere for origin of tube --
	//----------------------------------
	tubeStartSphere = new cShapeSphere(0.03);
	//tubeStartSphere->setLocalPos(0,-0.1,-0.2);
	//world->addChild(tubeStartSphere);
	originSphere->addChild(tubeStartSphere);
	tubeStartSphere->setLocalPos(0,-0.1,0);
	tubeStartSphere->m_material->setPink();
	tubeStartSphere->setEnabled(false);

	//set tube colors
	tubeColor[0].setOrangeLightSalmon();
	tubeColor[1].setPinkMediumVioletRed();
	tubeColor[2].setBlueCyan();
	tubeColor[3].setGreenYellow();

	lineColor[0].setOrangeLightSalmon();
	lineColor[1].setPinkMediumVioletRed();
	lineColor[2].setBlueCyan();
	lineColor[3].setGreenYellow();

	//// add another test sphere
	testSphere = new cShapeSphere(0.1);
	//camera->addChild(testSphere);
	//originSphere->addChild(testSphere);
	world->addChild(testSphere);
	testSphere->setLocalPos(0,0,-.20);
	testSphere->m_material->setPurpleDarkOrchid();	
	testSphere->m_material->setStiffness(.02*maxStiffness);
	testSphere->setHapticEnabled(true);
	testSphere->setEnabled(false);

	/*testTool = new cToolCursor(world);
	testSphere->addChild(testTool);
	testTool->setRadius(0.1);
	testTool->setWorkspaceRadius(0.7);
	testTool->initialize();*/

	// For debugging
	//debugLine = new cShapeLine(cVector3d(0,0,0), cVector3d(1,0,0));
	//originSphere->addChild(debugLine);
	//debugLine->setLineWidth(1.0);
	////debugLine->setEnabled(false);
	//debugLine->m_colorPointA.setGreenChartreuse();
	//debugLine->m_colorPointB.setGreenChartreuse();

	
	//----------------------------------
	//- Add sphere for selected points -
	//----------------------------------
	for(int i=0; i<10; i++) {
		pointSphere[i] = new cShapeSphere(0.01);
		//pointSphere[i] = new cShapeSphere(0.001);
		originSphere->addChild(pointSphere[i]);
		pointSphere[i]->m_material->setWhiteBeige();
		pointSphere[i]->setLocalPos(0,0,0);
		pointSphere[i]->setEnabled(false);
	}

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------
    cFont *font = NEW_CFONTCALIBRI36();					// create a font
	commandLabel = new cLabel(font);					// label for command
	camera->m_frontLayer->addChild(commandLabel);
	commandLabel->setLocalPos(330,680,0); //(330,340,0);

	materialLabel = new cLabel(font);					// label for material
	camera->m_frontLayer->addChild(materialLabel);
	cVector3d labelPos = commandLabel->getLocalPos();
	materialLabel->setLocalPos(labelPos(0),labelPos(1)-25,labelPos(2));

	modeLabel = new cLabel(font);						// label for mode
	camera->m_frontLayer->addChild(modeLabel);
	modeLabel->setLocalPos(labelPos(0),labelPos(1)+25,labelPos(2));

	cFont* smFont = NEW_CFONTCALIBRI26();
	warningLabel = new cLabel(smFont);					// label for warning
	camera->m_frontLayer->addChild(warningLabel);		
	warningLabel->setLocalPos(labelPos(0),labelPos(1)-50,labelPos(2));
	warningLabel->setEnabled(false);

	alphaLabel = new cLabel(smFont);
	camera->m_frontLayer->addChild(alphaLabel);
	alphaLabel->setLocalPos(labelPos(0),labelPos(1)-50,labelPos(2));
	alphaLabel->setEnabled(false);
	
	cFont* lgFont = NEW_CFONTCALIBRI36();
	waitLabel = new cLabel(lgFont);
	camera->m_frontLayer->addChild(waitLabel);
	waitLabel->setLocalPos(labelPos(0) + 200,labelPos(1) - 250,0); // y used to be +250
	waitLabel->setEnabled(false);

	// -------- create levels to display the different material options --------
	cColorf panelColor;
	panelColor.setGrayDim();

	for(int i=0; i<6; i++) {
		materialOptionDisplay[i] = new cLevel();
		camera->m_frontLayer->addChild(materialOptionDisplay[i]);
		materialOptionDisplay[i]->m_colorInactive.setGrayDim();
		materialOptionDisplay[i]->m_colorActive.setYellowGold();
		materialOptionDisplay[i]->setNumIncrements(30.0);
		materialOptionDisplay[i]->setRange(0.0, 30.0);
		materialOptionDisplay[i]->setValue(0.0);
		materialOptionDisplay[i]->setSingleIncrementDisplay(false);
		materialOptionDisplay[i]->setEnabled(false);
	}
	// Set positions of all levels
	materialOptionDisplay[0]->setLocalPos(1200,300,0);
	materialOptionDisplay[3]->setLocalPos(1200,302,0);
	materialOptionDisplay[1]->setLocalPos(1200,420,0);
	materialOptionDisplay[4]->setLocalPos(1200,422,0);
	materialOptionDisplay[2]->setLocalPos(1200,540,0);
	materialOptionDisplay[5]->setLocalPos(1200,542,0);
	
	// create labels to display on top of each level
	for(int i=0; i<3; i++) {
		materialOptionLabel[i] = new cLabel(font);					
		camera->m_frontLayer->addChild(materialOptionLabel[i]);		
		materialOptionLabel[i]->setLocalPos(materialOptionDisplay[i]->getLocalPos()(0)+35,materialOptionDisplay[i]->getLocalPos()(1)+20,0);
		materialOptionLabel[i]->rotateWigetDeg(90);
		materialOptionLabel[i]->m_fontColor.setPurple();
		materialOptionLabel[i]->setColor(panelColor);
		materialOptionLabel[i]->setEnabled(false);
	}
	materialOptionLabel[0]->setString("Nitinol");
	materialOptionLabel[1]->setString("PEBA");
	materialOptionLabel[2]->setString("Accura");
	
	//------------------------------------------------------------------------------
	// Set up leap graphics
	//------------------------------------------------------------------------------	
	// For image pass through
	//leapBitmap = new cBitmap();
	//camera->m_frontLayer->addChild(leapBitmap);	
	
	// left hand
#ifdef USE_LEAP
	leftCenterSphere = new cShapeSphere(0.03);						// create a sphere to represent the center of the hand
	originSphere2->addChild(leftCenterSphere);						// insert handCenterSphere sphere inside world
	cMaterial leftSphereMaterial;
	leftSphereMaterial.setColorf(1.0f, 0.8f, 0.7f, 0.8);
	leftCenterSphere->setMaterial(leftSphereMaterial);
	leftCenterSphere->setEnabled(false);
	// right hand
	rightCenterSphere = new cShapeSphere(0.03);						// create a sphere to represent the center of the hand
	originSphere2->addChild(rightCenterSphere);						// insert handCenterSphere sphere inside world
	cMaterial rightSphereMaterial;
	rightSphereMaterial.setColorf(1.0f, 0.8f, 0.7f, 0.8);
	rightCenterSphere->setMaterial(rightSphereMaterial);
	rightCenterSphere->setEnabled(false);
	// Fingers (left hand)
	for (int i = 0; i < 25; i++) {
		leftJoints.leftJointSphere[i] = new cShapeSphere(0.01);
		originSphere2->addChild(leftJoints.leftJointSphere[i]);
		cMaterial jointColor;
		jointColor.setColorf(1.0f, 0.8f, 0.7f, 0.8);
		leftJoints.leftJointSphere[i]->setMaterial(jointColor);
		leftJoints.leftJointSphere[i]->setUseMaterial(true);
		leftJoints.leftJointSphere[i]->setEnabled(false);

		leftFingers.leftFingerLine[i] = new cShapeLine(0,0);
		originSphere2->addChild(leftFingers.leftFingerLine[i]);
		leftFingers.leftFingerLine[i]->setLineWidth(10);
		cColorf boneColor;
		boneColor.set(1.0f, 0.8f, 0.7f, 0.8);
		leftFingers.leftFingerLine[i]->m_colorPointA = boneColor;
		leftFingers.leftFingerLine[i]->m_colorPointB = boneColor;
		leftFingers.leftFingerLine[i]->setEnabled(false);
	}
	// Fingers (right hand)
	for (int i = 0; i < 25; i++) {
		rightJoints.rightJointSphere[i] = new cShapeSphere(0.01);
		originSphere2->addChild(rightJoints.rightJointSphere[i]);
		cMaterial jointColor;
		jointColor.setColorf(1.0f, 0.8f, 0.7f, 0.8);
		rightJoints.rightJointSphere[i]->setMaterial(jointColor);
		rightJoints.rightJointSphere[i]->setEnabled(false);


		rightFingers.rightFingerLine[i] = new cShapeLine(0,0);
		originSphere2->addChild(rightFingers.rightFingerLine[i]);
		rightFingers.rightFingerLine[i]->setLineWidth(10);
		cColorf boneColor;
		boneColor.set(1.0f, 0.8f, 0.7f, 0.8);
		rightFingers.rightFingerLine[i]->m_colorPointA = boneColor;
		rightFingers.rightFingerLine[i]->m_colorPointB = boneColor;
		rightFingers.rightFingerLine[i]->setEnabled(false);
	}
#endif
	// Collision detector
	//indexDetector = new cGenericCollision();
	//rightFingers.rightFingerLine[4]->setCollisionDetector(indexDetector);

	//--------------------------------------------------------------------------
    // SET UP CONCENTRIC TUBE ROBOT PARAMETERS
    //--------------------------------------------------------------------------
	//Set up possible materials to use
	// NITINOL
	materialNameVec[0] = "Nitinol";
	EVec[0] = 50000000000;
	vVec[0] = 0.33;
	maxStrainVec[0] = 0.08;
	maxLength[0] = .4; //[m]
	minWallThickness[0] = .0003;
	// PEBA 2301
	materialNameVec[1] = "PEBA 2301";
	EVec[1] = 75;
	vVec[1] = 0.4;				// estimated based on poisson's ratio of polyethylene/nylons/plastics
	maxStrainVec[1] = .11;
	maxLength[1] = .350; //[m]
	minWallThickness[1] = .0007;
	// ACCURA 25
	materialNameVec[2] = "Accura 25";
	EVec[2] = 1625;
	vVec[2] = 0.4;				// estimated based on poisson's ratio of polypropylene/ABS
	maxStrainVec[2] = .04;
	maxLength[2] = .350; 
	minWallThickness[2] = .0005;
	
	// Call function to set up all tube parameters
	setTubeParams(set);

	for(int i=0; i<nTubes; i++) {
		ConcentricTubeSet::tube t;
		
		t.alpha = alpha_init[i];
		t.OD = OD_init[i];
		t.ID = ID_init[i];
		t.E = E_init[i];
		t.v = v_init[i];
		t.kappa = kappa_init[i];
		t.Beta = Beta_init[i];
		t.Lc = Lc_init[i];
		t.Ls = Ls_init[i];
		t.materialNum = 0;		// set initial material to Nitinol

		t.moment_guess = 0;
		t.fixedStepSize = true;

		set.addTube(t);
	}
	
	// Call function to set up material display properties
	for(int i=0; i<nTubes; i++) {
		updateTubeMaterial(set, i, 0);
	}

	//Set up other parameters
	for(int i=0; i<6; i++) {
		lineGripped[i] = false;
		lineGrippedForLengthChange[i] = false;
	}

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
	// create a thread which starts computing the kinematics
    kinematicsThread = new cThread();



	////////////////////////////////////////////////////////////////
	// ********************** TEST BED ****************************
	///////////////////////////////////////////////////////////////
#ifdef TESTBED
	kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
#endif




	//char path[150];
	//for(int i=0; i<nTubes; i++) {
	//	sprintf(path, "C:/Users/Tania/Documents/FilesForVisualization/setName%d.txt", i);
	//	ConcentricTubeSet::tube t;
	//	loadTube(path, &t);

	//	//set.m_tubes[i] = t;

	//	//set.m_tubes[i].alpha = 0;
	//	set.m_tubes[i].alpha = t.alpha;
	//	set.m_tubes[i].Beta = t.Beta;
	//	set.m_tubes[i].Lc = t.Lc;
	//	set.m_tubes[i].Ls = t.Ls;
	//	set.m_tubes[i].kappa = t.kappa;

	//}
    //kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
	/*runKinematics();
	printf("loaded data \n");*/


	// create a thread which deals with serial communication with arduino
    serialThread = new cThread();
    serialThread->start(runSerialComm, CTHREAD_PRIORITY_HAPTICS);

	// create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	// precision clock: Start clock running
    clock.reset();
	clock.start();

	// create a thread which starts data collection (don't start until key stroke)
	dataThread = new cThread();

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

		for (int i=0; i < data.size(); i++) {
			// write out time data
			myFile << "timeStamp: " << data[i].time << "\t" << "\t";
			// write out device position
			myFile << "devicePosX: " << data[i].devicePos(0) << "\t" << "\t";
			myFile << "devicePosY: " << data[i].devicePos(1) << "\t" << "\t";
			myFile << "devicePosZ: " << data[i].devicePos(2) << "\t" << "\t";
			// write out device rotation
			myFile << "deviceRot0: " << data[i].deviceRot.getRow(0)(0) << "\t" << "\t";
			myFile << "deviceRot1: " << data[i].deviceRot.getRow(0)(1) << "\t" << "\t";
			myFile << "deviceRot2: " << data[i].deviceRot.getRow(0)(2) << "\t" << "\t";
			myFile << "deviceRot3: " << data[i].deviceRot.getRow(1)(0) << "\t" << "\t";
			myFile << "deviceRot4: " << data[i].deviceRot.getRow(1)(1) << "\t" << "\t";
			myFile << "deviceRot5: " << data[i].deviceRot.getRow(1)(2) << "\t" << "\t";
			myFile << "deviceRot6: " << data[i].deviceRot.getRow(2)(0) << "\t" << "\t";
			myFile << "deviceRot7: " << data[i].deviceRot.getRow(2)(1) << "\t" << "\t";
			myFile << "deviceRot8: " << data[i].deviceRot.getRow(2)(2) << "\t" << "\t";
			// write out CTR base position
			myFile << "CTRPosX: " << data[i].CTRPos(0) << "\t" << "\t";
			myFile << "CTRPosY: " << data[i].CTRPos(1) << "\t" << "\t";
			myFile << "CTRPosZ: " << data[i].CTRPos(2) << "\t" << "\t";
			// write out CTR rotation
			myFile << "CTRRot0: " << data[i].CTRRot.getRow(0)(0) << "\t" << "\t";
			myFile << "CTRRot1: " << data[i].CTRRot.getRow(0)(1) << "\t" << "\t";
			myFile << "CTRRot2: " << data[i].CTRRot.getRow(0)(2) << "\t" << "\t";
			myFile << "CTRRot3: " << data[i].CTRRot.getRow(1)(0) << "\t" << "\t";
			myFile << "CTRRot4: " << data[i].CTRRot.getRow(1)(1) << "\t" << "\t";
			myFile << "CTRRot5: " << data[i].CTRRot.getRow(1)(2) << "\t" << "\t";
			myFile << "CTRRot6: " << data[i].CTRRot.getRow(2)(0) << "\t" << "\t";
			myFile << "CTRRot7: " << data[i].CTRRot.getRow(2)(1) << "\t" << "\t";
			myFile << "CTRRot8: " << data[i].CTRRot.getRow(2)(2) << "\t" << "\t";
			//write out anatomy model position
			myFile << "modelPosX: " << data[i].modelPos(0) << "\t" << "\t";
			myFile << "modelPosY: " << data[i].modelPos(1) << "\t" << "\t";
			myFile << "modelPosZ: " << data[i].modelPos(2) << "\t" << "\t";
			//write out anatomy model rotation
			myFile << "modelRot0: " << data[i].modelRot.getRow(0)(0) << "\t" << "\t";
			myFile << "modelRot1: " << data[i].modelRot.getRow(0)(1) << "\t" << "\t";
			myFile << "modelRot2: " << data[i].modelRot.getRow(0)(2) << "\t" << "\t";
			myFile << "modelRot3: " << data[i].modelRot.getRow(1)(0) << "\t" << "\t";
			myFile << "modelRot4: " << data[i].modelRot.getRow(1)(1) << "\t" << "\t";
			myFile << "modelRot5: " << data[i].modelRot.getRow(1)(2) << "\t" << "\t";
			myFile << "modelRot6: " << data[i].modelRot.getRow(2)(0) << "\t" << "\t";
			myFile << "modelRot7: " << data[i].modelRot.getRow(2)(1) << "\t" << "\t";
			myFile << "modelRot8: " << data[i].modelRot.getRow(2)(2) << "\t" << "\t";
			//write out pedal and button presses
			myFile << "pedalPressed: " << data[i].pedalPressed << "\t" << "\t";
			myFile << "button0Pressed: " << data[i].button0Pressed << "\t" << "\t";
			myFile << "button1Pressed: " << data[i].button1Pressed << "\t" << "\t";
			//write out tube parameters for tube 0
			myFile << "OD0: " << data[i].OD0 << "\t" << "\t";
			myFile << "ID0: " << data[i].ID0 << "\t" << "\t";
			myFile << "kappa0 " << data[i].kappa0 << "\t" << "\t";
			myFile << "alpha0: " << data[i].alpha0 << "\t" << "\t";
			myFile << "Beta0: " << data[i].Beta0 << "\t" << "\t";
			myFile << "Lc0: " << data[i].Lc0 << "\t" << "\t";
			myFile << "Ls0: " << data[i].Ls0 << "\t" << "\t";
			//write out tube parameters for tube 1
			myFile << "OD1: " << data[i].OD1 << "\t" << "\t";
			myFile << "ID1: " << data[i].ID1 << "\t" << "\t";
			myFile << "kappa1 " << data[i].kappa1 << "\t" << "\t";
			myFile << "alpha1: " << data[i].alpha1 << "\t" << "\t";
			myFile << "Beta1: " << data[i].Beta1 << "\t" << "\t";
			myFile << "Lc1: " << data[i].Lc1 << "\t" << "\t";
			myFile << "Ls1: " << data[i].Ls1 << "\t" << "\t";
			//write out tube parameters for tube 2
			myFile << "OD2: " << data[i].OD2 << "\t" << "\t";
			myFile << "ID2: " << data[i].ID2 << "\t" << "\t";
			myFile << "kappa2 " << data[i].kappa2 << "\t" << "\t";
			myFile << "alpha2: " << data[i].alpha2 << "\t" << "\t";
			myFile << "Beta2: " << data[i].Beta2 << "\t" << "\t";
			myFile << "Lc2: " << data[i].Lc2 << "\t" << "\t";
			myFile << "Ls2: " << data[i].Ls2 << "\t" << "\t";
			myFile << "tube0material: " << data[i].materialNum0 << "\t" << "\t";
			myFile << "tube1material: " << data[i].materialNum1 << "\t" << "\t";
			myFile << "tube2material: " << data[i].materialNum2 << "\n";
		}
		myFile.close();
		printf("data written to file \n");

        close();
        exit(0);
    }

	// write data to file without quitting program
	if(key=='w') {
			myFile << "timeStamp" << "\t" << "devicePosX" << "\t" << "devicePosY" << "\t" << "devicePosZ" << "\t" <<
				"deviceRot0" << "\t" << "deviceRot1" << "\t" << "deviceRot2" << "\t" << "deviceRot3" << "\t" << "deviceRot4" << "\t" <<
				"deviceRot5" << "\t" << "deviceRot6" << "\t" << "deviceRot7" << "\t" << "deviceRot8" << "\t" << 
				"CTRPosX" << "\t" << "CTRPosY" << "\t" << "CTRPosZ" << "\t" <<
				"CTRRot0" << "\t" << "CTRRot1" << "\t" << "CTRRot2" << "\t" << "CTRRot3" << "\t" << "CTRRot4" << "\t" <<
				"CTRRot5" << "\t" << "CTRRot6" << "\t" << "CTRRot7" << "\t" << "CTRRot8" << "\t" <<
				"modelPosX" << "\t" << "modelPosY" << "\t" << "modelPosZ" << "\t" <<
				"modelRot0" << "\t" << "modelRot1" << "\t" << "modelRot2" << "\t" << "modelRot3" << "\t" << "modelRot4" << "\t" <<
				"modelRot5" << "\t" << "modelRot6" << "\t" << "modelRot7" << "\t" << "modelRot8" << "\t" <<
				"pedalPressed" << "\t" << "button0Pressed" << "\t" << "button1Pressed" << "\t" <<
				"OD0" << "\t" << "ID0" << "\t" << "kappa0" << "\t" << "alpha0" << "\t" << "Beta0" << "\t" << "Lc0" << "\t" << "Ls0" << "\t" <<
				"OD1" << "\t" << "ID1" << "\t" << "kappa1" << "\t" << "alpha1" << "\t" << "Beta1" << "\t" << "Lc1" << "\t" << "Ls1" << "\t" <<
				"OD2" << "\t" << "ID2" << "\t" << "kappa2" << "\t" << "alpha2" << "\t" << "Beta2" << "\t" << "Lc2" << "\t" << "Ls2" << "\t" <<
				"tube0material" << "\t" << "tube1material" << "\t" << "tube2material" << "\n"; 

		for (int i=0; i < data.size(); i++) {
			// write out time data
			myFile << data[i].time << "\t";
			// write out device position
			myFile << data[i].devicePos(0) << "\t";
			myFile << data[i].devicePos(1) << "\t";
			myFile << data[i].devicePos(2) << "\t";
			// write out device rotation
			myFile << data[i].deviceRot.getRow(0)(0) << "\t";
			myFile << data[i].deviceRot.getRow(0)(1) << "\t";
			myFile << data[i].deviceRot.getRow(0)(2) << "\t";
			myFile << data[i].deviceRot.getRow(1)(0) << "\t";
			myFile << data[i].deviceRot.getRow(1)(1) << "\t";
			myFile << data[i].deviceRot.getRow(1)(2) << "\t";
			myFile << data[i].deviceRot.getRow(2)(0) << "\t";
			myFile << data[i].deviceRot.getRow(2)(1) << "\t";
			myFile << data[i].deviceRot.getRow(2)(2) << "\t";
			// write out CTR base position
			myFile << data[i].CTRPos(0) << "\t";
			myFile << data[i].CTRPos(1) << "\t";
			myFile << data[i].CTRPos(2) << "\t";
			// write out CTR rotation
			myFile << data[i].CTRRot.getRow(0)(0) << "\t";
			myFile << data[i].CTRRot.getRow(0)(1) << "\t";
			myFile << data[i].CTRRot.getRow(0)(2) << "\t";
			myFile << data[i].CTRRot.getRow(1)(0) << "\t";
			myFile << data[i].CTRRot.getRow(1)(1) << "\t";
			myFile << data[i].CTRRot.getRow(1)(2) << "\t";
			myFile << data[i].CTRRot.getRow(2)(0) << "\t";
			myFile << data[i].CTRRot.getRow(2)(1) << "\t";
			myFile << data[i].CTRRot.getRow(2)(2) << "\t";
			//write out anatomy model position
			myFile << data[i].modelPos(0) << "\t";
			myFile << data[i].modelPos(1) << "\t";
			myFile << data[i].modelPos(2) << "\t";
			//write out anatomy model rotation
			myFile << data[i].modelRot.getRow(0)(0) << "\t";
			myFile << data[i].modelRot.getRow(0)(1) << "\t";
			myFile << data[i].modelRot.getRow(0)(2) << "\t";
			myFile << data[i].modelRot.getRow(1)(0) << "\t";
			myFile << data[i].modelRot.getRow(1)(1) << "\t";
			myFile << data[i].modelRot.getRow(1)(2) << "\t";
			myFile << data[i].modelRot.getRow(2)(0) << "\t";
			myFile << data[i].modelRot.getRow(2)(1) << "\t";
			myFile << data[i].modelRot.getRow(2)(2) << "\t";
			//write out pedal and button presses
			myFile << data[i].pedalPressed << "\t";
			myFile << data[i].button0Pressed << "\t";
			myFile << data[i].button1Pressed << "\t";
			//write out tube parameters for tube 0
			myFile << data[i].OD0 << "\t";
			myFile << data[i].ID0 << "\t";
			myFile << data[i].kappa0 << "\t";
			myFile << data[i].alpha0 << "\t";
			myFile << data[i].Beta0 << "\t";
			myFile << data[i].Lc0 << "\t";
			myFile << data[i].Ls0 << "\t";
			//write out tube parameters for tube 1
			myFile << data[i].OD1 << "\t";
			myFile << data[i].ID1 << "\t";
			myFile << data[i].kappa1 << "\t";
			myFile << data[i].alpha1 << "\t";
			myFile << data[i].Beta1 << "\t";
			myFile << data[i].Lc1 << "\t";
			myFile << data[i].Ls1 << "\t";
			//write out tube parameters for tube 2
			myFile << data[i].OD2 << "\t";
			myFile << data[i].ID2 << "\t";
			myFile << data[i].kappa2 << "\t";
			myFile << data[i].alpha2 << "\t";
			myFile << data[i].Beta2 << "\t";
			myFile << data[i].Lc2 << "\t";
			myFile << data[i].Ls2 << "\t";
			myFile << data[i].materialNum0 << "\t";
			myFile << data[i].materialNum1 << "\t";
			myFile << data[i].materialNum2 << "\n";
		}
		myFile.close();
	}

    // option f: toggle fullscreen
    if (key == 'f')
    {
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

	// Rotate model RIGHT
	if (key == 'r')
	{
		int dir = 1;
		rotateLeftRight(dir, originSphere);
	}

	// Rotate model LEFT
	if (key == 'l')
	{
		int dir = -1;
		rotateLeftRight(dir, originSphere);
	}

	// Rotate model UP
	/*if (key == 'u')
	{
		int dir = 1;
		rotateUpDown(dir, originSphere);
	}*/

	// Rotate model DOWN
	if (key == 'd')
	{
		int dir = -1;
		rotateUpDown(dir, originSphere);
	}

	// Rotate model clockwise
	if (key == 'c')
	{
		int dir = 1;
		rotateCwCcw(dir, originSphere);
	}

	// Rotate model counterclockwise
	/*if (key == 'w')
	{
		int dir = -1;
		rotateCwCcw(dir, originSphere);
	}*/

	// Make mesh transparent
	if (key == 't')
	{
		cMultiMesh* transparentMesh;
		transparentMesh = liver_mesh;
		setTransparent(transparentMesh);
	}
	
	// Rotate tube RiGHT
	if (key == '1') 
	{
		int dir = 1;
		rotateLeftRight(dir, tubeStartSphere);
	}

	// Rotate tube LEFT
	if (key == '2')
	{
		int dir = -1;
		rotateLeftRight(dir, tubeStartSphere);
	}

	// Rotate tube UP
	if (key == '3')
	{
		int dir = 1;
		rotateUpDown(dir, tubeStartSphere);
	}

	// Rotate tube DOWN
	if (key == '4')
	{
		int dir = -1;
		rotateUpDown(dir, tubeStartSphere);
	}

	// Rotate tube clockwise
	if (key == '5')
	{
		int dir = 1;
		rotateCwCcw(dir, tubeStartSphere);
	}

	// Rotate tube counterclockwise
	if (key == '6')
	{
		int dir = -1;
		rotateCwCcw(dir, tubeStartSphere);}

	// Rotate model and tube
	if (key == '7') 
	{
		rotateAll();
	}

	// Compute kinematics
	if (key == 'q') 
	{
		calculateConfig(set);
	}

	// Draw
	if (key == 'a') 
	{

		cVector3d a1 = cVector3d(0,0,0);
		cVector3d a2 = cVector3d(-0.0008,-0.0098,0.0587);
		cVector3d a3 = cVector3d(-0.0021,-0.019,0.0763);
		cVector3d a4 = cVector3d(-0.0052,-0.0308,0.0922);
		cVector3d v = cVector3d(-0.0344,-0.1938,-0.9804);
		v = cNormalize(v);
		
		vector<cVector3d> *circParam;
		circParam = new vector<cVector3d>[3];
		circParam->push_back(a1);
		circParam->push_back(a2);
		circParam->push_back(a3);
		circParam->push_back(a4);
		circParam->push_back(v);


		a_mesh->setEnabled(false);
		kidney_mesh->setEnabled(false);
		liver_mesh->setEnabled(false);
		right_kidney_mesh->setEnabled(false);
		stone_mesh->setEnabled(false);
		skin_mesh->setEnabled(false);

		pointSphere[0]->setLocalPos(a1);
		pointSphere[0]->setEnabled(true);
		pointSphere[1]->setLocalPos(a2);
		pointSphere[1]->setEnabled(true);
		pointSphere[2]->setLocalPos(a3);
		pointSphere[2]->setEnabled(true);
		pointSphere[3]->setLocalPos(a4);
		pointSphere[3]->setEnabled(true);


		computeTubeParam(set, circParam);

		

	}

	// Change focal length/distance
	if(key == 'o') {
		userEyeFocalLength = userEyeFocalLength + 0.1;
		camera->setStereoFocalLength(userEyeFocalLength);
		printf("focal length: %f \n",userEyeFocalLength);
	}

	if(key == 'i') {
		userEyeFocalLength = userEyeFocalLength - 0.1;
		if(userEyeFocalLength < 0.1) {
			userEyeFocalLength = 0.1;
		}
		camera->setStereoFocalLength(userEyeFocalLength);
		printf("focal length: %f \n",userEyeFocalLength);
	}
	// Change eye separation
	if(key == 'k') {
		userEyeSeparation = userEyeSeparation + 0.005;
		camera->setStereoEyeSeparation(userEyeSeparation);
		printf("eye separation: %f \n",userEyeSeparation);
	}
	if(key == 'j') {
		userEyeSeparation = userEyeSeparation - 0.005;
		if(userEyeSeparation < 0.001) {
			userEyeSeparation = 0.001;
		}
		camera->setStereoEyeSeparation(userEyeSeparation);
		printf("eye separation: %f \n",userEyeSeparation);
	}
	


	if (key == 'b')
	{
		char *buffer;
		buffer = "4";
		unsigned int nbChar = 1;
		SP->WriteData(buffer, nbChar);
	}

	// Start data collection
	if (key == 's') 
	{
		dataThread->start(collectData, CTHREAD_PRIORITY_HAPTICS);
		printf("data collection started \n");
	}

	// undo last curvature change
	if (key == 'u')
	{
		kappaChange = true;
		commandName = "Change curvature";
		set.m_tubes[lastTube].kappa = kappaLast;
		
		waitLabel->setEnabled(true);
		kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
		waitLabel->setEnabled(false);

		/*if(set.isValidSet) {
			drawTubes(set);
		} */

		kappaChange = false;
	}
}

//------------------------------------------------------------------------------
// Close
//------------------------------------------------------------------------------
void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
    tool->stop();
}

//------------------------------------------------------------------------------
// Graphics timer
//------------------------------------------------------------------------------
void graphicsTimer(int data)
{
    if (simulationRunning)
    {
        glutPostRedisplay();
    }

	// If in simulation mode and it's the first time through
	if(simulationModeOn && !calculated) {

		// Calculate all configurations needed for simulation
		calculateConfig(set);


		//// Stop haptic loop from running
		//simulationRunning = false;
		//tool->stop();
	} else if(simulationModeOn && calculated && readyToDraw) {
		// Stop haptic loop from running
		simulationRunning = false;
		//tool->stop();
	}

	// If configurations have been calculated, draw them for all time steps
	if(readyToDraw) {
		simulate();
		glutPostRedisplay();
	}

	// If returning from simulation mode to design mode, redraw tubes
	if(redrawTubes) {
		// Stop haptic loop from running
		simulationRunning = false;
		tool->stop();
		
		// Redraw tubes
		drawTubes(set);

		// Start haptic loop again
		simulationRunning = true;
		hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
		tool->start();

		redrawTubes = false;
	}

    glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------
// Update graphics
//------------------------------------------------------------------------------
void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    ///////////////////////////////////////////////////////////////////// 
    commandLabel->setString ("Command Type: "+ commandName);		// Display command type
	materialLabel->setString("Material Type: " + materialName);  // Display material type
	modeLabel->setString("Mode: " + modeName);					// Display mode
	warningLabel->setString("Warning: " + warningName);
	alphaLabel->setString("Delta alpha: " + alphaValue);
	waitLabel->setString(waitName);

	//materialOptionDisplay->setLocalPos(1200,100,0);

	/////////////////////////////////////////////////////////////////////
    // UPDATE OCULUS AND LEAP
    /////////////////////////////////////////////////////////////////////
	updateOculusSensorReadings();					// Read Oculus sensors
	//updateLeap();									// Read Leap sensors
	camera->setLocalTransform(headTransform);		// Set camera based on Oculus readings

	// Check on timing for erasing initial points and tangent vector and erase after 10 seconds
	if(otherClock.getCurrentTimeSeconds() > 7) {
		// Erase intial points
		for(int i=0; i<(nTubes+1); i++) {
			pointSphere[i]->setEnabled(false);	
		}
		// Erase initial tangent ghost vector
		globalTanLine->setEnabled(false);
		tubeStartSphere->setEnabled(false);
		otherClock.stop();
		otherClock.reset();
	}



	//static unsigned long printCount = 0;
	//if (printCount % 10 == 0){

	//	/*cMatrix3d deviceRot = tool->getDeviceGlobalRot();
	//	cVector3d axis; double angle;
	//	deviceRot.toAxisAngle(axis, angle);
	//	printf("angle:  %f  \n",angle);*/

	//	/*cVector3d rPos = rightCenterSphere->getLocalPos();
	//	printf("Local x: %f, Local y: %f, Local z: %f \n", rPos(0),rPos(1),rPos(2));
	//	cVector3d modelPos = rightCenterSphere->getGlobalPos();
	//	printf("Global x: %f, Global y: %f, Global z: %f \n", modelPos(0),modelPos(1),modelPos(2));
	//	printf("\n");*/

	//	//printf("Eye Yaw: %f, Pitch: %f, Roll: %f \n", eyeYaw, eyePitch, eyeRoll);

	//	// To output information about fingerID
	//	/*for(Leap::FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); fl++)
	//	std::cout << "Finger ID: " << (*fl).id() << std::endl;*/	
	//}
	//printCount++;


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // render world

	//BEGIN_TIMING(renderView,100);
    camera->renderView(windowW, windowH);
	//END_TIMING(renderView,100);

    // swap buffers
    glutSwapBuffers();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}


//------------------------------------------------------------------------------
// Set up matrices for oculus
//------------------------------------------------------------------------------
void vectorizeOculusMatrix(int inputMatrix)
{
	float output[16];
	Matrix4f input;

	//Figure out which input matrix to vectorize
	if (inputMatrix == projLeft){
		input = leftProjection;
	}
	else if (inputMatrix == transLeft){
		input = leftTranslation;
	}
	else if (inputMatrix == projRight){
		input = rightProjection;
	}
	else if (inputMatrix == transRight){
		input = rightTranslation;
	}
	else{
		return;
	}

	//printf("Input Matrix Number: %d \n", inputMatrix);
	//Vectorize the matrix
	int vectorIndex = 0;
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++){
			output[vectorIndex] = input.M[j][i]; //glMultMatrix uses column major order
			//printf("Next vector output: %f \n", output[vectorIndex]);
			vectorIndex++;
		}
	}

	//Convert output to vector of choice
	if (inputMatrix == projLeft){
		for (int i = 0; i < 16; i++){
			projectionLeftVectorized[i] = output[i];
		}
	}
	else if (inputMatrix == transLeft){
		for (int i = 0; i < 16; i++){
			translationLeftVectorized[i] = output[i];
		}
	}
	else if (inputMatrix == projRight){
		for (int i = 0; i < 16; i++){
			projectionRightVectorized[i] = output[i];
		}
	}
	else if (inputMatrix == transRight){
		for (int i = 0; i < 16; i++){
			translationRightVectorized[i] = output[i];
		}
	}
}

//------------------------------------------------------------------------------
// Set up viewport parameters
//------------------------------------------------------------------------------
void vectorizeViewportParameters(Recti leftVP, Recti rightVP)
{
	// Get x pos, y pos, width and height of rectangle for each eye
	leftViewportParams[0] = leftVP.x;
	leftViewportParams[1] = leftVP.y;
	leftViewportParams[2] = leftVP.w;
	leftViewportParams[3] = leftVP.h;

	rightViewportParams[0] = rightVP.x;
	rightViewportParams[1] = rightVP.y;
	rightViewportParams[2] = rightVP.w;
	rightViewportParams[3] = rightVP.h;
}

//------------------------------------------------------------------------------
// Read sensors from Oculus
//------------------------------------------------------------------------------
void updateOculusSensorReadings(void)
{
	// Query the HMD for the current tracking state.
	ts = ovrHmd_GetTrackingState(pHMD, ovr_GetTimeInSeconds());
	HeadPose = ts.HeadPose;
	
	HeadPose.ThePose.Rotation.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&eyeYaw, &eyePitch, &eyeRoll);

	cVector3d globalX(1, 0, 0);
	cVector3d globalY(0, 1, 0);
	cVector3d globalZ(0, 0, 1);
	headRotation = startingRotation;
	headRotation.rotateAboutGlobalAxisRad(globalZ, (double)eyeRoll);
	headRotation.rotateAboutGlobalAxisRad(globalX, (double)eyePitch);
	headRotation.rotateAboutGlobalAxisRad(globalY, (double)eyeYaw);

	double eyeX = HeadPose.ThePose.Translation.x;
	double eyeY = HeadPose.ThePose.Translation.y;
	double eyeZ = HeadPose.ThePose.Translation.z;
	eyePos = cVector3d(eyeX, eyeY, eyeZ+oculusOffset);

	//printf("x: %f, y: %f, z: %f \n",eyeX,eyeY,eyeZ+oculusOffset);

	cTransform translateTransform = cTransform(eyePos); 
	cTransform rotateTransform = cTransform(headRotation);
	headTransform = translateTransform*rotateTransform;
	 


}

#ifdef USE_LEAP
//------------------------------------------------------------------------------
// Get information from Leap
//------------------------------------------------------------------------------
void updateLeap(void) {
	if( controller.isConnected()) 
	{
		frame = controller.frame();															// Update frame reading
		interactionBox = frame.interactionBox();
		//showInteractionBox(interactionBox);

		pHandList = frame.hands();
		handCount = pHandList.count();
		//printf("hand count: %i \n",handCount);

		// If no hands are visible, then disable graphics
		if(handCount==0) {
			eraseRightHand();
			eraseLeftHand();
		} else if(handCount==1) {
			if(frame.hands()[0].isLeft()) {
				eraseRightHand();			// If only left hand is visible, erase right hand
			} else {
				eraseLeftHand();			// If only right hand is visible, erase left hand
			}
		}

		// Loop through for each hand visible
		for (int i = 0; i < handCount; i++) {
			pHand = frame.hands()[i];																	// get hands in frame
			handID = pHand.id();																		// get hand ID

			// Get information about center of hand
			handCenterNorm = interactionBox.normalizePoint(pHand.palmPosition());
			
			//cVector3d handCenterNormRot = cVector3d(-handCenterNorm.x,-handCenterNorm.z,-handCenterNorm.y);
			//handCenter = handCenterNormRot + cVector3d(0.5,0.5,0.5);
			//printf("norm x: %f, norm y: %f, norm z: %f \n",handCenter(0),handCenter(1),handCenter(2));

			handCenter = cVector3d(handCenterNorm.x - 0.5, 0.5 - handCenterNorm.y, handCenterNorm.z - 0.5);
			handCenter = cMatrix3d(0,1,0,-1,0,0,0,0,-1)*handCenter;
			handCenter = cVector3d(handCenter(0),handCenter(1),handCenter(2));

			//printf("norm x: %f, norm y: %f, norm z: %f \n",handCenter(0),handCenter(1),handCenter(2));
			
			
			
			pFingerList = frame.fingers();
			fingerCount = pFingerList.count();
			//printf("finger count: %i \n", fingerCount);
			// Loop through for each finger 
			for (int j = 0; j < 5; j++) {
				pFinger = pHand.fingers()[j];
				// Loop through for each bone
				for (int k = 0; k < 4; k++) {
					pBoneType = static_cast<Bone::Type>(k);
					pBone = pFinger.bone(pBoneType);
					//std::cout << "Finger index: " << pBone.type() << " " << pBone << std::endl;
					
					// Get START joint for each bone
					Vector boneStartTemp = pBone.prevJoint();
					skeletonBoneStartNorm = interactionBox.normalizePoint(boneStartTemp);			
					skeletonBoneStart = cVector3d(skeletonBoneStartNorm.x - 0.5, 0.5 - skeletonBoneStartNorm.y, skeletonBoneStartNorm.z - 0.5);
					skeletonBoneStart = cMatrix3d(0,1,0,-1,0,0,0,0,-1)*skeletonBoneStart;
					skeletonBoneStart = cVector3d(skeletonBoneStart(0),skeletonBoneStart(1),skeletonBoneStart(2));

					// Get END joint for each bone
					Vector boneEndTemp = pBone.nextJoint();
					skeletonBoneEndNorm = interactionBox.normalizePoint(boneEndTemp);		
					skeletonBoneEnd = cVector3d(skeletonBoneEndNorm.x - 0.5, 0.5 - skeletonBoneEndNorm.y, skeletonBoneEndNorm.z - 0.5);
					skeletonBoneEnd = cMatrix3d(0,1,0,-1,0,0,0,0,-1)*skeletonBoneEnd;
					skeletonBoneEnd = cVector3d(skeletonBoneEnd(0),skeletonBoneEnd(1),skeletonBoneEnd(2));

					// Scale down hand size
					cVector3d distanceStart = (skeletonBoneStart-handCenter)/handScale;
					skeletonBoneStart = handCenter + distanceStart;
					cVector3d distanceEnd = (skeletonBoneEnd-handCenter)/handScale;
					skeletonBoneEnd = handCenter + distanceEnd;

					//printf("skeleton end x: %f,skeleton end y: %f, skeleton end z: %f \n",skeletonBoneEnd(0),skeletonBoneEnd(1),skeletonBoneEnd(2));

					if (pHand.isValid()){
						if(pHand.isLeft()){
							leftSkeletonHand.leftSkeletonFinger[j].leftBoneStart[k] = skeletonBoneStart;
							leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[k] = skeletonBoneEnd;
							//printf("Left Hand \n");
						} else {
							rightSkeletonHand.rightSkeletonFinger[j].rightBoneStart[k] = skeletonBoneStart;
							rightSkeletonHand.rightSkeletonFinger[j].rightBoneEnd[k] = skeletonBoneEnd;
							//printf("Right Hand \n");
						}
					}
				}
			}

			// Determine which hand to draw
			if (pHand != Hand::invalid()) {
				if(pHand.isLeft()){
					showLeftHand();
				} else if(pHand.isRight()) {
					showRightHand();
					
				}
			} 
		}

		


		//------------------------------------------------------------------------------
		//------------------------------ Image Data ------------------------------------
		//------------------------------------------------------------------------------
		//ImageList images = frame.images();
		//Image image;
		//for(int i = 0; i < 2; i++){
		//	image = images[i];
		//	const unsigned char* image_buffer = image.data();
		//	//leapIm = new cImage(image.width(),image.height(), GL_RGBA, GL_UNSIGNED_BYTE);
		//	leapIm = new cImage(image.width(),image.height(), GL_LUMINANCE, GL_UNSIGNED_BYTE);
		//	unsigned int numBytes = image.width() * image.height() * image.bytesPerPixel();
		//	memcpy(leapIm->getData(), image_buffer, numBytes);

		//	//leapIm->saveToFile("C:/Users/Tania/Documents/test.bmp");		// save image to make sure it's correct
		//	//printf("image width %d height %d numBytes %d type %d bbp %d\n", image.width(), image.height(), numBytes, image.format(), image.bytesPerPixel());
	
		//	leapIm->flipHorizontal();
		//	if(leapBitmap->loadFromImage(leapIm)) {
		//		if(image.isValid()) {
		//			
		//		} else {
		//			printf("invalid image \n");
		//		}
		//	} else {
		//		printf("bitmap failed to load \n");
		//	}
		//	leapBitmap->updateBitmapMesh();	
		//	leapBitmap->setUseTransparency(true);
		//	leapBitmap->setTransparencyLevel(0.5,true);

		//	leapBitmap->setSize(windowW,windowH);
		//	
		//	delete leapIm;			
		//}
		//------------------------------------------------------------------------------
		//------------------------------------------------------------------------------


		// Check for any gestures and respond accordingly
		//pGestureList = frame.gestures();
		//gestureCount = pGestureList.count();
		//for(int i=0; i<gestureCount;i++) {
		//	pGesture = frame.gestures()[i];
		//	gestureOfInterest = pGesture.id();
		//	if(pGesture.type()==Gesture::TYPE_SWIPE) {			// Swiping gesture
		//		if(gestureOfInterest==lastGestureOfInterest) {
		//			printf("Swipe detected! \n");
		//			//respondToSwipe();
		//		}
		//	}
		//	lastGestureOfInterest = gestureOfInterest;
		//}


		// Check for gripping
		//checkGrip();

	}
}

//------------------------------------------------------------------------------
// Hand drawing functions
//------------------------------------------------------------------------------
// Erase Right Hand if it's not in view
void eraseRightHand(void) {
	rightCenterSphere->setEnabled(false);

	for (int i = 0; i < 25; i++) {
		rightJoints.rightJointSphere[i]->setEnabled(false);
		rightFingers.rightFingerLine[i]->setEnabled(false);
	}
}
//Erase Left Hand if it's not in view
void eraseLeftHand(void) {
	leftCenterSphere->setEnabled(false);

	for (int i = 0; i < 25; i++) {
		leftJoints.leftJointSphere[i]->setEnabled(false);
		leftFingers.leftFingerLine[i]->setEnabled(false);
	}
}
// Show Right Hand if it's in view
void showRightHand(void) {
	rightCenterSphere->setEnabled(true);
	rightCenterSphere->setLocalPos(handCenter);

	//cVector3d rPos = rightCenterSphere->getLocalPos();
	//printf("x: %f, y: %f, z: %f \n", rPos(0),rPos(1),rPos(2));
	
	for (int i = 1; i < 25; i++) {
		rightJoints.rightJointSphere[i]->setEnabled(true);
		rightFingers.rightFingerLine[i]->setEnabled(true);
	}

	drawRightHand(rightSkeletonHand);


	/*Keep track of hand position for hovering
	Create a vector of x number of values
	push back to add each new value 
	when reached length x, check to see if MSE (or some metric) is small enough
	if so, then consider to be hovering
	also get rid of oldest value from vector*/
	globalHandCenter = rightCenterSphere->getGlobalPos();
	horizHandHistory.push_back(globalHandCenter(0));
	vertHandHistory.push_back(globalHandCenter(1));
	if((horizHandHistory.size())>15) {
		horizError = 0;
		vertError = 0;
		for(int i=0; i<(horizHandHistory.size()); i++) {
			horizError = horizError + (horizHandHistory[i] - 0.4);
			vertError = vertError + (vertHandHistory[i] - 0.4);
		}
		horizError = horizError/(horizHandHistory.size());		// average error between hand center and 0.4
		vertError = vertError/(vertHandHistory.size());			// average error between hand center and 0.4
		if((oculusOffset<0.65) && (oculusOffset>0.55)) {
			if(abs(horizError)<0.07) {
				int dir = 1;
				rotateLeftRight(dir, originSphere);					// rotate model right
			} else if(abs(horizError)>0.73) {
				int dir = -1;
				rotateLeftRight(dir, originSphere);					// rotate model left
			} else if(abs(vertError)<0.07) {
				int dir = 1;										// rotate model up
				rotateUpDown(dir, originSphere);
			} else if(abs(vertError)>0.73) {
				int dir = -1;										// rotate model down
				rotateUpDown(dir, originSphere);
			}
		}
		horizHandHistory.erase(horizHandHistory.begin());		// erase first element
		vertHandHistory.erase(vertHandHistory.begin());			// erase first element
	}


}

// Show Left Hand if it's in view
void showLeftHand(void) {
	leftCenterSphere->setEnabled(true);
	leftCenterSphere->setLocalPos(handCenter);

	for (int i = 1; i < 25; i++) {
		leftJoints.leftJointSphere[i]->setEnabled(true);
		leftFingers.leftFingerLine[i]->setEnabled(true);
	}

	drawLeftHand(leftSkeletonHand);
}

// Draw Skeleton Hand
void drawLeftHand(LeftSkeletonHand leftSkeletonHand) {
	for(int j = 0; j < 5; j++) {		// j=finger number	
		leftJoints.leftJointSphere[5*j+0]->setLocalPos(leftSkeletonHand.leftSkeletonFinger[j].leftBoneStart[0]);
		leftJoints.leftJointSphere[5*j+1]->setLocalPos(leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[0]);
		leftJoints.leftJointSphere[5*j+2]->setLocalPos(leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[1]);
		leftJoints.leftJointSphere[5*j+3]->setLocalPos(leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[2]);
		leftJoints.leftJointSphere[5*j+4]->setLocalPos(leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[3]);

		//printf("LHbone1x: %f, LHbone1y: %f, LHbone1z: %f \n",  leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[1](0),leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[1](1),leftSkeletonHand.leftSkeletonFinger[j].leftBoneEnd[1](2));
	}

	// Draw bones between joints
	for(int k=1; k<4; k++) {
		leftFingers.leftFingerLine[k]->m_pointA = leftJoints.leftJointSphere[k]->getLocalPos();
		leftFingers.leftFingerLine[k]->m_pointB = leftJoints.leftJointSphere[k+1]->getLocalPos();
	}
	for(int k=4; k<7; k++) {
		leftFingers.leftFingerLine[k]->m_pointA = leftJoints.leftJointSphere[k+2]->getLocalPos();
		leftFingers.leftFingerLine[k]->m_pointB = leftJoints.leftJointSphere[k+3]->getLocalPos();
	}
	for(int k=7; k<10; k++) {
		leftFingers.leftFingerLine[k]->m_pointA = leftJoints.leftJointSphere[k+4]->getLocalPos();
		leftFingers.leftFingerLine[k]->m_pointB = leftJoints.leftJointSphere[k+5]->getLocalPos();
	}
	for(int k=10; k<13; k++) {
		leftFingers.leftFingerLine[k]->m_pointA = leftJoints.leftJointSphere[k+6]->getLocalPos();
		leftFingers.leftFingerLine[k]->m_pointB = leftJoints.leftJointSphere[k+7]->getLocalPos();
	}
	for(int k=13; k<17; k++) {
		leftFingers.leftFingerLine[k]->m_pointA = leftJoints.leftJointSphere[k+7]->getLocalPos();
		leftFingers.leftFingerLine[k]->m_pointB = leftJoints.leftJointSphere[k+8]->getLocalPos();
	}
	leftFingers.leftFingerLine[17]->m_pointA = leftJoints.leftJointSphere[2]->getLocalPos();
	leftFingers.leftFingerLine[17]->m_pointB = leftJoints.leftJointSphere[6]->getLocalPos();
	leftFingers.leftFingerLine[18]->m_pointA = leftJoints.leftJointSphere[6]->getLocalPos();
	leftFingers.leftFingerLine[18]->m_pointB = leftJoints.leftJointSphere[11]->getLocalPos();
	leftFingers.leftFingerLine[19]->m_pointA = leftJoints.leftJointSphere[11]->getLocalPos();
	leftFingers.leftFingerLine[19]->m_pointB = leftJoints.leftJointSphere[16]->getLocalPos();
	leftFingers.leftFingerLine[20]->m_pointA = leftJoints.leftJointSphere[16]->getLocalPos();
	leftFingers.leftFingerLine[20]->m_pointB = leftJoints.leftJointSphere[21]->getLocalPos();
	leftFingers.leftFingerLine[21]->m_pointA = leftJoints.leftJointSphere[1]->getLocalPos();
	leftFingers.leftFingerLine[21]->m_pointB = leftJoints.leftJointSphere[5]->getLocalPos();
	leftFingers.leftFingerLine[22]->m_pointA = leftJoints.leftJointSphere[5]->getLocalPos();
	leftFingers.leftFingerLine[22]->m_pointB = leftJoints.leftJointSphere[10]->getLocalPos();
	leftFingers.leftFingerLine[23]->m_pointA = leftJoints.leftJointSphere[10]->getLocalPos();
	leftFingers.leftFingerLine[23]->m_pointB = leftJoints.leftJointSphere[15]->getLocalPos();
	leftFingers.leftFingerLine[24]->m_pointA = leftJoints.leftJointSphere[15]->getLocalPos();
	leftFingers.leftFingerLine[24]->m_pointB = leftJoints.leftJointSphere[20]->getLocalPos();


}

void drawRightHand(RightSkeletonHand rightSkeletonHand) {
	for(int j = 0; j < 5; j++) {
		rightJoints.rightJointSphere[5*j+0]->setLocalPos(rightSkeletonHand.rightSkeletonFinger[j].rightBoneStart[0]);
		rightJoints.rightJointSphere[5*j+1]->setLocalPos(rightSkeletonHand.rightSkeletonFinger[j].rightBoneEnd[0]);
		rightJoints.rightJointSphere[5*j+2]->setLocalPos(rightSkeletonHand.rightSkeletonFinger[j].rightBoneEnd[1]);
		rightJoints.rightJointSphere[5*j+3]->setLocalPos(rightSkeletonHand.rightSkeletonFinger[j].rightBoneEnd[2]);
		rightJoints.rightJointSphere[5*j+4]->setLocalPos(rightSkeletonHand.rightSkeletonFinger[j].rightBoneEnd[3]);
	}

	// Draw bones between joints
	for(int k=1; k<4; k++) {
		rightFingers.rightFingerLine[k]->m_pointA = rightJoints.rightJointSphere[k]->getLocalPos();
		rightFingers.rightFingerLine[k]->m_pointB = rightJoints.rightJointSphere[k+1]->getLocalPos();
	}
	for(int k=4; k<7; k++) {
		rightFingers.rightFingerLine[k]->m_pointA = rightJoints.rightJointSphere[k+2]->getLocalPos();
		rightFingers.rightFingerLine[k]->m_pointB = rightJoints.rightJointSphere[k+3]->getLocalPos();
	}
	for(int k=7; k<10; k++) {
		rightFingers.rightFingerLine[k]->m_pointA = rightJoints.rightJointSphere[k+4]->getLocalPos();
		rightFingers.rightFingerLine[k]->m_pointB = rightJoints.rightJointSphere[k+5]->getLocalPos();
	}
	for(int k=10; k<13; k++) {
		rightFingers.rightFingerLine[k]->m_pointA = rightJoints.rightJointSphere[k+6]->getLocalPos();
		rightFingers.rightFingerLine[k]->m_pointB = rightJoints.rightJointSphere[k+7]->getLocalPos();
	}
	for(int k=13; k<17; k++) {
		rightFingers.rightFingerLine[k]->m_pointA = rightJoints.rightJointSphere[k+7]->getLocalPos();
		rightFingers.rightFingerLine[k]->m_pointB = rightJoints.rightJointSphere[k+8]->getLocalPos();
	}
	rightFingers.rightFingerLine[17]->m_pointA = rightJoints.rightJointSphere[2]->getLocalPos();
	rightFingers.rightFingerLine[17]->m_pointB = rightJoints.rightJointSphere[6]->getLocalPos();
	rightFingers.rightFingerLine[18]->m_pointA = rightJoints.rightJointSphere[6]->getLocalPos();
	rightFingers.rightFingerLine[18]->m_pointB = rightJoints.rightJointSphere[11]->getLocalPos();
	rightFingers.rightFingerLine[19]->m_pointA = rightJoints.rightJointSphere[11]->getLocalPos();
	rightFingers.rightFingerLine[19]->m_pointB = rightJoints.rightJointSphere[16]->getLocalPos();
	rightFingers.rightFingerLine[20]->m_pointA = rightJoints.rightJointSphere[16]->getLocalPos();
	rightFingers.rightFingerLine[20]->m_pointB = rightJoints.rightJointSphere[21]->getLocalPos();
	rightFingers.rightFingerLine[21]->m_pointA = rightJoints.rightJointSphere[1]->getLocalPos();
	rightFingers.rightFingerLine[21]->m_pointB = rightJoints.rightJointSphere[5]->getLocalPos();
	rightFingers.rightFingerLine[22]->m_pointA = rightJoints.rightJointSphere[5]->getLocalPos();
	rightFingers.rightFingerLine[22]->m_pointB = rightJoints.rightJointSphere[10]->getLocalPos();
	rightFingers.rightFingerLine[23]->m_pointA = rightJoints.rightJointSphere[10]->getLocalPos();
	rightFingers.rightFingerLine[23]->m_pointB = rightJoints.rightJointSphere[15]->getLocalPos();
	rightFingers.rightFingerLine[24]->m_pointA = rightJoints.rightJointSphere[15]->getLocalPos();
	rightFingers.rightFingerLine[24]->m_pointB = rightJoints.rightJointSphere[20]->getLocalPos();



}

//------------------------------------------------------------------------------
// Response functions
//------------------------------------------------------------------------------
void respondToSwipe(void) {
	swipeGesture = SwipeGesture(pGesture);
	swipeDirection = swipeGesture.direction();
	//printf("Swipe dir x: %f, Swipe dir y: %f, Swipe dir z: %f \n",swipeDirection.x,swipeDirection.y,swipeDirection.z);
	if(abs(swipeDirection.x) > abs(swipeDirection.z)) {			// HORIZONTAL SWIPE
		horizontalSwipe = true;
		int dir;
		if(swipeDirection.x>0) {
			dir = -1;			// Rotate left
		} else {
			dir = 1;			// Rotate right
		}
		rotateLeftRight(dir, originSphere);
	} else if(abs(swipeDirection.z) > abs(swipeDirection.x)) {	// VERTICAL SWIPE
		verticalSwipe = true;
		int dir;
		if(swipeDirection.z>0) {
			dir = -1;			// Rotate down
		} else {
			dir = 1;			// Rotate up
		}
		rotateUpDown(dir, originSphere);
	}
}
#endif

void rotateLeftRight(int dir, cShapeSphere* objectToRotate) {
	if(dir==1) {	
		objectToRotate->rotateAboutGlobalAxisDeg(0,-1,0,10);
		commandName = "Rotate Right";
	} else {
		objectToRotate->rotateAboutGlobalAxisDeg(0,-1,0,-10);		// Rotate left
		commandName = "Rotate Left";
	}
}

void rotateUpDown(int dir, cShapeSphere* objectToRotate) {
	if(dir==1) {
		objectToRotate->rotateAboutGlobalAxisDeg(-1,0,0,20);		// Rotate Up
		commandName = "Rotate Up";
	} else {
		objectToRotate->rotateAboutGlobalAxisDeg(-1,0,0,-20);		// Rotate Down
		commandName = "Rotate Down";
	}
}

void rotateCwCcw(int dir, cShapeSphere* objectToRotate) {
	if(dir==1) {
		objectToRotate->rotateAboutGlobalAxisDeg(0,0,1,20);			// Clockwise ('c')
		commandName = "Rotate Clockwise";
	} else {
		objectToRotate->rotateAboutGlobalAxisDeg(0,0,1,-20);		// Counterclockwise ('w')
		commandName = "Rotate Counter clockwise";
	}
}

void rotateAll(void) {
	originSphere->rotateAboutGlobalAxisDeg(0,0,1,20);
	tubeStartSphere->rotateAboutGlobalAxisDeg(0,0,1,20);
	commandName = "Rotate All";
}

void setTransparent(cMultiMesh* transparentMesh) {
	transparentMesh->setTransparencyLevel(0.1,true,false);
}

#ifdef USE_LEAP
void checkGrip(void) {
	int thumbInd = 4;
	int indexInd = 9;
	cVector3d thumbDist = (rightSkeletonHand.rightSkeletonFinger[thumbInd].rightBoneEnd[3]); 

	cMaterial tipColor;
	tipColor.setPinkHot();
	// Thumb tip
	rightJoints.rightJointSphere[thumbInd]->setMaterial(tipColor);
	// Index finger tip
	rightJoints.rightJointSphere[indexInd]->setMaterial(tipColor);


	Vector boxCenter = interactionBox.center();
	cVector3d boxCent = cVector3d(boxCenter.x,boxCenter.y,boxCenter.z);
	posSphere->setEnabled(true);
	posSphere->setLocalPos(boxCent);

}
#endif

void zoom(float deltaOffset) {
	oculusOffset = oculusOffset + deltaOffset;
	rotNum = 0;
} 

//------------------------------------------------------------------------------
// Thread to compute kinematics
//------------------------------------------------------------------------------
void runKinematics(void) {
	waitLabel->setEnabled(true);
	kinematics(set);
	
	setInvalidated = true;
	waitLabel->setEnabled(false);
}

//------------------------------------------------------------------------------
// Thread to communicate with Arduino
//------------------------------------------------------------------------------
void runSerialComm(void) {	
	while(SP->IsConnected())
	{
		// Read incoming data from Arduino
		readResult = SP->ReadData(incomingData,dataLength);
		if(readResult!=-1) {	
			mode = std::stoi (incomingData,nullptr);
			printf("%i \n",mode);
			
			if(mode==1) {
				modeName = "Master clutch";
				printf("Clutch \n");
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->clutchPressed();
				
			} else if(mode==0) {
				printf("Clutch released \n");
				designModeOn = true;
				modeName = "Design Mode";
				cPhantomDeviceWithClutch *hDevice = (cPhantomDeviceWithClutch *)hapticDevice.get();
				hDevice->releaseClutch();
				
			} else if(mode==2) {
				if(!cameraModeOn) {
					cameraModeOn = true; 
					designModeOn = false;
					simulationModeOn = false;
					rotateModeOn = false;
					modeName = "Camera Mode";

					rotateSphere->setEnabled(true);
					rotateSphere->setTransparencyLevel(0.3);
					rotateSphere->setRadius(0.12);
					rotateSphere->m_material->setStiffness(0.8 * maxStiffness);		// set parameters related to effect
					rotateSphere->setHapticEnabled(true);
				} else {
					cameraModeOn = false;
					designModeOn = true;
					modeName = "Design Mode";

					rotateSphere->setEnabled(false);
					rotateSphere->setHapticEnabled(false);
				}
				alphaLabel->setEnabled(false);
			} else if(mode==3) {
				if(cameraModeOn) {
					commandName = "Zoom in";
					float deltaOffset = -0.05;
					zoom(deltaOffset);
				} else if(!increaseLength) { 
					printf("setting increase length = true\n");
					increaseLength = true;
				}
				alphaLabel->setEnabled(false);
			} else if(mode==4) {
				if(cameraModeOn) {
					commandName = "Zoom out";
					float deltaOffset = 0.05;
					zoom(deltaOffset);
				} else if(!decreaseLength) { 
					printf("setting decrease length = true\n");
					decreaseLength = true;
				}
				alphaLabel->setEnabled(false);
			} else if(mode==5) {
				if(!rotateModeOn) {
					commandName = "";
					modeName = "Design Mode 2";
					rotateModeOn = true;
					designModeOn = false;
					cameraModeOn = false;
					simulationModeOn = false;
					warningLabel->setEnabled(false);
					//setShowMaterialLabels();
				} else {
					commandName = "";
					rotateModeOn = false;
					designModeOn = true;
					modeName = "Design Mode";
					alphaLabel->setEnabled(false);
					setShowMaterialLabels();
				}
			} else if(mode==6) {
				if(!simulationModeOn) {
					commandName = "";
					designModeOn = false;
					cameraModeOn = false;
					rotateModeOn = false;
					simulationModeOn = true;
					
				} else {
					commandName = "";
					simulationModeOn = false;
					designModeOn = true;
					modeName = "DesignMode";
					calculated = false;
					redrawTubes = true;
				}
				alphaLabel->setEnabled(false);
			}
		}
		lastMode = mode;
	}
}

//------------------------------------------------------------------------------
// Thread to collect data
//------------------------------------------------------------------------------
void collectData(void) {
	while(simulationRunning) {
		dataStruct currentData;										// instant of data structure
		currentData.time = clock.getCurrentTimeSeconds();			// get current time
		currentData.devicePos = tool->getDeviceGlobalPos();			// get position of the device
		currentData.deviceRot = tool->getDeviceGlobalRot();			// get rotation of the device
		currentData.CTRPos = tubeStartSphere->getGlobalPos();		// get position of CTR base
		currentData.CTRRot = tubeStartSphere->getGlobalRot();		// get rotation of CTR
		currentData.modelPos = originSphere->getGlobalPos();		// get position of model
		currentData.modelRot = originSphere->getGlobalRot();		// get rotation of model
		currentData.pedalPressed = mode;							// get which pedal was pressed
		currentData.button0Pressed = tool->getUserSwitch(0);		// get whether button0 is pressed or not
		currentData.button1Pressed = tool->getUserSwitch(1);		// get whether button1 is pressed or not

		currentData.OD0 = set.m_tubes[0].OD;
		currentData.OD1 = set.m_tubes[1].OD;
		currentData.OD2 = set.m_tubes[2].OD;
		currentData.ID0 = set.m_tubes[0].ID;
		currentData.ID1 = set.m_tubes[1].ID;
		currentData.ID2 = set.m_tubes[2].ID;
		currentData.kappa0 = set.m_tubes[0].kappa;
		currentData.kappa1 = set.m_tubes[1].kappa;
		currentData.kappa2 = set.m_tubes[2].kappa;
		currentData.alpha0 = set.m_tubes[0].alpha;
		currentData.alpha1 = set.m_tubes[1].alpha;
		currentData.alpha2 = set.m_tubes[2].alpha;
		currentData.Beta0 = set.m_tubes[0].Beta;
		currentData.Beta1 = set.m_tubes[1].Beta;
		currentData.Beta2 = set.m_tubes[2].Beta;
		currentData.Lc0 = set.m_tubes[0].Lc;
		currentData.Lc1 = set.m_tubes[1].Lc;
		currentData.Lc2 = set.m_tubes[2].Lc;
		currentData.Ls0 = set.m_tubes[0].Ls;
		currentData.Ls1 = set.m_tubes[1].Ls;
		currentData.Ls2 = set.m_tubes[2].Ls;
		currentData.materialNum0 = set.m_tubes[0].materialNum;
		currentData.materialNum1 = set.m_tubes[1].materialNum;
		currentData.materialNum2 = set.m_tubes[2].materialNum;


		//// Add data from current time step to vector of all data
		data.push_back(currentData);

		// Delay loop
		Sleep(20);			// collect data at 100Hz
	}
}

//------------------------------------------------------------------------------
// Thread to run main haptics loop
//------------------------------------------------------------------------------
void updateHaptics(void)
{
    // initialize frequency counter
    frequencyCounter.reset();
	int waitCount = 0;
	bool increasePointNum = false;
	float origContactVecMag = 0.01;
	float x = 0;
	cVector3d forceDir = cVector3d(0,0,0);

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;
	
	// ***** testing ******
	cVector3d devicePos;
	cVector3d devicePosCamFrame;
	cTransform deviceTransform;
	cTransform cameraTransform;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // READ HAPTIC DEVICE
        /////////////////////////////////////////////////////////////////////

        // read user-switch status (button 0)
		int button0, button1;
		button0 = tool->getUserSwitch(0);
		button1 = tool->getUserSwitch(1);

        /////////////////////////////////////////////////////////////////////
        // UPDATE 3D CURSOR MODEL
        /////////////////////////////////////////////////////////////////////
		cursor->setLocalPos(tool->getDeviceLocalPos());
		cursor->setLocalRot(tool->getDeviceLocalRot());

		cVector3d f = tool->m_lastComputedGlobalForce;
		float fMag = sqrt(f(0)*f(0) + f(1)*f(1) + f(2)*f(2));
		
		//check if we need to redraw tubes
		if(setInvalidated && set.isValidSet) {
			drawTubes(set);
		}

		//------------------------------------------------------------------------------
		// RESPOND TO BUTTON PRESSES
		//------------------------------------------------------------------------------

		//--------------------------- Initializing design ------------------------------
		//------------ button0 (place points) button1 (calc init design) ---------------
		//------------------------------------------------------------------------------
		if (designModeOn && !CTRInitialized && !readyToDrawInitCTR) {
			hapticsOn = false;							// turn off haptics so that intial design can be done in place (within the anatomy)


			if(pointNum==0) {
				// set initial tangent vector
				tanLine->m_pointB.set(0,0,-0.08);											// set endpoint of line

				cTransform cursorToWorldTrans = cursor->getGlobalTransform();				// Transformation from cursor frame to world frame
				cVector3d localEndPoint = tanLine->m_pointB;								// endpoint of line in cursor frame				
				cursorToWorldTrans.mulr(localEndPoint,globalTanVec);						// Transform local tan vector to world coord

				cVector3d globalStartPoint = cursor->getGlobalPos();
				globalTanLine->m_pointA.set(globalStartPoint(0),globalStartPoint(1),globalStartPoint(2));			// set start start point of tan vec in world coord
				globalTanLine->m_pointB.set(globalTanVec(0),globalTanVec(1),globalTanVec(2));						// set end point of tan vec in world coord	
				globalTanLine->setEnabled(true);

				v2_temp = globalTanVec - globalStartPoint;
			}
			//------------------------------------------------------------------------------
			//------------------------- PLACE INITIAL POINTS -------------------------------
			//------------------------------------------------------------------------------
			if(button0==1) {
				increasePointNum = true;

				cVector3d desPosLocal;
				cVector3d desPos = tool->getDeviceGlobalPos();
				cTransform originSphereToWorldTrans = originSphere->getGlobalTransform();
				originSphereToWorldTrans.invert();
				originSphereToWorldTrans.mulr(desPos,desPosLocal);
				pointSphere[pointNum]->setLocalPos(desPosLocal);
				pointSphere[pointNum]->setEnabled(true);

				if(pointNum==0) {
					v2_init = v2_temp;
				}
			//------------------------------------------------------------------------------
			//-------------------------- CALCULATE INIT CTR --------------------------------
			//------------------------------------------------------------------------------
			} else if(button1==1) {

				vector<cVector3d> *circParam;
				circParam = new vector<cVector3d>[pointNum+1];
				for(int i=0; i<pointNum; i++) {
					circParam->push_back(pointSphere[i]->getLocalPos());
				}
				circParam->push_back(v2_init);								// trying to use user defined tangent vector
				computeTubeParam(set, circParam);
				readyToDrawInitCTR = true;
				circParam->clear();			//try this????
			} else if(increasePointNum) {
				increasePointNum = false;
				pointNum = pointNum + 1;
			} else if(readyToDrawInitCTR) {
				
			}
			
		//------------------------------------------------------------------------------	
		//-------------------- In Camera Mode and Button0 Pressed ----------------------
		//-------------------------------- Rotate View ---------------------------------
		} else if ((button0==1) && (cameraModeOn)) {				
			// Change color of tool sphere										
			hapticsOn = true;		//testing putting this in
			if((fMag!=0) && (!gripped)) {																		// If in contact for the first time
				lastContactPoint = tool->getDeviceGlobalPos();				
				lastContactVec = lastContactPoint - (originSphere->getGlobalPos());
				// Change cursor color
				cursor->setEnabled(true);
				cursor->m_material->setOrangeCoral();
				tool->setShowEnabled(false);

				origContactVecMag = sqrt(lastContactVec(0)*lastContactVec(0) + lastContactVec(1)*lastContactVec(1) + lastContactVec(2)*lastContactVec(2));
				gripped = true;
			} else if(gripped) {																				// If "gripped" onto model
				cVector3d contactPoint = tool->getDeviceGlobalPos();
				cVector3d contactVec = contactPoint - (originSphere->getGlobalPos());
				lastContactVec.crossr(contactVec,normVec);														// Calculate the normal to current contact vector and last contact vector
				float contactVecMag = sqrt(contactVec(0)*contactVec(0) + contactVec(1)*contactVec(1) + contactVec(2)*contactVec(2));
				float lastContactVecMag = sqrt(lastContactVec(0)*lastContactVec(0) + lastContactVec(1)*lastContactVec(1) + lastContactVec(2)*lastContactVec(2));
				theta = acos((lastContactVec.dot(contactVec))/(contactVecMag*lastContactVecMag));				// Calculate the angle between current and last contact vectors
				
				if(tool->isInContact(rotateSphere)) {
					if(theta>0.001) {
						originSphere->rotateAboutGlobalAxisRad(normVec(0),normVec(1),normVec(2),theta);				// Rotate model based on normal vector and angle
						rotNum = 0;
					}

					// Set current contact point and vector to the last ones
					lastContactPoint = contactPoint;
					lastContactVec = contactVec;

					// Set command name for label
					commandName = "Rotate camera";
				} else {
					gripped = false;
					cursor->setEnabled(false);
					tool->setShowEnabled(true);
				}
				
			} 
		//------------------------------------------------------------------------------	
		//-------------------- In Camera Mode and Button1 Pressed ----------------------
		//------------------------------ Default views ---------------------------------
		} else if ((button1==1) && (cameraModeOn)) {
			if(!rotating) {
				rotating = true;
				cMatrix3d currentRot = originSphere->getLocalRot();
				// Determine how to rotate view
				if(!currentRot.equals(lastRot)) {
					if(currentRot.equals(defaultRotation) || rotNum!=0) {
						rotNum = rotNum + 1;
						if(rotNum<5) {
							originSphere->rotateAboutGlobalAxisDeg(0,-1,0,90); 
						} else if(rotNum<7) {
							originSphere->rotateAboutGlobalAxisDeg(1,0,0,90);
						} else {
							originSphere->rotateAboutGlobalAxisDeg(1,0,0,90);
							rotNum = 0;
						}
					} else {
						showDefaultView(defaultRotation, defaultOculusOffset);
						rotNum = 0;
					}
				}
				lastRot = currentRot;
			} 
		} else if((button1==0) && (cameraModeOn) && rotating) {			// for debouncing!
			rotating = false;

		//------------------------------------------------------------------------------
		//-------------------- In Design Mode and Button0 Pressed ----------------------
		//------------------------- Translate insertion point --------------------------
        } else if((button0==1) && (designModeOn) && CTRInitialized) {											// If in design mode and button0 is pressed AND CTR has been initialized
			if(proximalSphere->m_interactionInside) {															// If in contact with proximal sphere (for translation)
				cursor->setEnabled(true);
				cursor->m_material->setYellowGold();
				tool->setShowEnabled(false);

				hapticsOn = false;

				cVector3d translateVec = tool->getDeviceGlobalPos() - tubeStartSphere->getGlobalPos();
				//tubeStartSphere->translate(translateVec);			// can put this back in if things stop working...

				// **** testing to see if it fixes problem with translation not working after camera mode default views *****
				cVector3d desPosLocal;
				cVector3d desPos = tool->getDeviceGlobalPos();
				cTransform originSphereToWorldTrans = originSphere->getGlobalTransform();
				originSphereToWorldTrans.invert();
				originSphereToWorldTrans.mulr(desPos,desPosLocal);
				tubeStartSphere->setLocalPos(desPosLocal);
				
				commandName = "Translate insertion point";
				waitCount = 0;
			//------------------------------------------------------------------------------
			//------------------------- Change tube curvature ------------------------------
			//------------------ Button 0 and in contact with lines ------------------------
			} else {																							// If in contact with lines to adjust curvature
				for(int k=nTubes-1; k>=0; k--) {
					if((tool->isInContact(centerLine[k])) && (!lineGripped[k])) {
						kappaChange = true;
						cursor->setEnabled(true);
						cursor->setMaterial(tubeColor[k]);
						tool->setShowEnabled(false);
						lineGripped[k] = true;

						// Define tip and base of line to be changed (in LOCAL coord)
						cVector3d tipPoint = centerLine[k]->m_pointA;			// endpoint of line in cursor frame
						cVector3d basePoint = centerLine[k]->m_pointB;			// base of line in cursor frame
						cVector3d globalTipPoint;
						cVector3d globalBasePoint;
						// Calculate tip and base pos in GLOBAL coord
						cTransform tubeStartSphereToWorldTrans = tubeStartSphere->getGlobalTransform();		// Transform from tubeStartSphere to world frame
						tubeStartSphereToWorldTrans.mulr(tipPoint,globalTipPoint);							// global pos of tip of line
						tubeStartSphereToWorldTrans.mulr(basePoint,globalBasePoint);						// global pos of base of line						
						// Calculate normalized vector in direction of line
						cVector3d lineVec = cSub(globalTipPoint,globalBasePoint);
						unitVecOrig = cNormalize(lineVec);

					} else if (lineGripped[k]) {
						// Define tip and base of line to be changed (in LOCAL coord)
						cVector3d tipPoint = centerLine[k]->m_pointA;			// endpoint of line in cursor frame
						cVector3d basePoint = centerLine[k]->m_pointB;			// base of line in cursor frame
						cVector3d globalTipPoint;
						cVector3d globalBasePoint;

						// Calculate tip and base pos in GLOBAL coord
						cTransform tubeStartSphereToWorldTrans = tubeStartSphere->getGlobalTransform();		// Transform from tubeStartSphere to world frame
						tubeStartSphereToWorldTrans.mulr(tipPoint,globalTipPoint);							// global pos of tip of line
						tubeStartSphereToWorldTrans.mulr(basePoint,globalBasePoint);						// global pos of base of line
						
						// Calculate normalized vector in direction of line
						cVector3d lineVec = cSub(globalTipPoint,globalBasePoint);
						cVector3d unitVec = cNormalize(lineVec);

						// Calculate new point for tip of line in GLOBAL coord
						cVector3d toolPos = tool->getDeviceGlobalPos();
						cVector3d toolVec = cSub(toolPos,globalBasePoint);				// Calculate tool position vector from base point
						float deltaH = cDot(toolVec,unitVec);							// Length of new tip point from base point
						cVector3d newPoint = globalBasePoint + (deltaH*unitVec);		

						// Calculate new point for tip of line in LOCAL coord
						cVector3d localNewPoint;
						tubeStartSphereToWorldTrans.invert();
						tubeStartSphereToWorldTrans.mulr(newPoint,localNewPoint);

						// Compute desired force to apply to tool to keep it along line (and make sure it is in CAMERA LOCAL COORD)
						cVector3d tempForceDir = cSub(newPoint,toolPos);
						tempForceDir = 40*tempForceDir;
						cTransform camToWorldTrans = camera->getGlobalTransform();
						camToWorldTrans.invert();
						camToWorldTrans.mulr(tempForceDir,forceDir);

						// Set tip point to new tip point
						centerLine[k]->m_pointA.set(localNewPoint(0),localNewPoint(1),localNewPoint(2));

						dotProd = unitVecOrig.dot(unitVec);

						// Compute change in radius of curvature
						a3 = localNewPoint;
						a1 = cVector3d(set.positionStored.x[tubeStartIndReduced[k]],											
									   set.positionStored.y[tubeStartIndReduced[k]], 
									   set.positionStored.z[tubeStartIndReduced[k]]);
						if(k>0) {
							a2 = cVector3d(set.positionStored.x[tubeStartIndReduced[k-1]-1],											
									   set.positionStored.y[tubeStartIndReduced[k-1]-1], 
									   set.positionStored.z[tubeStartIndReduced[k-1]-1]);
						} else {
							a2 = cVector3d(set.positionStored.x[set.positionStored.x.size()-1],											
									   set.positionStored.y[set.positionStored.y.size()-1], 
									   set.positionStored.z[set.positionStored.z.size()-1]);
						}
						a4 = cVector3d(set.positionStored.x[tubeMidIndReduced[k]],											
									   set.positionStored.y[tubeMidIndReduced[k]], 
									   set.positionStored.z[tubeMidIndReduced[k]]);

						// Set command label
						commandName = "Change curvature";

						//printf("dot prod: %f \n", unitVecOrig.dot(unitVec));
						//printf("unit vec: %f %f %f \n", unitVec(0),unitVec(1),unitVec(2));
					}
				}
			}
		//------------------------------------------------------------------------------
		//-------------------- In Design Mode and Button1 Pressed ----------------------
		//------------------------- Change CTR orientation -----------------------------
		} else if((button1==1) && (designModeOn) && CTRInitialized) {
			if((proximalSphere->m_interactionInside) && (!gripped) && (!lengthChange)) {											// If in contact with proximal sphere
				cursor->setEnabled(true);
				cursor->m_material->setRedFireBrick();
				tool->setShowEnabled(false);
				lastContactPoint = tool->getInteractionPoint(0)->getGlobalPosProxy();
				lastContactVec = lastContactPoint - (tubeStartSphere->getGlobalPos());
				gripped = true;

				// testing for rotating about vector from tip to base by using pen rotation only
				deviceRot = tool->getDeviceGlobalRot();
				cVector3d axis; 
				deviceRot.toAxisAngle(axis, lastAngle);

			} else if(gripped) {  // rotate about vector from tip to base by using pen rotation only

				cVector3d contactPoint = tool->getDeviceGlobalPos();
				cVector3d contactVec = contactPoint - (tubeStartSphere->getGlobalPos());

				// ****** Calculate force to apply to make tool stay at tubeStartSphere position ******
				kappaChange = true;
				cVector3d tempForceDir = -30*contactVec;
				cTransform camToWorldTrans = camera->getGlobalTransform();
				camToWorldTrans.invert();
				camToWorldTrans.mulr(tempForceDir,forceDir);
				// ************************************************************************************

				deviceRot = tool->getDeviceGlobalRot();
				cVector3d axis; 
				deviceRot.toAxisAngle(axis, angle);
				angleChange = lastAngle - angle;
				printf("delta angle: %f \n",angleChange);

				cVector3d rotAxis = cSub(distalSphere->getGlobalPos(),tubeStartSphere->getGlobalPos());


				cMatrix3d originSphereToWorldRot = originSphere->getGlobalRot();
				cVector3d newRotAxis;
				originSphereToWorldRot.invert();
				originSphereToWorldRot.mulr(rotAxis,newRotAxis);

				tubeStartSphere->rotateAboutGlobalAxisDeg(newRotAxis(0),newRotAxis(1),newRotAxis(2),-0.5*angleChange);
				//tubeStartSphere->rotateAboutGlobalAxisDeg(rotAxis(0),rotAxis(1),rotAxis(2),-0.5*angleChange);
				//lastAngle = angle;


			} else if ((distalSphere->m_interactionInside) && !distalGripped && (!lengthChange)) {
				cursor->setEnabled(true);
				cursor->m_material->setRedFireBrick();
				tool->setShowEnabled(false);
				//lastContactPoint = tool->getInteractionPoint(0)->getGlobalPosProxy();
				lastContactPoint = tool->getDeviceGlobalPos();
				lastContactVec = lastContactPoint - (tubeStartSphere->getGlobalPos());
				distalGripped = true;

				/*pA = cSub(tubeStartSphere->getGlobalPos(),distalSphere->getGlobalPos());
				pA = cNormalize(pA);*/

				hapticsOn = false;

			} else if(distalGripped) {
				cVector3d contactPoint = tool->getDeviceGlobalPos();
				cVector3d contactVec = contactPoint - (tubeStartSphere->getGlobalPos());
				lastContactVec.crossr(contactVec,normVec);														// Calculate the normal to current contact vector and last contact vector
				float contactVecMag = sqrt(contactVec(0)*contactVec(0) + contactVec(1)*contactVec(1) + contactVec(2)*contactVec(2));
				float lastContactVecMag = sqrt(lastContactVec(0)*lastContactVec(0) + lastContactVec(1)*lastContactVec(1) + lastContactVec(2)*lastContactVec(2));
				theta = acos((lastContactVec.dot(contactVec))/(contactVecMag*lastContactVecMag));				// Calculate the angle between current and last contact vectors

				// Change rotation axis to be in orignSphere coord frame
				cVector3d normVecLocal;
				cMatrix3d originSphereToWorldRot = originSphere->getGlobalRot();
				cVector3d newNormVec;
				originSphereToWorldRot.invert();
				originSphereToWorldRot.mulr(normVec,newNormVec);


				if(theta>0.001) {
					tubeStartSphere->rotateAboutGlobalAxisRad(newNormVec(0),newNormVec(1),newNormVec(2),theta);
					//tubeStartSphere->rotateAboutGlobalAxisRad(normVec(0),normVec(1),normVec(2),theta);				// Rotate model based on normal vector and angle
					rotNum = 0;
				}

				// Set current contact point and vector to the last ones
				lastContactPoint = contactPoint;
				lastContactVec = contactVec;

				// ****** Calculate force to apply to make tool stay at distal tip position ******
				cVector3d desPos = contactPoint - (tube.tubeSphere[tubeEndIndReduced[0]]->getGlobalPos());
				kappaChange = true;
				cVector3d tempForceDir = -20*desPos;
				cTransform camToWorldTrans = camera->getGlobalTransform();
				camToWorldTrans.invert();
				camToWorldTrans.mulr(tempForceDir,forceDir);

			//------------------------------------------------------------------------------
			//-------------------- In Design Mode and Button1 Pressed ----------------------
			//-------------------------- Change Tube Length --------------------------------
			} else { 
				for(int k=nTubes-1; k>=0; k--) {
					if((tool->isInContact(centerLine[k])) && (!lineGrippedForLengthChange[k])) {		// **** testing new way to change tube length
						if(!lengthChange) {
							kappaChange = true;
							cursor->setEnabled(true);
							cursor->setMaterial(tubeColor[k]);
							tool->setShowEnabled(false);
							lineGrippedForLengthChange[k] = true;
						}
					} else if (lineGrippedForLengthChange[k]) {
						// determine force to apply to keep cursor at center of curve
						cVector3d toolPos = tool->getDeviceGlobalPos();										// tool pos in global coord

						// try forcing cursor to end point of tube
						//cVector3d posDes = tube.tubeSphere[tubeEndInd[k]]->getGlobalPos();
						cVector3d posDes = tube.tubeSphere[tubeEndIndReduced[k]]->getGlobalPos();
						cVector3d tempForceDir = cSub(posDes,toolPos);
						double tempForce = tempForceDir.length();
						
						//printf("dist: %f \n", tempForce);
						if(tempForce < 0.006) {
							lengthChange = true;
						}
						
						tempForceDir = 50*tempForceDir;
						cTransform camToWorldTrans = camera->getGlobalTransform();
						camToWorldTrans.invert();
						camToWorldTrans.mulr(tempForceDir,forceDir);


						if(lengthChange) {
							// determine how to change color of spheres
							double toolDistToMid = (cSub(tube.tubeSphere[tubeMidIndReduced[k]]->getGlobalPos(),toolPos)).length();
							double endDistToMid = (cSub(tube.tubeSphere[tubeMidIndReduced[k]]->getGlobalPos(),posDes)).length();
							double approxNumSpheres = tempForce/hsize/5;
							int spheresToChange;
							
							if(toolDistToMid<endDistToMid) { // shortening tube length
								sphereInd = tubeEndIndReduced[k] - approxNumSpheres;			// to keep track of index of endpoint
								deltaLength = -approxNumSpheres*5*hsize;
								for(int i=tubeStartIndReduced[k]; i<sphereInd; i++) {
									if(k!=0) {	// case for all tubes except innermost tube
										tube.tubeSphere[i]->setMaterial(tubeColor[k]);				// spheres from start of tube to endInd
									} else {	// case for innermost tube
										tube.tubeSphere[i]->setShowEnabled(true);					// need to ERASE spheres from endInd to the end of the tube
									}
								}
								for(int i=sphereInd; i<tubeEndIndReduced[k]+1; i++) {
									if(k!=0) {	// case for all tubes except innermost tube
										tube.tubeSphere[i]->setMaterial(tubeColor[k-1]);		// spheres from endInd to end of tube, turn next tube color
									} else {	// case for innermost tube
										//tube.tubeSphere[i]->setEnabled(false);					// need to ERASE spheres from endInd to the end of the tube
										tube.tubeSphere[i]->setShowEnabled(false);
									}
								}
								
							} else {			// lengthening tube 
								sphereInd = tubeEndIndReduced[k] + approxNumSpheres;			// to keep track of index of endpoint
								deltaLength = approxNumSpheres*5*hsize;
								if(k!=0) {		// case for all tubes except innermost tube
									for(int i=tubeStartIndReduced[k]; i<sphereInd; i++) {
										tube.tubeSphere[i]->setMaterial(tubeColor[k]);			// spheres from start of tube to endInd
									}			
									if(sphereInd < lastSphereInd) {
										for(int i=sphereInd; i<lastSphereInd; i++) {
											tube.tubeSphere[i]->setMaterial(tubeColor[k-1]);	// spheres from endInd to lastEndInd, turn back to original color
										}
									}
								} else {		// case for innermost tube
									// calculate approx tangent to curve at that point
									//cVector3d approxTan = cSub(tube.tubeSphere[tubeEndIndReduced[k]-10]->getLocalPos(),tube.tubeSphere[tubeEndIndReduced[k]]->getLocalPos());
									//approxTan = cNormalize(approxTan);
									//float dist = hsize*5;
									//int j=0;

									//for(int i=tubeEndIndReduced[k]+1; i<sphereInd; i++) {		// spheres beyond end of tube
									//	j=j+1;
									//	tube.tubeSphere[i] = new cShapeSphere((set.m_tubes[k].OD)/2);			// create sphere with radius equal to radius of tube k
									//	tubeStartSphere->addChild(tube.tubeSphere[i]);							// add new sphere as child to tube start sphere
									//	cVector3d newPos = cAdd(tube.tubeSphere[tubeEndIndReduced[k]]->getLocalPos(), dist*j*approxTan);
									//	tube.tubeSphere[i]->setLocalPos(newPos(0),newPos(1),newPos(2));
									//	tube.tubeSphere[i]->setMaterial(tubeColor[k]);
									//	tube.tubeSphere[i]->setEnabled(true);
									//}

								}

								lastSphereInd = sphereInd;
							}
						}

					}
				}

			}

		//------------------------------------------------------------------------------
		//-------------------- In Rotate Mode and Button1 Pressed ----------------------
		//------------------------- Change alpha of tubes ------------------------------
		} else if((button1==1) && rotateModeOn) {				
			if(!readyToChangeMaterial && !changeMaterial) {
				for(int ind=0; ind<set.sStored.size(); ind++) {
					if((tool->isInContact(tube.tubeSphere[ind])) && (!grippedToRot)) {
						grippedToRot = true;	
						grippedToRotInd = ind;

						deviceRot = tool->getDeviceGlobalRot();
						cVector3d axis; 
						deviceRot.toAxisAngle(axis, lastAngle);

						cursor->setEnabled(true);
						cursor->m_material->setRedFireBrick();
						tool->setShowEnabled(false);

						break;
					} else if (grippedToRot) {
						deviceRot = tool->getDeviceGlobalRot();
						cVector3d axis; 
						deviceRot.toAxisAngle(axis, angle);
						angleChange = lastAngle - angle;
						//printf("delta angle: %f \n",angleChange);
						alphaValue = std::to_string(angleChange);
						alphaLabel->setEnabled(true);
						warningLabel->setEnabled(false);
					}
				}
			} else {
				changeMaterial = true;
				readyToChangeMaterial = false;
				printf("updating kinematics based on material change \n");
			}
		}  else if((button0==1) && rotateModeOn) {
			for(int ind=0; ind<set.sStored.size(); ind++) {
				if((tool->isInContact(tube.tubeSphere[ind])) && (!readyToChangeMaterial)) {
					readyToChangeMaterial = true;	

					// Determine tube number
					if(ind>tubeStartIndReduced[0]) {					// for innermost tube
						readyToChangeMaterialInd = 0;		
					} else {									// for all other tubes
						for(int j=nTubes-1; j>0; j--) {
							if((ind>tubeStartIndReduced[j]) && (ind<tubeStartIndReduced[j-1])) {
								readyToChangeMaterialInd = j;
							}
						} 
					} 

					cursor->setEnabled(true);
					cursor->setMaterial(tubeColor[readyToChangeMaterialInd]);;
					tool->setShowEnabled(false);

					// start debounce timer
					debounceClock.reset();
					debounceClock.start();

					updateTubeMaterial(set,readyToChangeMaterialInd,set.m_tubes[readyToChangeMaterialInd].materialNum);
					setShowMaterialLabels();
					break;
				} else if(readyToChangeMaterial) {
					double currTime = debounceClock.getCurrentTimeSeconds();
					if(currTime>DEBOUNCE_TIME) {
						int currMaterialNum = set.m_tubes[readyToChangeMaterialInd].materialNum;
						int nextMaterialNum;
						if(currMaterialNum==2) {
							nextMaterialNum = 0;
						} else {
							nextMaterialNum = currMaterialNum + 1;
						}
						set.m_tubes[readyToChangeMaterialInd].materialNum = nextMaterialNum;
						updateTubeMaterial(set,readyToChangeMaterialInd,nextMaterialNum);

						cursor->setEnabled(true);
						cursor->setMaterial(tubeColor[readyToChangeMaterialInd]);;
						tool->setShowEnabled(false);
						
						// start debounce timer
						debounceClock.reset();
						debounceClock.start();
					}
				}
			}

		//------------------------------------------------------------------------------
		//------------------ In Simulation Mode and Button1 Pressed --------------------
		//-------------------------- Simulate CTR moving -------------------------------			
		} else if((button0==1) && simulationModeOn) {	
			currSetDrawing = 0;
			readyToDraw = true;

		//------------------------------------------------------------------------------
		//---------------------------- APPLY ALL CHANGES -------------------------------
		//------------------------------------------------------------------------------

        } else {
			cursor->setEnabled(false);
			tool->setShowEnabled(true);

			gripped = false;
			distalGripped = false;
			//hapticsOn = true;

			if(readyToDrawInitCTR) {
				drawInitCTR(set);
			}
			

			if(commandName=="Translate insertion point" && waitCount < 1000) {
				waitCount = waitCount + 1;
			} else {
				hapticsOn = true;
			}

			// UPDATE TUBE CURVATURE
			for(int k=nTubes-1; k>=0; k--) {
				if(lineGripped[k]) {
					updateTubeCurvature(set, k, a1, a2, a3, a4, dotProd);				// If line was being changed but is now released, compute kinematics
				}
				lineGripped[k] = false;
				kappaChange = false;
			}

			// UPDATE TUBE LENGTH
			for(int k=nTubes-1; k>=0; k--) {
				if(lineGrippedForLengthChange[k]) {
					printf("delta Lc: %f \n", deltaLength);
					updateTubeLength(set, k, 0, deltaLength);				// If line was being changed but is now released, compute kinematics
				}
				lineGrippedForLengthChange[k] = false;
				kappaChange = false;
				lengthChange = false;
			}

			// ROTATE TUBE
			if(grippedToRot) {
				if(grippedToRotInd>tubeStartIndReduced[0]) {					// for innermost tube
					updateTubeAlpha(set, 0, angleChange);
					//printf("tube to change: %i \n", 0);		
				} else {									// for all other tubes
					for(int j=nTubes-1; j>0; j--) {
						if((grippedToRotInd>tubeStartIndReduced[j]) && (grippedToRotInd<tubeStartIndReduced[j-1])) {
							updateTubeAlpha(set, j, angleChange);
							//printf("tube to change: %i \n", j);		
						}
					} 
				}
				grippedToRot = false;
			}

			// CHANGE TUBE MATERIAL
			if(changeMaterial) {
				updateTubeMaterialAndConfig(set);
			}
        }

		
		
        /////////////////////////////////////////////////////////////////////
        // COMPUTE AND APPLY FORCES
        /////////////////////////////////////////////////////////////////////
		world->computeGlobalPositions(true);
		tool->updatePose();
		if(!simulationModeOn) {
			tool->computeInteractionForces();
		}

		if(kappaChange) {
			hapticDevice->setForce(forceDir);
			//lengthChange = true;
		}

		if(hapticsOn && !kappaChange) {
			tool->applyForces();
		}

        // update frequency counter
        frequencyCounter.signal(1);

		//if(CTRInitialized) {
		//	if(tool->m_hapticPoint->getNumCollisionEvents()>0) {
		//		printf("tool coll: %i \n",tool->m_hapticPoint->getNumCollisionEvents());
		//	}
		//	for(int i=0; i<numCTRSpheres-11; i++) {
		//		/*CTRTool[i]->setEnabled(true);
		//		CTRTool[i]->updatePose();*/
		//		
		//		//CTRTool[i]->computeInteractionForces();
		//		// check to see how many interaction points there are for each point along CTR
		//		/*int numInteractionPoints = CTRTool[i]->getNumInteractionPoints();
		//		for(int j=0; j<numInteractionPoints; j++) {*/
		//			// get pointer to next interaction point of tool
		//			//cHapticPoint* interactionPoint = CTRTool[i]->getInteractionPoint(j);
		//			// check primary contact point if available
		//			//if(interactionPoint->getNumCollisionEvents() > 0) {
		//			//printf("num collisions: %i \n", CTRTool[i]->m_hapticPoint->getNumCollisionEvents());
		//			//if(CTRTool[i]->m_hapticPoint->getNumCollisionEvents()>0) {
		//			//	//cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(0);
		//			//	//cCollisionEvent* collisionEvent = CTRTool[i]->m_hapticPoint->getCollisionEvent(0);
		//			//	// given the mesh object we may be touching, we search for its owner
		//			//	//cGenericObject* object = collisionEvent->m_object;
		//			//	//printf("object name: %s \n",object->m_name);
		//			//	printf("collision! \n");
		//			//}
		//		//}
		//		if(tube.tubeSphere[i]->computeCollisionDetection(tube.tubeSphere[i]->getGlobalPos(),tube.tubeSphere[i+10]->getGlobalPos(),CTRCollisionRecord,CTRCollisionSetting)){
		//			printf("collided! \n");
		//		}
		//		
		//	}
		//}

    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
// Draw CTR
//------------------------------------------------------------------------------
void drawCTR(float length_straight, float length_curved, float curvature_radius, float diameter) {
	// Clear anything from before
	tubeStartSphere->deleteAllChildren();

	// Determine points that define backbone
	int n_straight = length_straight*10000 - 1;										// number of points for straight part of backbone
	int n_curved = length_curved*10000 - 1;											// number of points for curved part of backbone
	n = n_straight + n_curved;														// total number of points 
	float dist = (length_straight + length_curved)/n;								// distance between points
	printf("n: %i \n", n);

	cVector3d* backbonePoints;														// vector of points along backbone
	backbonePoints = new cVector3d[n];
	backbonePoints[0] = cVector3d(0,0,0);											// define starting point as (0,0,0)

	// for straight section of backbone
	for(int i = 1; i < n_straight+1; i++) {										
		backbonePoints[i] = cVector3d(0,backbonePoints[i-1](1) + dist,0);			// create vector containing coordinates of points along straight part
		//printf("backbone x: %f, backbone y: %f, backbone z: %f \n",backbonePoints[i](0),backbonePoints[i](1),backbonePoints[i](2));
	}

	cVector3d center = backbonePoints[n_straight];
	float theta;
	int j = 0;

	// for curved section of backbone
	for(int i=n_straight+1; i <= n; i++) {											// create vector containing coordinates of points along curved part
		j = j+1;
		theta = (j*(length_curved/n_curved))/curvature_radius;						// angle depends on arc length
		backbonePoints[i](0) = -curvature_radius + curvature_radius*cos(theta);		// x-component 
		backbonePoints[i](1) = center(1) + curvature_radius*sin(theta);				// y-component
		backbonePoints[i](2) = 0;													// z-component
		//printf("backbone x: %f, backbone y: %f, backbone z: %f \n",backbonePoints[i](0),backbonePoints[i](1),backbonePoints[i](2));

	}

	// Draw spheres that make up CTR
	for (int i = 0; i <= n; i++) {
		tube.tubeSphere[i] = new cShapeSphere((diameter/2)*1);						// create sphere with radius equal to radius of CTR
		tubeStartSphere->addChild(tube.tubeSphere[i]);
		tube.tubeSphere[i]->setLocalPos(backbonePoints[i]*1);						// set position of sphere based on points along backbone
		//tube.tubeSphere[i]->translate(-meshCenter);
		tube.tubeSphere[i]->setEnabled(true);
	}

}


void drawTubes(ConcentricTubeSet set) {
	setInvalidated = false;

	// Clear anything from before
	tubeStartSphere->deleteAllChildren();

	//// for debugging
	//tubeStartSphere->setEnabled(true);

	// Draw spheres that make up each tube of CTR
	cVector3d backbonePoints;																				// vector of points along backbone
	cVector3d sEqualsZero = tubeStartSphere->getLocalPos();
	int ind = 0;
	int indReduced = 0;
	int sphereInd = 0;
	float lastLength = 0;
	int lastInd = 0;
	bool* lineDrawn;
	lineDrawn = new bool[nTubes];
	for(int k=nTubes-1; k>=0; k--) {
		lineDrawn[k] = false;
	}
	cGenericEffect* lineEffect[6];
	double s;
	
	// for testing/debugging comment out below
	//// Do initial calculations to determine 3 necessary spheres to render
	//for(int k=nTubes-1; k>=0; k--) {
	//	// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0)	
	//	float length = set.m_tubes[k].Ls + set.m_tubes[k].Lc + set.m_tubes[k].Beta;
	//	
	//	// Store index of start of each tube
	//	tubeStartInd[k] = ind;
	//	tubeEndInd[k] = tubeStartInd[k] + floor((length-lastLength)/hsize);
	//	tubeMidInd[k] = tubeStartInd[k] + floor((length-lastLength)/hsize/2);
	//	
	//	lastLength = length;
	//	ind = tubeEndInd[k] + 1;
	//}
	// for testing/debugging
	tubeStartInd[nTubes-1] = 0;
	tubeEndInd[0] = set.sStored.size()-1;
	
	numCTRSpheres = 0;
	ind = 0;				// set back to 0 for start of determining which spheres to draw
	for(int k=nTubes-1; k>=0; k--) {	
		
		int testInd = 0;

		// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0)	
		float length = set.m_tubes[k].Ls + set.m_tubes[k].Lc + set.m_tubes[k].Beta;

		// for testing/deubgging
		if(k!=(nTubes-1)) {
			tubeStartInd[k] = ind;
		} 

		

		s = set.sStored[ind];
		while(s<length)
		{
			testInd = testInd + 1;				// only want to draw every 10th sphere (or start,middle,end)
			
			if((s>=0) && ((ind==tubeStartInd[k])||(ind==tubeMidInd[k])||(ind==tubeEndInd[k])||(testInd==3))){			// NOTE: USED TO BE (TESTIND == 5) SO THAT IT WOULD ONLY DRAW EVERY 10TH SPHERE
			//if(s>=0) {
				// Reset to zero each time to make sure we only draw every 5th sphere
				testInd = 0;
				sphereInd = sphereInd + 1;

				tube.tubeSphere[indReduced] = new cShapeSphere((set.m_tubes[k].OD)/2);									// create sphere with radius equal to radius of tube i
				tubeStartSphere->addChild(tube.tubeSphere[indReduced]);												// add new sphere as child to tube start sphere
				backbonePoints = cVector3d(set.positionStored.x[ind],											// set backbone points based on stored xyz position computed from kinematics
										   set.positionStored.y[ind], 
										   set.positionStored.z[ind]);		
				// If in simulation mode, want to draw from s=0 as the start sphere
				if(s==0) {
					sEqualsZero = cVector3d(set.positionStored.x[ind],											// Save position at s=0
										   set.positionStored.y[ind], 
										   set.positionStored.z[ind]);
				}
				if(simulationModeOn) {
					backbonePoints = backbonePoints-sEqualsZero;
				}

				tube.tubeSphere[indReduced]->setLocalPos(backbonePoints);												// set position of sphere based on points along backbone
				tube.tubeSphere[indReduced]->setMaterial(tubeColor[k]);

				// Add effect to CTR
				cGenericEffect* effectCTR;																		// temp variable
				effectCTR = new cEffectMagnet(tube.tubeSphere[indReduced]);											// create a haptic effect
				tube.tubeSphere[indReduced]->addEffect(effectCTR);														// add effect to object
				tube.tubeSphere[indReduced]->m_material->setMagnetMaxDistance(0.01);
				tube.tubeSphere[indReduced]->m_material->setStiffness(0.2*maxStiffness);								// set parameters related to effect
				tube.tubeSphere[indReduced]->setHapticEnabled(true);
				tube.tubeSphere[indReduced]->setEnabled(true);

				// -------------------- testing ------------------------
				// make spheres into tools for collision detection
				/*if(sphereInd==5) {
					CTRTool[numCTRSpheres] = new cToolCursor(world);
					tube.tubeSphere[indReduced]->addChild(CTRTool[numCTRSpheres]);
					CTRTool[numCTRSpheres]->setRadius(1.2*(set.m_tubes[k].OD)/2);
					CTRTool[numCTRSpheres]->setWorkspaceRadius(0.7);
					CTRTool[numCTRSpheres]->setShowEnabled(true);
					CTRTool[numCTRSpheres]->start();
					CTRTool[numCTRSpheres]->initialize();
					CTRTool[numCTRSpheres]->setEnabled(true);
					CTRTool[numCTRSpheres]->setHapticEnabled(true);
					sphereInd = 0;
					numCTRSpheres = numCTRSpheres + 1;	
				}*/
				/*tube.tubeSphere[indReduced]->setCollisionDetector(CTRCollisionDetect[indReduced]);
				tube.tubeSphere[indReduced]->setShowCollisionDetector(true);
				numCTRSpheres = indReduced;*/
				// ------------------------------------------------------


				if((ind==0) && (!simulationModeOn)) {
					proximalSphere = new cShapeSphere(0.01);
					tubeStartSphere->addChild(proximalSphere);
					proximalSphere->setLocalPos(backbonePoints);
					proximalSphere->setTransparencyLevel(0.5);

					// Add effect to proximal end of tube
					cGenericEffect* newEffect;											 // temp variable
					proximalSphere->m_material->setWhiteHoneydew();						 // set material color
					newEffect = new cEffectSurface(proximalSphere);						 // create a haptic effect
					proximalSphere->addEffect(newEffect);								 // add effect to object
					proximalSphere->m_material->setStiffness(0.8 * maxStiffness);		 // set parameters related to effect
					proximalSphere->setHapticEnabled(true);
					proximalSphere->setEnabled(true);
				}

				//if((!lineDrawn[k]) && (!simulationModeOn) && (!lineGripped[k])) {
				//	if(ind == tubeMidInd[k]) {
				//		float height = ((set.m_tubes[k].OD)/2)+0.008;

				//		cVector3d tubeStartPoint = cVector3d(set.positionStored.x[tubeStartInd[k]],set.positionStored.y[tubeStartInd[k]],set.positionStored.z[tubeStartInd[k]]);
				//		cVector3d tubeEndPoint = cVector3d(set.positionStored.x[tubeEndInd[k]],set.positionStored.y[tubeEndInd[k]],set.positionStored.z[tubeEndInd[k]]);
				//		cVector3d lineEndPoint = computeEndPoint(tubeStartPoint, tubeEndPoint, backbonePoints, height);
				//		centerLine[k] = new cShapeLine(lineEndPoint,backbonePoints);


				//		//centerLine[k] = new cShapeLine(backbonePoints-cVector3d(height,height,height),backbonePoints);
				//		tubeStartSphere->addChild(centerLine[k]);
				//		centerLine[k]->m_material->setWhiteHoneydew();
				//		centerLine[k]->setLineWidth(30);

				//		lineEffect[k] = new cEffectMagnet(centerLine[k]);
				//		centerLine[k]->addEffect(lineEffect[k]);
				//		centerLine[k]->m_material->setMagnetMaxDistance(0.01);
				//		centerLine[k]->m_material->setStiffness(0.01 * maxStiffness);

				//		lineDrawn[k] = true;
				//		lastLength = length;
				//		lastInd = ind + (ind-lastInd);
				//		centerLine[k]->setEnabled(true);
				//		//tubeMidInd[k] = ind;
				//	}
				//}

				// try adding distal sphere as well
				if((ind==tubeEndInd[0]) && !simulationModeOn) {
					distalSphere = new cShapeSphere(0.006);
					tubeStartSphere->addChild(distalSphere);
					distalSphere->setLocalPos(backbonePoints);
					distalSphere->setTransparencyLevel(0.9);

					// Add effect to proximal end of tube
					cGenericEffect* newEffect;										// temp variable
					//distalSphere->m_material->setGraySilver();						// set material color
					distalSphere->setMaterial(tubeColor[0]);
					newEffect = new cEffectSurface(distalSphere);					// create a haptic effect
					distalSphere->addEffect(newEffect);								// add effect to object
					distalSphere->m_material->setStiffness(0.5 * maxStiffness);		// set parameters related to effect
					distalSphere->setHapticEnabled(true);
					distalSphere->setEnabled(true);

				}

				// try drawing line btwn spheres
				//if(ind!=0) {
				//	tube.tubeLine[indReduced] = new cShapeLine(tube.tubeSphere[indReduced]->getLocalPos(),tube.tubeSphere[indReduced-1]->getLocalPos());		
				//	tubeStartSphere->addChild(tube.tubeLine[indReduced]);
				//	// set color of line
				//	tube.tubeLine[indReduced]->m_colorPointA.setR(lineColor[k].getR());
				//	tube.tubeLine[indReduced]->m_colorPointA.setG(lineColor[k].getG());
				//	tube.tubeLine[indReduced]->m_colorPointA.setB(lineColor[k].getB());
				//	tube.tubeLine[indReduced]->m_colorPointB.setR(lineColor[k].getR());
				//	tube.tubeLine[indReduced]->m_colorPointB.setG(lineColor[k].getG());
				//	tube.tubeLine[indReduced]->m_colorPointB.setB(lineColor[k].getB());
				//	// set width of line
				//	tube.tubeLine[indReduced]->setLineWidth(1000*set.m_tubes[k].OD);
				//	/*cVector3d tempLine = (tube.tubeSphere[indReduced-1]->getLocalPos())-(tube.tubeSphere[indReduced]->getLocalPos());
				//	tubeStartSphere->addChild(tube.tubeCylinder[indReduced]);
				//	float height = tempLine.length();
				//	tube.tubeCylinder[indReduced] = new cShapeCylinder(set.m_tubes[k].OD/2, set.m_tubes[k].OD/2, height,tubeColor[k].create());
				//	tube.tubeCylinder[indReduced]->setLocalPos(tube.tubeSphere[indReduced]->getLocalPos()(0),tube.tubeSphere[indReduced]->getLocalPos()(1),tube.tubeSphere[indReduced]->getLocalPos()(2));*/
				//}



				// keep track of new indices based on only drawing every 5
				if(ind==tubeStartInd[k]) {
					tubeStartIndReduced[k] = indReduced;
				} else if(ind==tubeMidInd[k]) {
					tubeMidIndReduced[k] = indReduced;
				} else if(ind==tubeEndInd[k]) {
					tubeEndIndReduced[k] = indReduced;
				}
				// use to keep track of indices for tubeSphere
				indReduced = indReduced + 1;


			}
			/*if(!designModeOn) {
				proximalSphere->setEnabled(false);
				centerLine[k]->setEnabled(false);
			}*/
			ind = ind + 1;

			if(ind<(set.sStored.size())) {
				s = set.sStored[ind];
			} else {
				s = length;
			}
			
		}

		// Try setting mid index and drawing lines
		// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0)
		tubeEndInd[k] = ind-1;
		tubeMidInd[k] = tubeStartInd[k] + floor((tubeEndInd[k]-tubeStartInd[k])/2.0);

		if((!lineDrawn[k]) && (!simulationModeOn) && (!lineGripped[k])) {
			float height = ((set.m_tubes[k].OD)/2)+0.008;

			cVector3d tubeStartPoint = cVector3d(set.positionStored.x[tubeStartInd[k]],set.positionStored.y[tubeStartInd[k]],set.positionStored.z[tubeStartInd[k]]);
			cVector3d tubeEndPoint = cVector3d(set.positionStored.x[tubeEndInd[k]],set.positionStored.y[tubeEndInd[k]],set.positionStored.z[tubeEndInd[k]]);
			cVector3d midPointBackbonePoint = cVector3d(set.positionStored.x[tubeMidInd[k]],											// set backbone points based on stored xyz position computed from kinematics
														set.positionStored.y[tubeMidInd[k]], 
														set.positionStored.z[tubeMidInd[k]]);
			cVector3d lineEndPoint = computeEndPoint(tubeStartPoint, tubeEndPoint, midPointBackbonePoint, height);
			centerLine[k] = new cShapeLine(lineEndPoint,midPointBackbonePoint);

			tubeStartSphere->addChild(centerLine[k]);
			centerLine[k]->m_material->setWhiteHoneydew();
			centerLine[k]->setLineWidth(100);

			lineEffect[k] = new cEffectMagnet(centerLine[k]);
			centerLine[k]->addEffect(lineEffect[k]);
			centerLine[k]->m_material->setMagnetMaxDistance(0.015);
			centerLine[k]->m_material->setStiffness(0.5 * maxStiffness);


			lineDrawn[k] = true;
			lastLength = length;
			lastInd = ind + (ind-lastInd);
			centerLine[k]->setEnabled(true);
		}
		
	}


	//// Draw spheres for changing length
	//if(designModeOn) {
	//	for(int k=nTubes-1; k>=0; k--) {
	//		lengthSphere[k] = new cShapeSphere(0.004);
	//		tubeStartSphere->addChild(lengthSphere[k]);
	//		// Need different method for sphere for innermost tube
	//		if(k==0) {
	//			lengthSphere[k]->setLocalPos(set.positionStored.x[set.positionStored.x.size()-1],											
	//										 set.positionStored.y[set.positionStored.y.size()-1], 
	//										 set.positionStored.z[set.positionStored.z.size()-1]);
	//			lengthSphereInd[k] = set.positionStored.x.size()-1;
	//		} else {
	//			lengthSphere[k]->setLocalPos(set.positionStored.x[tubeStartInd[k-1]],											
	//										 set.positionStored.y[tubeStartInd[k-1]], 
	//										 set.positionStored.z[tubeStartInd[k-1]]);
	//			lengthSphereInd[k] = tubeStartInd[k-1];
	//		}
	//		lengthSphere[k]->setTransparencyLevel(0.5);

	//		// Add effect to spheres
	//		cGenericEffect* newEffect;												 // temp variable
	//		lengthSphere[k]->m_material->setWhiteHoneydew();						 // set material color
	//		newEffect = new cEffectSurface(lengthSphere[k]);						 // create a haptic effect
	//		lengthSphere[k]->addEffect(newEffect);									// add effect to object
	//		lengthSphere[k]->m_material->setStiffness(0.8 * maxStiffness);			// set parameters related to effect
	//		lengthSphere[k]->setHapticEnabled(true);
	//		lengthSphere[k]->setEnabled(true);
	//	} 
	//} else {
	//	for(int k=nTubes-1; k>=0; k--) {
	//		lengthSphere[k]->setEnabled(false);
	//	}
	//}
}

void drawSimulation(ConcentricTubeSet set) {
	setInvalidated = false;

	// Clear anything from before
	tubeStartSphere->deleteAllChildren();

	// Draw spheres that make up each tube of CTR
	cVector3d backbonePoints;																				// vector of points along backbone
	cVector3d sEqualsZero = tubeStartSphere->getLocalPos();
	int ind = 0;
	int indReduced = 0;
	float lastLength = 0;
	int lastInd = 0;
	double s;
	
	// Do initial calculations to determine 3 necessary spheres to render
	for(int k=nTubes-1; k>=0; k--) {
		// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0)	
		float length = set.m_tubes[k].Ls + set.m_tubes[k].Lc + set.m_tubes[k].Beta;
		
		// Store index of start of each tube
		tubeStartInd[k] = ind;
		tubeEndInd[k] = tubeStartInd[k] + floor((length-lastLength)/hsize);
		tubeMidInd[k] = tubeStartInd[k] + floor((length-lastLength)/hsize/2);
		
		lastLength = length;
		ind = tubeEndInd[k] + 1;
	}
	

	ind = 0;				// set back to 0 for start of determining which spheres to draw
	for(int k=nTubes-1; k>=0; k--) {	
		
		int testInd = 0;

		// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0)	
		float length = set.m_tubes[k].Ls + set.m_tubes[k].Lc + set.m_tubes[k].Beta;

		s = set.sStored[ind];
		while(s<length)
		{
			testInd = testInd + 1;				// only want to draw every 10th sphere (or start,middle,end)
			if(s>=0) {
				// Reset to zero each time to make sure we only draw every 5th sphere
				testInd = 0;

				tube.tubeSphere[indReduced] = new cShapeSphere((set.m_tubes[k].OD)/2);									// create sphere with radius equal to radius of tube i
				tubeStartSphere->addChild(tube.tubeSphere[indReduced]);												// add new sphere as child to tube start sphere
				backbonePoints = cVector3d(set.positionStored.x[ind],											// set backbone points based on stored xyz position computed from kinematics
										   set.positionStored.y[ind], 
										   set.positionStored.z[ind]);		
				// If in simulation mode, want to draw from s=0 as the start sphere
				if(s==0) {
					sEqualsZero = cVector3d(set.positionStored.x[ind],											// Save position at s=0
										   set.positionStored.y[ind], 
										   set.positionStored.z[ind]);
				}
				if(simulationModeOn) {
					backbonePoints = backbonePoints-sEqualsZero;
				}

				tube.tubeSphere[indReduced]->setLocalPos(backbonePoints);												// set position of sphere based on points along backbone
				tube.tubeSphere[indReduced]->setMaterial(tubeColor[k]);

				// Add effect to CTR
				cGenericEffect* effectCTR;																		// temp variable
				effectCTR = new cEffectMagnet(tube.tubeSphere[indReduced]);											// create a haptic effect
				tube.tubeSphere[indReduced]->addEffect(effectCTR);														// add effect to object
				tube.tubeSphere[indReduced]->m_material->setMagnetMaxDistance(0.01);
				tube.tubeSphere[indReduced]->m_material->setStiffness(0.2*maxStiffness);								// set parameters related to effect
				tube.tubeSphere[indReduced]->setHapticEnabled(true);
				tube.tubeSphere[indReduced]->setEnabled(true);

				// keep track of new indices based on only drawing every 5
				if(ind==tubeStartInd[k]) {
					tubeStartIndReduced[k] = indReduced;
				} else if(ind==tubeMidInd[k]) {
					tubeMidInd[k] = indReduced;
				} else if(ind==tubeEndInd[k]) {
					tubeEndIndReduced[k] = indReduced;
				}
				// use to keep track of indices for tubeSphere
				indReduced = indReduced + 1;
			}
			
			ind = ind + 1;

			if(ind<(set.sStored.size())) {
				s = set.sStored[ind];
			} else {
				s = length;
			}
			
		}

		
	}
}
//------------------------------------------------------------------------------
// Update tube location or orientation
//------------------------------------------------------------------------------
void updateInsertionPoint(ConcentricTubeSet set, float insertPointX, float insertPointY, float insertPointZ) {
	tubeStartSphere->setLocalPos(insertPointX,insertPointY,insertPointZ);
}

void updateOrientation(ConcentricTubeSet set, cVector3d &axisToRotateAbout) {
	tubeStartSphere->rotateAboutGlobalAxisDeg(axisToRotateAbout, 20);
}

void updateInitOrientation(ConcentricTubeSet set, cVector3d &axisToRotateAbout, double angle) {
	tubeStartSphere->rotateAboutGlobalAxisRad(axisToRotateAbout, angle);
}

cMatrix3d calculateRotMat(cVector3d pA, cVector3d pB) {
	// note that pA and pB must be unit vectors
	double cosTheta = cDot(pA,pB);
	double sinTheta = cCross(pA,pB).length();
	cMatrix3d G = cMatrix3d(cosTheta,-sinTheta,0,sinTheta,cosTheta,0,0,0,1);
	cVector3d u = pA;
	cVector3d v = (pB-cosTheta*pA)/((pB-cosTheta*pA).length());
	cVector3d w = cCross(pB,pA);
	cMatrix3d F;
	cMatrix3d Finv;
	F.setCol0(u);
	F.setCol1(v);
	F.setCol2(w);
	Finv = F;
	F.invert();
	cMatrix3d rot = cMul(cMul(Finv,G),F);
	rot.invert();

	return rot;		// rotation from B to A (A_R_B)
}

//------------------------------------------------------------------------------
// Update tube parameters
//------------------------------------------------------------------------------
void updateTubeLength(ConcentricTubeSet &set, int tubeIndex, float deltaLs, float deltaLc) {
	float oldLengthTot = set.m_tubes[tubeIndex].Ls + set.m_tubes[tubeIndex].Lc;
	float newLengthTot =  (set.m_tubes[tubeIndex].Ls) + deltaLs + (set.m_tubes[tubeIndex].Lc) + deltaLc;
	
	if(newLengthTot < maxLength[set.m_tubes[tubeIndex].materialNum]) {
		if(tubeIndex==0) {								// innermost tube
			float prevTubeLength = (set.m_tubes[tubeIndex+1].Ls) + (set.m_tubes[tubeIndex+1].Lc);
			if(newLengthTot>prevTubeLength) {			// make sure it's still longer than previous tube
				set.m_tubes[tubeIndex].Ls = (set.m_tubes[tubeIndex].Ls) + deltaLs;
				set.m_tubes[tubeIndex].Lc = (set.m_tubes[tubeIndex].Lc) + deltaLc;
				warningLabel->setEnabled(false);
			} else {									// if it isn't, then don't change tube length
				warningName = "min length";
				warningLabel->setEnabled(true);
			}
		} else if(tubeIndex == nTubes-1) {				// outermost tube
			float nextTubeLength = (set.m_tubes[tubeIndex-2].Ls) + (set.m_tubes[tubeIndex-2].Lc);
			if(newLengthTot<nextTubeLength) {			// make sure it's still shorter than next tube
				set.m_tubes[tubeIndex].Ls = (set.m_tubes[tubeIndex].Ls) + deltaLs;
				set.m_tubes[tubeIndex].Lc = (set.m_tubes[tubeIndex].Lc) + deltaLc;
				warningLabel->setEnabled(false);
			} else {									// if it isn't, then don't change tube length
				warningName = "max length";
				warningLabel->setEnabled(true);
			}
		} else {										// all other tubes								
			float prevTubeLength = (set.m_tubes[tubeIndex+1].Ls) + (set.m_tubes[tubeIndex+1].Lc);
			float nextTubeLength = (set.m_tubes[tubeIndex-1].Ls) + (set.m_tubes[tubeIndex-1].Lc);
			if((newLengthTot<nextTubeLength) && (newLengthTot>prevTubeLength)) {			// make sure it's still shorter than next tube and longer than previous tube
				set.m_tubes[tubeIndex].Ls = (set.m_tubes[tubeIndex].Ls) + deltaLs;
				set.m_tubes[tubeIndex].Lc = (set.m_tubes[tubeIndex].Lc) + deltaLc;
				warningLabel->setEnabled(false);
			} else {									// if it isn't, then don't change tube length
				warningName = "outside range";
				warningLabel->setEnabled(true);
			}
		}
	} else {
		warningName = "max length";
		warningLabel->setEnabled(true);
		printf("max length for material hit \n");
	}

	if(newLengthTot != oldLengthTot) {
		waitLabel->setEnabled(true);
		runKinematics();
		waitLabel->setEnabled(false);
		commandName = "";
	}

	if(set.isValidSet) {
		drawTubes(set);
	} else {
		printf("could not solve \n");
		
	}

}

void updateTubeCurvature(ConcentricTubeSet &set, int tubeIndex, cVector3d a1, cVector3d a2, cVector3d a3, cVector3d a4, float dotProd) {
	
	// Compute radius of circle based on three points a1,a2,a3
	cVector3d a = a1-a3;
	cVector3d b = a2-a3;
	cVector3d amb = a-b;
	float ambMag = sqrt(amb*amb);
	cVector3d axb = cCross(a,b);
	float axbMag = sqrt(axb*axb);
	float r = (sqrt(a*a)*sqrt(b*b)*ambMag)/(2*axbMag);

	// Calculate cross products to determine if there's a change in sign
	/*cVector3d c = a2-a1;
	cVector3d d = a4-a1;
	cVector3d e = cCross(c,d);
	e = cNormalize(e);
	cVector3d f = a3-a1;
	cVector3d g = cCross(c,f);
	g = cNormalize(g);
	bool sameSign;
	printf("testing costheta: %f \n", cDot(e,g));
	printf("e: %f %f %f \n",e(0),e(1),e(2));
	printf("g: %f %f %f \n",g(0),g(1),g(2));*/
	

	/*if(cDot(e,g)>0) {
		sameSign = true;
	} else {
		sameSign = false;
	}*/
	// save old value of kappa
	float oldKappa = set.m_tubes[tubeIndex].kappa;
	kappaLast = oldKappa;
	lastTube = tubeIndex;

	// Check for what radius of curvature should be for tube to be changed (based on assumption that EQUILIBRIUM CURVATURE is newly computed curvature)
	float kappaEqNew = 1/r;
	float rTubeNew = r;
	if(tubeIndex==0) {
		/*printf("kappaOld: %f \n", oldKappa);
		printf("kappaNew: %f \n", 1/r);
		printf("diff: %f \n", oldKappa-(1/r));*/
		if(dotProd>0) {	// want to decrease curvature
			rTubeNew = 1/(oldKappa - (oldKappa-(1/r)));
		} else {		// want to increase curvature
			rTubeNew = 1/(oldKappa + abs(oldKappa-(1/r)));
		}
	} else {
		float sum = 0;
		for(int i=1; i<nTubes; i++) {
			float I = (M_PI/64)*((pow(set.m_tubes[i].OD,4))-(pow(set.m_tubes[i].ID,4)));
			sum = sum + kappaEqNew*I*set.m_tubes[i].E;
		}
		rTubeNew = (((M_PI/64)*((pow(set.m_tubes[tubeIndex].OD,4))-(pow(set.m_tubes[tubeIndex].ID,4))))*set.m_tubes[tubeIndex].E)/sum;
	}
	

	// Check to see if curvature exceeds allowable curvature depending on material
	float rMin = (set.m_tubes[tubeIndex].OD)/(2*maxStrain);
	/*if(abs(r)>rMin) {
		if(sameSign) {
			set.m_tubes[tubeIndex].kappa = 1/r;
		} else { 
			set.m_tubes[tubeIndex].kappa = -1/r;
		}*/
	if(abs(rTubeNew)>rMin) {
		//if(sameSign) {
			set.m_tubes[tubeIndex].kappa = 1/rTubeNew;
		//} else { 
			//set.m_tubes[tubeIndex].kappa = -1/rTubeNew;
		//}
		warningName = "";
		warningLabel->setEnabled(false);
	} else {
		//if(sameSign) {
			set.m_tubes[tubeIndex].kappa = 1/rMin;
		//} else {
			//set.m_tubes[tubeIndex].kappa = -1/rMin;
		//}
		warningName = "Exceeds max strain";
		warningLabel->setEnabled(true);
	}

	printf("changed kappa: %f \n",set.m_tubes[tubeIndex].kappa);
	if((set.m_tubes[tubeIndex].kappa) != oldKappa) {					// only compute kinematics if kappa value has changed
		waitLabel->setEnabled(true);
		kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
		waitLabel->setEnabled(false);
	}

	if(set.isValidSet) {
		drawTubes(set);
	} else {
		printf("could not solve \n");
		// if couldn't solve, go back to original kappa value
		set.m_tubes[tubeIndex].kappa = oldKappa;
		drawTubes(set);
	}
}

void updateTubeMaterial(ConcentricTubeSet &set, int tubeIndex, int materialNumber) {
	set.m_tubes[tubeIndex].E = EVec[materialNumber];
	set.m_tubes[tubeIndex].v = vVec[materialNumber];
	maxStrain = maxStrainVec[materialNumber];

	materialName = materialNameVec[materialNumber];

	// Update display
	for(int i=0; i<3; i++) {
		if(i==materialNumber){
			materialOptionDisplay[materialNumber]->setValue(40.0);
			materialOptionDisplay[materialNumber+3]->setValue(39.0);
		} else {
			materialOptionDisplay[i]->setValue(0.0);
			materialOptionDisplay[i+3]->setValue(0.0);
		}
	}
}

void updateTubeMaterialAndConfig(ConcentricTubeSet &set) {
	// make sure curvatures are achievable with given materials?
	for(int i=0; i<nTubes; i++) {
		float rMin = (set.m_tubes[i].OD)/(2*maxStrain);
		float r = 1/set.m_tubes[i].kappa;
		if(abs(r)<rMin) {
			if(r>0) {
				set.m_tubes[i].kappa = 1/rMin;
			} else {
				set.m_tubes[i].kappa = -1/rMin;
			}
			warningName = "Tube " + to_string(static_cast<long double>(i)) + " exceeded max strain";
			warningLabel->setEnabled(true);
		}
	}

	kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
	if(set.isValidSet) {
		drawTubes(set);
	} else {
		printf("could not solve \n");
	}
	changeMaterial = false;
}

void updateTubeAlpha(ConcentricTubeSet &set, int tubeIndex, double angleChange) {
	
	float deltaAlpha = (float)(angleChange) * 2;

	printf("old alpha: %f \n",set.m_tubes[tubeIndex].alpha);
	set.m_tubes[tubeIndex].alpha = (set.m_tubes[tubeIndex].alpha) + deltaAlpha;
	printf("new alpha: %f \n",set.m_tubes[tubeIndex].alpha);

	//runKinematics();

	kinematicsThread->start(runKinematics, CTHREAD_PRIORITY_HAPTICS);
	//drawTubes(set);
	if(set.isValidSet) {
		drawTubes(set);
	} else {
		printf("could not solve \n");
	}
}


//------------------------------------------------------------------------------
// Compute configurations for simulation
//------------------------------------------------------------------------------
//void calculateConfig(ConcentricTubeSet &set) {
//	calculated = true;
//	readyToDraw = false;
//
//	simulatedBeta.clear();
//	simulatedAlpha.clear();
//
//	// Create empty vector to store Beta values and alpha values
//	for(unsigned int j = 0; j<nTubes; j++) {
//		std::vector<float> newRow;
//		simulatedBeta.push_back(newRow);
//		simulatedAlpha.push_back(newRow);
//	}
//
//	// Calculate total tube lengths to determine initial Beta values and set initial alpha values to 0
//	for(int k=nTubes-1; k>=0; k--) {
//		float sigma = set.m_tubes[k].Lc + set.m_tubes[k].Ls;
//		float initBetaVal = -sigma + 0.001 + (0.001*(nTubes-k-1));
//		std::vector<float> rowBeta = simulatedBeta[k];
//		rowBeta.push_back(initBetaVal);					//Only manipulate one row
//		simulatedBeta[k] = rowBeta;						//Store row back into matrix
//
//		float initAlphaVal = 0;
//		std::vector<float> rowAlpha = simulatedAlpha[k];
//		rowAlpha.push_back(initAlphaVal);				//Only manipulate one row
//		simulatedAlpha[k] = rowAlpha;					//Store row back into matrix
//	}
//
//	// Create set of Beta
//	int iter = 0;
//	float newBetaVal;
//	for(int i=0; i<nTubes; i++) {
//		float inc = (abs(simulatedBeta[nTubes-iter-1][iter*10]) -abs(set.m_tubes[nTubes-iter-1].Beta))/10;			// amount to increment Beta each time step
//		for(int j=0; j<10; j++) {
//			for(int k=nTubes-1; k>=0; k--) {
//				std::vector<float> rowBeta = simulatedBeta[k];
//				if((i!=0) && (k>=(nTubes-iter))) {
//					newBetaVal = rowBeta[rowBeta.size()-1];			
//				} else {
//					newBetaVal = rowBeta[rowBeta.size()-1] + inc;
//				}
//				rowBeta.push_back(newBetaVal);					//Only manipulate one row
//				simulatedBeta[k] = rowBeta;						//Store row back into matrix
//				
//				// for debugging
//				/*if(k==0) {
//					printf("Beta tube 2: %f \t i val: %i \n",newBetaVal, i);
//				}*/
//
//			}
//		}
//		iter = iter + 1;
//	}
//
//	// Create set of alpha
//	iter = 0;
//	float newAlphaVal;
//	for(int i=0; i<nTubes; i++) {
//		float inc = ((set.m_tubes[nTubes-iter-1].alpha) - (simulatedAlpha[nTubes-iter-1][iter*4]))/4;			// amount to increment Beta each time step
//
//		for(int j=0; j<4; j++) {
//			for(int k=nTubes-1; k>=0; k--) {
//				std::vector<float> rowAlpha = simulatedAlpha[k];
//				if(k==(nTubes-iter-1)) {
//					newAlphaVal = rowAlpha[rowAlpha.size()-1] + inc;
//				} else {
//					newAlphaVal = rowAlpha[rowAlpha.size()-1];	
//				}
//				rowAlpha.push_back(newAlphaVal);					//Only manipulate one row
//				simulatedAlpha[k] = rowAlpha;						//Store row back into matrix
//
//				//// for debugging
//				//if(k==0) {
//				//	printf("alpha tube 2: %f \t i val: %i \n",newAlphaVal, i);
//				//}
//			}
//		}
//		iter = iter + 1;
//	}
//
//
//	// Calculate configuration for each alpha/Beta pair and store final configuration info.
//	std::vector<float> rowB = simulatedBeta[0];
//	std::vector<float> rowA = simulatedAlpha[0];
//	int numSets = rowB.size() + rowA.size();						// number of configurations during simulation
//	//delete simulatedSets;
//	simulatedSets = new ConcentricTubeSet[numSets];					// allocate memory for sets to store
//	
//	tool->setForcesOFF();											// turn forces off during computations
//	
//	ConcentricTubeSet currentSet;
//	for(int i=0; i<numSets; i++) {
//		currentSet = set;
//		for(int k=nTubes-1; k>=0; k--) {
//			if(i<rowB.size()) {
//				currentSet.m_tubes[k].Beta = simulatedBeta[k][i];
//				currentSet.m_tubes[k].alpha = simulatedAlpha[k][0];
//			} else {
//				currentSet.m_tubes[k].alpha = simulatedAlpha[k][i-rowB.size()];
//				//currentSet.m_tubes[k].Beta = simulatedBeta[k][rowB.size()-1];
//			}
//		}
//
//		waitLabel->setEnabled(true);
//		kinematics(currentSet);											// compute kinematics to determine configuration for each time step
//		waitLabel->setEnabled(false);
//		simulatedSets[i] = currentSet;									// Store entire SET to draw later
//		printf("time step: %i \n",i);
//	}
//
//	tool->setForcesON();											// turn forces back on
//	printf("CalculateConfig() done\n");
//
//	//*******
//	currSetDrawing = 0;
//	
//	modeName = "Simulation Mode";
//	commandName = " ";
//}

void calculateConfig(ConcentricTubeSet &set) {
	calculated = true;
	readyToDraw = false;

	simulatedBeta.clear();
	simulatedAlpha.clear();

	// Create empty vector to store Beta values and alpha values
	for(unsigned int j = 0; j<nTubes; j++) {
		std::vector<float> newRow;
		simulatedBeta.push_back(newRow);
		simulatedAlpha.push_back(newRow);
	}

	// Calculate total tube lengths to determine initial Beta values and set alpha values to final alpha values as calculated (should remain fixed!!)
	for(int k=nTubes-1; k>=0; k--) {
		float sigma = set.m_tubes[k].Lc + set.m_tubes[k].Ls;
		//float initBetaVal = -sigma + 0.001 + (0.001*(nTubes-k-1));
		float initBetaVal = -sigma + 0.00001 + (0.000001*(nTubes-k-1));
		std::vector<float> rowBeta = simulatedBeta[k];
		rowBeta.push_back(initBetaVal);					//Only manipulate one row
		simulatedBeta[k] = rowBeta;						//Store row back into matrix
		std::vector<float> rowAlpha = simulatedAlpha[k];
		rowAlpha.push_back(set.m_tubes[k].alpha);		//Only manipulate one row
		simulatedAlpha[k] = rowAlpha;					//Store row back into matrix
	}

	// Create set of Beta
	int divisions = 7;  //3;		//10;
	int iter = 0;
	float newBetaVal;
	for(int i=0; i<nTubes; i++) {
		float inc = (abs(simulatedBeta[nTubes-iter-1][iter*divisions]) -abs(set.m_tubes[nTubes-iter-1].Beta))/divisions;			// amount to increment Beta each time step
		for(int j=0; j<divisions; j++) {
			for(int k=nTubes-1; k>=0; k--) {
				std::vector<float> rowBeta = simulatedBeta[k];
				/*if((i==0) && (k==(nTubes-1)) && (j>=(divisions-1))) {
					newBetaVal = rowBeta[rowBeta.size()-1];
				} else if((i!=0) && (k>=(nTubes-iter))) {*/
				if((i!=0) && (k>=(nTubes-iter))) {
					newBetaVal = rowBeta[rowBeta.size()-1];			
				} else {
					newBetaVal = rowBeta[rowBeta.size()-1] + inc;
				}
				rowBeta.push_back(newBetaVal);					//Only manipulate one row
				simulatedBeta[k] = rowBeta;						//Store row back into matrix
				
				// for debugging
				/*if(k==0) {
					printf("Beta tube 2: %f \t i val: %i \n",newBetaVal, i);
				}*/

			}
		}
		iter = iter + 1;
}

	// Calculate configuration for each alpha/Beta pair and store final configuration info.
	std::vector<float> rowB = simulatedBeta[0];
	int numSets = rowB.size();										// number of configurations during simulation
	printf("num sets: %i  \n", numSets);
	//delete simulatedSets;
	simulatedSets = new ConcentricTubeSet[numSets];					// allocate memory for sets to store
	
	tool->setForcesOFF();											// turn forces off during computations
	
	ConcentricTubeSet currentSet;
	numValidSets = 0;
	for(int i=0; i<numSets; i++) {
		currentSet = set;
		for(int k=nTubes-1; k>=0; k--) {
			currentSet.m_tubes[k].Beta = simulatedBeta[k][i];
			currentSet.m_tubes[k].alpha = simulatedAlpha[k][0];			// alpha remains fixed throughout
			// **** for debugging ***
			float checkingLength = set.m_tubes[k].Lc + set.m_tubes[k].Ls + simulatedBeta[k][i];
			printf("length: %f \n",checkingLength);
		}

		waitLabel->setEnabled(true);
		kinematics(currentSet);											// compute kinematics to determine configuration for each time step
		waitLabel->setEnabled(false);

		// only store if kinematics calculation was valid
		if(currentSet.isValidSet) {
			simulatedSets[numValidSets] = currentSet;									// Store entire SET to draw later
			numValidSets = numValidSets + 1;
		}

		printf("time step: %i \n",i);
	}

	tool->setForcesON();											// turn forces back on
	printf("CalculateConfig() done\n");

	//*******
	currSetDrawing = 0;
	
	modeName = "Simulation Mode";
	commandName = " ";
}

void simulate(void) {

	//std::vector<float> rowB = simulatedBeta[0];
	//std::vector<float> rowA = simulatedAlpha[0];
	//int numSets = rowB.size();
	////int numSets = rowB.size() + rowA.size();						// number of configurations during simulation

	//if((simulatedSets != NULL) && (currSetDrawing < numSets)) {
	//	drawTubes(simulatedSets[currSetDrawing]);
	//	/*if(currIter>10) {
	//		currSetDrawing++;
	//		currIter = 0;
	//	}
	//	currIter++;*/
	//	currSetDrawing++;
	//} else if((simulatedSets != NULL) && (currSetDrawing == numSets)) {
	//	 simulationRunning = true;
	//	 hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
	//	 tool->start();
	//	 readyToDraw = false;
	//}
	int button1;

	if((simulatedSets != NULL) && (currSetDrawing < numValidSets)) {
		button1 = tool->getUserSwitch(1); // check to see if user tried to pause simulation
		if(button1==1) {
			pauseSimPressed = !pauseSimPressed;
		}
		if(!pauseSimPressed) {
			drawSimulation(simulatedSets[currSetDrawing]);
			currSetDrawing++;
		}

		//button1 = tool->getUserSwitch(1); // check to see if user tried to pause simulation
		//if(button1==1) {
		//	pauseSimPressed = !pauseSimPressed;
		//}
	} else if((simulatedSets != NULL) && (currSetDrawing == numValidSets)) {
		 simulationRunning = true;
		 hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
		 tool->start();
		 readyToDraw = false;
	} 
}


//------------------------------------------------------------------------------
// Calculate initial design
//------------------------------------------------------------------------------
struct curveFuncToSolve
{
	vector<cVector3d> *param;

	int operator()(const Eigen::VectorXf &xvec, Eigen::VectorXf &fvec) const 
	{ 
		// convert Eigen::VectorXf to gsl_vector
		gsl_vector *x_gsl = gsl_vector_alloc (4);
		gsl_vector *f_gsl = gsl_vector_alloc (4);
		for(int i=0; i<4; i++) {
			gsl_vector_set (x_gsl, i, xvec(i));
			gsl_vector_set (f_gsl, i, fvec(i));
		}
		curveFunc(x_gsl, (void *)this->param, f_gsl);
		for(int i=0; i<4; i++) {
			// convert gslvector f into fvec (eigen vector)
			fvec(i) = gsl_vector_get (f_gsl,i);
		}
		return 0;
	}

#define EPS 1e-5
	int df(const Eigen::VectorXf &xvec, Eigen::MatrixXf &fjac) const
    {
		Eigen::VectorXf epsilon1(4);
		Eigen::VectorXf epsilon2(4);
		Eigen::VectorXf epsilon3(4);
		Eigen::VectorXf epsilon4(4);
		for(int i=0; i<4; i++) {
			epsilon1(i) = 0;
			epsilon2(i) = 0;
			epsilon3(i) = 0;
			epsilon4(i) = 0;
		}

		epsilon1(0) = EPS;
		epsilon2(1) = EPS;
		epsilon3(2) = EPS;
		epsilon4(3) = EPS;


	    // cx
		Eigen::VectorXf fvecxa(4);
		operator()(xvec + epsilon1, fvecxa);
		Eigen::VectorXf fvecxb(4);
		operator()(xvec - epsilon1, fvecxb);
		Eigen::VectorXf fvecx = (fvecxa-fvecxb)/(2*EPS);

		// cy
		Eigen::VectorXf fvecya(4);
		operator()(xvec + epsilon2, fvecya);
		Eigen::VectorXf fvecyb(4);
		operator()(xvec - epsilon2, fvecyb);
		Eigen::VectorXf fvecy = (fvecya-fvecyb)/(2*EPS);

		// cz
		Eigen::VectorXf fvecza(4);
		operator()(xvec + epsilon3, fvecza);
		Eigen::VectorXf fveczb(4);
		operator()(xvec - epsilon3, fveczb);
		Eigen::VectorXf fvecz = (fvecza-fveczb)/(2*EPS);

		// r
		Eigen::VectorXf fvecra(4);
		operator()(xvec + epsilon4, fvecra);
		Eigen::VectorXf fvecrb(4);
		operator()(xvec - epsilon4, fvecrb);
		Eigen::VectorXf fvecr = (fvecra-fvecrb)/(2*EPS);

		// form jacobian
		fjac.col(0) = fvecx;
		fjac.col(1) = fvecy;
		fjac.col(2) = fvecz;
		fjac.col(3) = fvecr;


		return 0;
	}

	int inputs() const { return 4; }	// inputs is the dimension of x.
	int values() const { return 4; } // "values" is the number of f_i and 

};

void computeTubeParam(ConcentricTubeSet &set, void *circParam) {
	vector<cVector3d> param = *(vector<cVector3d> *)circParam;				// recast
	// Input parameters in circParam: a1,a2,...an,v
	int numPoints = param.size()-1;											// number of points selected
	int numCurves = param.size()-2;											// number of curves (number of tubes)
	
	points = new cVector3d[numPoints];										// set up variable to hold a1 through an
	cVector3d v = param[param.size()-1];									// initial direction vector
	// Allocate space for variables
	rValues = new double[numCurves];										// store computed r values
	cValues = new cVector3d[numCurves];										// store computed c values
	normals = new cVector3d[numCurves];										// store computed normals to each curve
	//----for testing-----
	savedAlphas = new double[nTubes];

	// points a1 to an
	for(int i=0; i<numPoints; i++) {
		points[i] = param[i];
	}

	// Compute radius and center of each curve
	for(int i=0; i<numCurves; i++) {
		//***************************
		//for debugging
		char *str1 = new char[1024];
		sprintf(str1, "v = [%.8f; %.8f; %.8f]", v(0), v(1), v(2));
		OutputDebugString(str1);
		delete str1;
		char *str2 = new char[1024];
		sprintf(str2, "a1 = [%.8f; %.8f; %.8f]", points[i](0),points[i](1),points[i](2));
		OutputDebugString(str2);
		delete str2;
		char *str3 = new char[1024];
		sprintf(str3, "a2 = [%.8f; %.8f; %.8f]", points[i+1](0),points[i+1](1),points[i+1](2));
		OutputDebugString(str3);
		delete str3;
		//*****************************

		v2 = computeCurveParam(set,points[i],points[i+1],v);
		// Save values 
		rValues[i] = rSolved;
		cValues[i] = cSolved;
		// set the newly computed v to be used in next calculation
		v = v2;

	}

	// add code to determine best path and ensure continuous path
	for(int i=0; i<numCurves; i++) {
		// Add curves and save values to new struct
		ConcentricTubeSet::curve newCurve;
		newCurve.r = rValues[i];
		newCurve.c = cValues[i];

		// Compute bx, by, normal
		cVector3d bx_temp = points[i]-cValues[i];
		bx_temp = cNormalize(bx_temp);
		cVector3d tempNorm = cCross(points[i]-cValues[i],points[i+1]-cValues[i]);
		tempNorm = cNormalize(tempNorm);
		cVector3d by_temp = cCross(points[i]-cValues[i], tempNorm);
		by_temp = cNormalize(by_temp);
		// Save bx, by, normal
		newCurve.bx = bx_temp;
		newCurve.by = by_temp;
		newCurve.normVec = tempNorm;

		// Compute theta and curved length
		double numerator = cDot(points[i] - cValues[i], points[i+1] - cValues[i]);
		double denominator = (cDistance(points[i],cValues[i]))*(cDistance(points[i+1],cValues[i]));
		newCurve.theta = acos(numerator/denominator);
		newCurve.lc = rValues[i]*(newCurve.theta);

		// Set tube curved length to length of curve
		set.m_tubes[numCurves-i-1].Lc = newCurve.lc;

		//initialize dir to be 2 (meaningless)
		newCurve.dir = 2;

		set.addCurve(newCurve);

	}

	computeOptPath(set, circParam);



	// find alpha AND curvature for each tube
	set.m_tubes[nTubes-1].alpha = 0;											// assume the outermost tube is at alpha=0
	cVector3d w0 = cCross(param[param.size()-1],set.m_curves[0].normVec);		// create right hand coord sys with initial tangent vector and normal to first curve
	set.m_tubes[0].kappa = 1/rValues[numCurves-1];								// curvature of innermost tube depends on radius of FINAL circle
	
	// curvature calculations
	for(int i=1; i<numCurves; i++) {
		float kappaEq = 1/rValues[numCurves-i-1];
		/*float I1 = (M_PI/64)*((pow(set.m_tubes[i-1].OD,4))-(pow(set.m_tubes[i-1].ID,4)));
		float I2 = (M_PI/64)*((pow(set.m_tubes[i].OD,4))-(pow(set.m_tubes[i].ID,4)));
		set.m_tubes[i].kappa = (kappaEq*(set.m_tubes[i-1].E * I1 + set.m_tubes[i].E * I2) - (set.m_tubes[i-1].E * I1 * set.m_tubes[i-1].kappa))/(set.m_tubes[i].E * I2);
		*/

		float sumNumer1 = 0;
		float sumNumer2 = 0;
		for(int j=0; j<(i+1); j++) {
			float I = (M_PI/64)*((pow(set.m_tubes[j].OD,4))-(pow(set.m_tubes[j].ID,4)));
			sumNumer1 = sumNumer1 + kappaEq*(I*set.m_tubes[j].E);
			//if(j<i) {
			/*if(j==(i-1)) {
				sumNumer2 = sumNumer2 + set.m_tubes[j].kappa*set.m_tubes[j].E*I;
			}*/
		}
		float In = (M_PI/64)*((pow(set.m_tubes[i].OD,4))-(pow(set.m_tubes[i].ID,4)));
		set.m_tubes[i].kappa = (sumNumer1-sumNumer2)/(set.m_tubes[i].E*In);
	}

	// length calculations
	bool exceedsLength = false;
	for(int i=0; i<numCurves; i++) {
		int tubeInd = numCurves-i-1;
		// OLD WAY OF CALCULATING LC
		//if(tubeInd==(nTubes-1)) {			// outermost tube only
		//	set.m_tubes[tubeInd].Lc = set.m_curves[i].lc;
		//} else {
		//	//set.m_tubes[tubeInd].Lc = set.m_curves[i].lc + set.m_tubes[tubeInd+1].Lc;		// Assuming it's possible for inner tube to have curved length of calc length plus length of previous curved lengths of outer tubes
		//	// Try only adding length of calculated curvatures of previous tubes
		//	set.m_tubes[tubeInd].Lc = set.m_curves[i].lc + set.m_curves[i-1].lc;		
		//}
		set.m_tubes[tubeInd].Lc = set.m_curves[i].lc;	// try setting curved length of tube to same length as curved segment (AND NOT ADDING ON LENGTH OF OTHER CURVED SEGMENTS TOO)
		if(tubeInd==(nTubes-1)) {	
			set.m_tubes[tubeInd].Ls = 0.05;//+(0.01*(nTubes-tubeInd));
			set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01;
		} else if(tubeInd==(nTubes-2)) {
			set.m_tubes[tubeInd].Ls = 0.05+0.06+ set.m_tubes[tubeInd+1].Lc; //+(0.01*(nTubes-tubeInd))
			set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01 + set.m_tubes[tubeInd+1].Lc;
		} else {
			set.m_tubes[tubeInd].Ls = 0.05+0.06+0.1+ set.m_tubes[tubeInd+1].Lc+ set.m_tubes[tubeInd+2].Lc;
			set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01 + set.m_tubes[tubeInd+1].Lc + set.m_tubes[tubeInd+2].Lc;
		}
		
		if((set.m_tubes[tubeInd].Lc + set.m_tubes[tubeInd].Ls)>maxLength[set.m_tubes[tubeInd].materialNum]) {
			exceedsLength = true;			
			/*float oldLc = set.m_tubes[tubeInd].Lc;
			set.m_tubes[tubeInd].Lc = maxLength[set.m_tubes[tubeInd].materialNum] - set.m_tubes[tubeInd].Ls;
			set.m_tubes[tubeInd].Beta = set.m_tubes[tubeInd].Beta - (oldLc-(set.m_tubes[tubeInd].Lc));
			printf("Initial design exceeded max length for tube %i by %f \n", tubeInd, oldLc + set.m_tubes[tubeInd].Ls-maxLength[set.m_tubes[tubeInd].materialNum]);
			*/			
		}

		//set.m_tubes[tubeInd].Ls = 0.1+(0.01*(nTubes-tubeInd));			// OLD WAY OF CALCULATING LS
		//set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01;		// OLD WAY
		
		// alpha calculations
		int curveInd = i;
		if(curveInd!=0) {			// do for all tubes EXCEPT outermost (where alpha=0)
			double y = cDot(set.m_curves[curveInd].normVec,w0);											// project normal from each curve onto the plane made by normal0 and w0
			double x = cDot(set.m_curves[curveInd].normVec,set.m_curves[0].normVec);
			set.m_tubes[tubeInd].alpha = atan2(y,x);
		}		
	}

	printf("kappa tube 0: %f \t, alpha tube 0: %f \n",set.m_tubes[0].kappa,set.m_tubes[0].alpha); 
	printf("kappa tube 1: %f \t, alpha tube 1: %f \n",set.m_tubes[1].kappa,set.m_tubes[1].alpha); 
	printf("kappa tube 2: %f \t, alpha tube 2: %f \n",set.m_tubes[2].kappa,set.m_tubes[2].alpha);

	// checking length
	// check to see if total length exceeds the length that can be printed/made
	if(exceedsLength) {
		for(int i=0; i<numCurves; i++) {
			int tubeInd = numCurves-i-1;
			set.m_tubes[tubeInd].Lc = set.m_tubes[tubeInd].Lc - 0.01;
			if(tubeInd==(nTubes-2)) {
				set.m_tubes[tubeInd].Ls = 0.05 + 0.06 + set.m_tubes[tubeInd+1].Lc; 
				set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01 + set.m_tubes[tubeInd+1].Lc;
			} else if(tubeInd==0) {
				set.m_tubes[tubeInd].Ls = 0.05 + 0.06 + 0.1 + set.m_tubes[tubeInd+1].Lc+ set.m_tubes[tubeInd+2].Lc;
				set.m_tubes[tubeInd].Beta = -set.m_tubes[tubeInd].Ls + 0.01 + set.m_tubes[tubeInd+1].Lc + set.m_tubes[tubeInd+2].Lc;
			}
			if((set.m_tubes[tubeInd].Lc + set.m_tubes[tubeInd].Ls)>maxLength[set.m_tubes[tubeInd].materialNum]) {
				exceedsLength = true;
			}
		}
	}

}

cVector3d computeCurveParam(ConcentricTubeSet &set,  cVector3d a1, cVector3d a2, cVector3d v) {
	vector<cVector3d> circParam;
	circParam.push_back(a1);
	circParam.push_back(a2);
	circParam.push_back(v);

	findCurves(circParam);									// compute c and r of circle defined by a1,a2,v

	cVector3d normVec = cCross(a1-cSolved,a2-cSolved);
	normVec = cNormalize(normVec);
	v2 = cCross(normVec,(a2-cSolved));
	v2 = cNormalize(v2);

	//printf("v2: %f %f %f \n",v2(0),v2(1),v2(2));

	circParam.clear();		//test this

	return v2;												// return vector tangent to a2

}

int findCurves(vector<cVector3d> &circParam) {
	//const gsl_multiroot_fsolver_type *T;
	//gsl_multiroot_fsolver *solver;
	//int status;
	//size_t iter = 0;	 		
	//gsl_multiroot_function f = {&curveFunc, 4, &circParam};

	double x_init[4] = {1,1,1,1};
	//gsl_vector *x = gsl_vector_alloc(4);

	//gsl_vector_set(x,0,x_init[0]);
	//gsl_vector_set(x,1,x_init[1]);
	//gsl_vector_set(x,2,x_init[2]);
	//gsl_vector_set(x,3,x_init[3]);



	for(int i=0; i<4; i++) {
		circVec(i) = x_init[i];
	}


	curveFuncToSolve func;
	func.param = &circParam;
	Eigen::LevenbergMarquardt<curveFuncToSolve, float> lm(func);
	lm.minimize(circVec);


	

	printf("cx: %f \n", circVec(0));
	printf("cy: %f \n", circVec(1));
	printf("cz: %f \n", circVec(2));
	printf("r: %f \n", circVec(3));

	cSolved = cVector3d(circVec(0), circVec(1), circVec(2));
	rSolved = circVec(3);



	//T = gsl_multiroot_fsolver_hybrids;
	//solver = gsl_multiroot_fsolver_alloc (T, 4);
	//gsl_multiroot_fsolver_set (solver, &f, x);

	//do
	//	{
	//		iter++;
	//		status = gsl_multiroot_fsolver_iterate (solver);

	//		printf("iter: %i, x: %f  %f  %f %f \n",iter, gsl_vector_get (solver->x, 0), gsl_vector_get (solver->x, 1),gsl_vector_get (solver->x, 2),gsl_vector_get (solver->x, 3));
	//		printf("iter: %i, f: %f  %f  %f %f \n",iter, gsl_vector_get (solver->f, 0), gsl_vector_get (solver->f, 1),gsl_vector_get (solver->f, 2),gsl_vector_get (solver->f, 3));

	//		if (status)   /* check if solver is stuck */
	//			break;

	//		status = gsl_multiroot_test_residual (solver->f, 1e-6);				//
	//	}
	//while (status == GSL_CONTINUE && iter < 1000);

	//printf ("status = %s\n", gsl_strerror (status));

	//cSolved = cVector3d(gsl_vector_get (solver->x, 0), gsl_vector_get (solver->x, 1), gsl_vector_get (solver->x, 2));
	//rSolved = gsl_vector_get (solver->x, 3);

	/*gsl_multiroot_fsolver_free (solver);*/
	//gsl_vector_free (x);

	/*printf("cx: %f \t, cy: %f \t, cz: %f \t, r: %f \n",cSolved(0),cSolved(1),cSolved(2),rSolved);*/

	return 0;

}

int curveFunc(const gsl_vector * x, void *circParam, gsl_vector * f) {

	vector<cVector3d> param = *(vector<cVector3d> *)circParam;				// recast

	cVector3d a1 = param[0];
	cVector3d a2 = param[1];
	cVector3d v = param[2];

	double cx = gsl_vector_get (x, 0);
	double cy = gsl_vector_get (x, 1);
	double cz = gsl_vector_get (x, 2);
	cVector3d c = cVector3d(cx,cy,cz);
	double r = gsl_vector_get (x, 3);

	double y0 = cDistance(a1,c) - r;
	double y1 = cDistance(a2,c) - r;
	double y2 = cDot(v,(a1-c));
	cVector3d crossProd = cCross(cCross(a1-c,a2-c),cCross(v,a1-c));
	double y3 = crossProd.length() ;

	gsl_vector_set (f, 0, y0);
	gsl_vector_set (f, 1, y1);
	gsl_vector_set (f, 2, y2);
	gsl_vector_set (f, 3, y3);


	return GSL_SUCCESS;
}

cVector3d computeEndPoint(cVector3d tubeStartPoint, cVector3d tubeEndPoint, cVector3d curveMidPoint, float lineLength) {
	cVector3d endPointVec = cSub(tubeEndPoint,tubeStartPoint);
	double vecLength = endPointVec.length();
	vecLength = vecLength/2;
	endPointVec.normalize();
	cVector3d lineMidPoint = tubeStartPoint + vecLength*endPointVec;
	cVector3d unitVec = cSub(curveMidPoint,lineMidPoint);
	unitVec.normalize();
	cVector3d lineEndPoint = curveMidPoint + lineLength*unitVec;

	return lineEndPoint;
}

void computeOptPath(ConcentricTubeSet &set, void *circParam) {
	vector<cVector3d> param = *(vector<cVector3d> *)circParam;
	int numCurves = param.size()-2;											// number of curves (number of tubes)
	double theta0;
	double theta1;
	double theta2;
	cVector3d pts0;
	cVector3d pts1;
	cVector3d pts2;
	double av1;
	double av2;

	for(int i=0; i<numCurves; i++) {
		double sum1 = 0;
		double sum2 = 0;
		// compute first 20 points along curve
		for(int j=0; j<20; j++) {
			if(i==0) {		 // if first curve

				// determine direction of first curve
				if(j==0) {
					theta1 = -1*set.m_curves[i].theta;		// testing dir=1
					theta2 = 0;			// testing dir=-1
				} else {
					theta1 = theta1 - 0.01;
					theta2 = theta2 + 0.01;
				}
				pts1 = set.m_curves[i].r * cos(theta1) * set.m_curves[i].bx + sin(theta1)*set.m_curves[i].by + set.m_curves[i].c;
				pts2 = set.m_curves[i].r * cos(theta2) * set.m_curves[i].bx + sin(theta2)*set.m_curves[i].by + set.m_curves[i].c;
				sum1 = sum1 + cDistance(globalTanLine->m_pointB,pts1);
				sum2 = sum2 + cDistance(globalTanLine->m_pointB,pts2);
				/*sum1 = sum1 + cDistance(globalTanVec,pts1);
				sum2 = sum2 + cDistance(globalTanVec,pts2);*/

				printf("tan: %f \t %f \t %f \n", globalTanLine->m_pointB.get(0),globalTanLine->m_pointB.get(1),globalTanLine->m_pointB.get(2));
				printf("pts: %f \t %f \t %f \n", pts1(0),pts1(1),pts1(2));
				printf("pts: %f \t %f \t %f \n", pts2(0),pts2(1),pts2(2));


				//if(set.m_curves[i].dir == 2) {		// first time computing
				//	if(j==19) {
				//		set.m_curves[i].dir = 1;
				//	}
				//} else {
				//	
				//	set.m_curves[i].dir = -1;
				//}
			} else {		// if any other curve, check for which arc to use
				if(set.m_curves[i-1].dir == 1) {			// if direction of previous curve was 1
					if(j==0) {  // initialize theta values
						theta0 = -1*set.m_curves[i-1].theta;
						theta1 = 0;
						theta2 = 0; //2*M_PI - set.m_curves[i].theta;
					} else {
						theta0 = theta0 - 0.01;
						theta1 = theta1 - 0.01;
						theta2 = theta2 + 0.01;
					}
				} else if(set.m_curves[i-1].dir == -1) {	// if direction of previous curve was -1
					if(j==0) {  // initialize theta values
						theta0 = set.m_curves[i-1].theta;
						theta1 = 0;
						theta2 = 0;
					} else {
						theta0 = theta0 + 0.01;
						theta1 = theta1 - 0.01;
						theta2 = theta2 + 0.01;
					}
				}
				// calculate subsequent points along previous curve and first points along both possible arcs
				pts0 = set.m_curves[i-1].r * cos(theta0) * set.m_curves[i-1].bx + sin(theta0)*set.m_curves[i-1].by + set.m_curves[i-1].c;
				pts1 = set.m_curves[i].r * cos(theta1) * set.m_curves[i].bx + sin(theta1)*set.m_curves[i].by + set.m_curves[i].c;
				pts2 = set.m_curves[i].r * cos(theta2) * set.m_curves[i].bx + sin(theta2)*set.m_curves[i].by + set.m_curves[i].c;
				sum1 = sum1 + cDistance(pts0,pts1);
				sum2 = sum2 + cDistance(pts0,pts2);
			}
		}

		if(i==0) {
			if(sum1>sum2){
				set.m_curves[i].dir = 1;
			} else {
				set.m_curves[i].dir = -1;
				set.m_curves[i].theta = 2*M_PI - set.m_curves[i].theta;
			}
		}else {
			av1 = sum1/20;
			av2 = sum2/20;
			if(av1<av2) {
				//theta remains unchanged
				set.m_curves[i].dir = 1;
			} else {
				set.m_curves[i].theta = 2*M_PI - set.m_curves[i].theta;
				set.m_curves[i].dir = -1;
			}
		}
	}

	//for debugging
	char *str1 = new char[1024];
	sprintf(str1, "dir = [%i; %i; %i]", set.m_curves[0].dir, set.m_curves[1].dir, set.m_curves[2].dir);
	OutputDebugString(str1);
	delete str1;
	char *str2 = new char[1024];
	sprintf(str2, "theta = [%.8f; %.8f; %.8f]", set.m_curves[0].theta, set.m_curves[1].theta, set.m_curves[2].theta);
	OutputDebugString(str2);
	delete str2;
}

static void printMatrix(const gsl_matrix *x, const char *path)
{
	FILE *f = fopen(path, "w");
	assert(f != NULL);

    string strt = "[\n";
	fwrite(strt.c_str(), 1, strt.length(), f);
	for(int r=0; r<x->size1; r++) {
		string rowstr = "";
		for(int c=0; c<x->size2-1;c++) {
			rowstr = rowstr + to_string((long double)gsl_matrix_get(x, r, c)) + " ";
		}
		rowstr = rowstr + to_string((long double)gsl_matrix_get(x,r,x->size2-1)) + ";\n";
		fwrite(rowstr.c_str(), 1, rowstr.length(), f);
	}
	string end = "]";
	fwrite(end.c_str(), 1, end.length(), f);
	fclose(f);
}

void drawInitCTR(ConcentricTubeSet &set) {
	cVector3d origPos = tubeStartSphere->getLocalPos();		// save original pos

	float insertPointX = pointSphere[0]->getLocalPos()(0); 
	float insertPointY = pointSphere[0]->getLocalPos()(1); 
	float insertPointZ = pointSphere[0]->getLocalPos()(2);
	updateInsertionPoint(set, insertPointX, insertPointY, insertPointZ);


	//Testing out new code for finding correct initial point:
	//cMatrix3d newRot = tubeStartSphere->getLocalRot();
	//pointSphere[0]->setLocalRot(newRot);
	//float insertPointX = pointSphere[nTubes+1]->getLocalPos()(0);
	//float insertPointY = pointSphere[nTubes+1]->getLocalPos()(1);
	//float insertPointZ = pointSphere[nTubes+1]->getLocalPos()(2);
	//updateInsertionPoint(set, insertPointX, insertPointY, insertPointZ);

	//tubeStartSphere->setFrameSize(0.1);
	//pointSphere[0]->setFrameSize(0.1);
	//tubeStartSphere->setShowFrame(true);
	//pointSphere[0]->setShowFrame(true);


	
	

	//set.m_tubes[nTubes-2].alpha = savedAlphas[nTubes-2];
	//set.m_tubes[nTubes-3].alpha = savedAlphas[nTubes-3];
	runKinematics();				// call this DIRECTLY (rather than in a separate thread) to be sure values are computed before drawing

	if(set.isValidSet) {
		printf("valid set \n");
	} else {
		printf("could not solve \n");
		reinitialize();
		updateInsertionPoint(set,origPos(0),origPos(1),origPos(2));
	}
	printf("second computation done \n");

	// To save parameters for debugging
	char path[150];
	for(int i=0; i<nTubes; i++) {
		sprintf(path, "C:/Users/Tania/Documents/FilesForVisualization/setName%d.txt", i);
		saveTube(path, &set.m_tubes[i]);
	}
	printTube(set);
	printf("finished saving \n");


	if(set.isValidSet) {
		//// Calculations for determining initial orientation
		computeInitOrientation(set);
		drawTubes(set);

		// Set clock to measure time before erasing initial points and tangent line drawn
		otherClock.reset();
		otherClock.start();

		// For now don't erase pointSphere[i] (initial points drawn)
		//for(int i=0; i<(nTubes+1); i++) {
		//	pointSphere[i]->setEnabled(false);	
		//}

		readyToDrawInitCTR = false;
		CTRInitialized = true; 
		hapticsOn = true;

		// ADD ALL HAPTIC EFFECTS TO ANATOMY 
		// Bone
		a_mesh->computeBoundaryBox(true);
		a_mesh->createAABBCollisionDetector(toolRadius);
		a_mesh->setStiffness(0.3*maxStiffness, true);
		// Skin
		skin_mesh->computeBoundaryBox(true);
		skin_mesh->createAABBCollisionDetector(toolRadius);
		skin_mesh->setStiffness(0.3*maxStiffness, true);
		// Left kidney
		kidney_mesh->computeBoundaryBox(true);
		kidney_mesh->createAABBCollisionDetector(toolRadius);
		kidney_mesh->setStiffness(0.3*maxStiffness, true);
		//kidney_mesh->setShowCollisionDetector(true,true);
		// Liver
		liver_mesh->computeBoundaryBox(true);
		liver_mesh->createAABBCollisionDetector(toolRadius);
		liver_mesh->setStiffness(0.3*maxStiffness, true);
		// Stone
		stone_mesh->computeBoundaryBox(true);
		stone_mesh->createAABBCollisionDetector(toolRadius);
		stone_mesh->setStiffness(0.3*maxStiffness, true);
		// Renal pelvis
		renal_pelvis_mesh->computeBoundaryBox(true);
		renal_pelvis_mesh->createAABBCollisionDetector(toolRadius);
		renal_pelvis_mesh->setStiffness(0.3*maxStiffness, true);
		// Right kidney
		right_kidney_mesh->computeBoundaryBox(true);
		right_kidney_mesh->createAABBCollisionDetector(toolRadius);
		right_kidney_mesh->setStiffness(0.3*maxStiffness, true);
	
	}

}

void computeInitOrientation(ConcentricTubeSet set) {
	//*** Calculations for determining initial orientation ***
	int *pointsInSegment;
	pointsInSegment = new int[nTubes];
	double s;
	int tempInd = 0;	
	// need to run through a loop to store start indices for each segment
	for(int k=nTubes-1; k>=0; k--) {		// Start from OUTERMOST tube (tube #n) and going til INNERMOST tube (tube #0) to store tube indices!
		float length = set.m_tubes[k].Ls + set.m_tubes[k].Lc + set.m_tubes[k].Beta;	
		tubeStartInd[k] = tempInd;			// Store index of start of each tube
		s = set.sStored[tempInd];
		while(s<length)
		{
			tempInd = tempInd + 1;
			if(tempInd<(set.sStored.size())) {
				s = set.sStored[tempInd];
			} else {
				s = length;
			}
		}
	}
	// then calculate the number of points in each segment
	for(int i=0; i<nTubes; i++) {				
		if(i==0) {								// innermost tube
			pointsInSegment[i] = set.sStored.size() - tubeStartInd[i];
		} else {								// all other tubes
			pointsInSegment[i] = tubeStartInd[i-1] - tubeStartInd[i];
		}
		//printf("points: %i \n", pointsInSegment[i]);
	}
	//printf("sum: %i \n", pointsInSegment[0]+pointsInSegment[1]+pointsInSegment[2]);
	//printf("tot: %i \n", set.positionStored.x.size());

	// Initialize matrices X1 and X2
	int nCol = set.positionStored.x.size();
	gsl_matrix * X1 = gsl_matrix_alloc (3, nCol);			// allocate 
	gsl_matrix * X2 = gsl_matrix_alloc (3, nCol);			// allocate
	// Form X1: matrix of points of CTR (translated)
	for(int j=0; j<nCol; j++) {
		size_t col = j;

		double xVal = set.positionStored.x[j];
		//double yVal = set.positionStored.y[j] + 0.1;		// translate to center at the origin
		double yVal = set.positionStored.y[j];		
		double zVal = set.positionStored.z[j];

		gsl_matrix_set (X1, 0, col, xVal);
		gsl_matrix_set (X1, 1, col, yVal);
		gsl_matrix_set (X1, 2, col, zVal);
		
	}
	// Form X2: matrix of points along curves
	size_t col = 0;
	int ind = 0;											// keep track of curve number
	cVector3d shiftVec;
	float lastLc;
	for(int k=nTubes-1; k>=0; k--) {	 
		float dist;
		if(k==(nTubes-1)) {
			dist = (set.m_tubes[k].Lc)/pointsInSegment[k];	// distance between points along curve
			lastLc = set.m_tubes[k].Lc;
		} else {
			dist = (set.m_tubes[k].Lc-lastLc)/pointsInSegment[k];	// distance between points along curve
			lastLc = set.m_tubes[k].Lc;
		}

		// trying to divide by pointsInSegment[k]-1 instead....
		//if(k==(nTubes-1)) {
		//	dist = (set.m_tubes[k].Lc)/(pointsInSegment[k]-1);	// distance between points along curve
		//	lastLc = set.m_tubes[k].Lc;
		//} else {
		//	dist = (set.m_tubes[k].Lc-lastLc)/(pointsInSegment[k]-1);	// distance between points along curve
		//	lastLc = set.m_tubes[k].Lc;
		//}
		for(int i=0; i<pointsInSegment[k]; i++) {			
			double theta = (dist*i)/rValues[ind];

			//for debug testing
			double numerator = cDot(points[ind]-cValues[ind],points[ind+1]-cValues[ind]);
			cVector3d vec1 = (points[ind]-cValues[ind]);
			cVector3d vec2 = points[ind+1]-cValues[ind];
			double denom = vec1.length() * vec2.length();
			double thetaTest = acos(numerator/denom);



			cVector3d bx = points[ind]-cValues[ind];
			bx = cNormalize(bx);
			cVector3d tempNorm = cCross(points[ind]-cValues[ind],points[ind+1]-cValues[ind]);
			tempNorm = cNormalize(tempNorm);
			cVector3d by = cCross(points[ind]-cValues[ind], tempNorm);
			by = cNormalize(by);

			if((ind==0) && (k==(nTubes-1)) && (i==0)) {
				shiftVec = rValues[ind]*(cos(theta)*bx + sin(theta)*by) + cValues[ind];
				printf("shiftVec: %f \t %f \t %f \n", shiftVec(0), shiftVec(1), shiftVec(2));
			}
			cVector3d pts = rValues[ind]*(cos(theta)*bx + sin(theta)*by) + cValues[ind] - shiftVec;
			//cVector3d pts = rValues[ind]*(cos(theta)*bx + sin(theta)*by) + cValues[ind];

			gsl_matrix_set (X2, 0, col, pts(0));
			//gsl_matrix_set (X2, 1, col, pts(1) + 0.1);			// translate to center at the origin
			gsl_matrix_set (X2, 1, col, pts(1));			
			gsl_matrix_set (X2, 2, col, pts(2));
			
			col = col + 1;
		}
		ind = ind + 1;
	}

	// Procrustes method 
	gsl_matrix *X2T = gsl_matrix_alloc (nCol, 3);;
	gsl_matrix_transpose_memcpy (X2T, X2);
	//gsl_matrix_mul_elements (X1, X2T);					// multiplied matrix now stored in X1
	gsl_matrix *A = gsl_matrix_alloc(3,3);
	gsl_linalg_matmult (X1, X2T, A);						// A = X1*X2T

	gsl_matrix *V = gsl_matrix_alloc (3, 3);
	gsl_vector *S = gsl_vector_alloc (3);
	gsl_vector * work = gsl_vector_alloc (3);
	gsl_linalg_SV_decomp (A, V, S, work);

	gsl_matrix *R = gsl_matrix_alloc(3,3);
	gsl_linalg_matmult_mod (V, GSL_LINALG_MOD_NONE, A, GSL_LINALG_MOD_TRANSPOSE, R);		// R = V*A'
	

	//cMatrix3d currentRot = tubeStartSphere->getLocalRot();
	cMatrix3d Rot;											// Get best rotation matrix
	Rot = cMatrix3d(gsl_matrix_get(R,0,0),gsl_matrix_get(R,0,1),gsl_matrix_get(R,0,2),
					gsl_matrix_get(R,1,0),gsl_matrix_get(R,1,1),gsl_matrix_get(R,1,2),
					gsl_matrix_get(R,2,0),gsl_matrix_get(R,2,1),gsl_matrix_get(R,2,2));
	//Rot = cTranspose(Rot);
	tubeStartSphere->setLocalRot(Rot);
	//tubeStartSphere->translate(shiftVec);

	printf("rotated \n");

	// For debugging
	printMatrix(X1, "C:/Users/Tania/Documents/FilesForVisualization/X1.dat");
	printMatrix(X2, "C:/Users/Tania/Documents/FilesForVisualization/X2.dat");
	gsl_matrix * X3 = gsl_matrix_alloc (3, nCol);			// allocate 
	gsl_linalg_matmult (R, X1, X3);
	printMatrix(X3, "C:/Users/Tania/Documents/FilesForVisualization/X3.dat");

	printf("done with comp \n");

}

void reinitialize(void) {
	CTRInitialized = false;
	readyToDrawInitCTR = false;
	readyToDraw = false;
	pointNum = 0;
	//updateInsertionPoint(set, -insertPointX, -insertPointY, -insertPointZ);
	//increasePointNum = false;
	for(int i=0; i<10; i++) {
		pointSphere[i]->setLocalPos(0,0,0);
		pointSphere[i]->setEnabled(false);
	}
}

// For debugging convergence problem
void saveTube(const char *path, const ConcentricTubeSet::tube *t)
{
	FILE *f = fopen(path, "wb");
	assert(fwrite((void *)t, 1, sizeof(ConcentricTubeSet::tube), f) == sizeof(ConcentricTubeSet::tube));
	fclose(f);
}

void loadTube(const char *path, ConcentricTubeSet::tube *t)
{
	FILE *f = fopen(path, "rb");
	assert(fread((void *)t, 1, sizeof(ConcentricTubeSet::tube), f) == sizeof(ConcentricTubeSet::tube));
	fclose(f);
}

//void printTube(ConcentricTubeSet::tube *t) {
void printTube(ConcentricTubeSet set) {
	char *str1 = new char[1024];
	sprintf(str1, "alpha = [%.8f; %.8f; %.8f]", set.m_tubes[0].alpha, set.m_tubes[1].alpha, set.m_tubes[2].alpha);
	OutputDebugString(str1);
	delete str1;

	char *str2 = new char[1024];
	sprintf(str2, "Beta = [%.8f; %.8f; %.8f]", set.m_tubes[0].Beta, set.m_tubes[1].Beta, set.m_tubes[2].Beta);
	OutputDebugString(str2);
	delete str2;

	char *str3 = new char[1024];
	sprintf(str3, "Lc = [%.8f; %.8f; %.8f]", set.m_tubes[0].Lc, set.m_tubes[1].Lc, set.m_tubes[2].Lc);
	OutputDebugString(str3);
	delete str3;

	char *str4 = new char[1024];
	sprintf(str4, "Ls = [%.8f; %.8f; %.8f]", set.m_tubes[0].Ls, set.m_tubes[1].Ls, set.m_tubes[2].Ls);
	OutputDebugString(str4);
	delete str4;

	char *str5 = new char[1024];
	sprintf(str5, "kappa = [%.8f; %.8f; %.8f]", set.m_tubes[0].kappa, set.m_tubes[1].kappa, set.m_tubes[2].kappa);
	OutputDebugString(str5);
	delete str5;
}


//char path[150];
//for(i=0; i<set.n_tubes; i++) {
//	sprintf(path, "C:/Users/Tania/Documents/FilesForVisualization/setName%d.txt", i);
//	saveTube(path, &set.m_tubes[i]);
//}
//char path[150];
//for(i=0; i<set.n_tubes; i++) {
//	sprintf(path, "C:/Users/Tania/Documents/FilesForVisualization/setName%d.txt", i);
//	ConcentricTubeSet::tube t;
//	loadTube(path, &t);
//	set.m_tubes[i] = t;
//}



/*---------------------------------------------------------------------------------------------------------------------*/

/////////////////////////////////////////////////////////////////////
///////// CHANGE VALUES HERE TO CHANGE TUBE PARAMETERS //////////////
/////////////////////////////////////////////////////////////////////

// CHANGE VALUES HERE TO CHANGE INITIAL TUBE PARAMETERS
void setTubeParams(ConcentricTubeSet set) {
	// Set value of alpha for each tube
	alpha_init = new float[nTubes];
	alpha_init[0] = M_PI/2;//-2.81074452;		
	alpha_init[1] = -M_PI/2;//0.2627804;		
	alpha_init[2] = 0;

	// Set value of OD for each tube
	OD_init = new float[nTubes];
	OD_init[0] = 0.0018;
	OD_init[1] = 0.0025;
	OD_init[2] = 0.0035; //.35

	// Set value of ID for each tube
	ID_init = new float[nTubes];
	ID_init[0] = 0.0013;
	ID_init[1] = 0.0020;
	ID_init[2] = 0.0030;

	// Set value of E for each tube
	E_init = new float[nTubes];
	E_init[0] = 50000000000;
	E_init[1] = 50000000000;
	E_init[2] = 50000000000;

	// Set value of v for each tube
	v_init = new float[nTubes];
	v_init[0] = 0.33;
	v_init[1] = 0.33;
	v_init[2] = 0.33;

	// Set value of kappa for each tube
	kappa_init = new float[nTubes];
	kappa_init[0] = 10;//46.58657455; //10;
	kappa_init[1] = 10;//56.71429062;//10;
	kappa_init[2] = 9;//43.95024872; //9;

	// Set value of Beta for each tube
	Beta_init = new float[nTubes];
	Beta_init[0] = -0.13; //-.12
	Beta_init[1] = -0.09;//-0.11;//-0.09;
	Beta_init[2] = -0.07;//-0.1;//-0.07;

	// Set value of Lc for each tube
	Lc_init = new float[nTubes];
	Lc_init[0] = 0.08;//0.13625698;
	Lc_init[1] = 0.07;//0.08762831;
	Lc_init[2] = 0.05;//0.04509944;

	// Set value of Ls for each tube
	Ls_init = new float[nTubes];
	Ls_init[0] = 0.14;//0.13;//0.14;
	Ls_init[1] = 0.10;//0.12;//0.10;
	Ls_init[2] = 0.08;//0.11;//0.08;

	// Material name
	materialName = materialNameVec[0];
	maxStrain = maxStrainVec[0];

}

#ifdef USE_LEAP
void checkIndexCollision(void) {
	float numSpheres = set.sStored.size();
	for(int i=0; i<numSpheres; i++) {
		rightCenterSphere->computeLocalInteraction(tube.tubeSphere[i]->getLocalPos(),cVector3d(0,0,0),1);
		if(rightCenterSphere->m_interactionInside) {
			printf("in contact \n");
		}
	}
}
#endif

//------------------------------------------------------------------------------
// Update view
//------------------------------------------------------------------------------
void showDefaultView(cMatrix3d defaultRot, float defaultOff) {
	// Set offset to the default
	oculusOffset = defaultOff;
	// Set rotation of originSphere to the default
	originSphere->setLocalRot(defaultRot);
}

void getClutchOffset() {
	clutchOffset = tool->getDeviceGlobalPos();
	clutchOffsetLocal = tool->getLocalPos();

	//simulationRunning = false;
}

void setShowMaterialLabels() {
	bool visible;
	if(rotateModeOn) {
		visible = true;
	} else {
		visible = false;
	}
	for(int i=0; i<6; i++) {
		materialOptionDisplay[i]->setEnabled(visible);
	}
	for(int i=0; i<3; i++) {
		materialOptionLabel[i]->setEnabled(visible);
	}
}
//------------------------------------------------------------------------------
// Old functions...
//------------------------------------------------------------------------------

#ifdef USE_LEAP
void colorPixels(unsigned int x, unsigned int y) {
	cColorb color;
	color.setRedDarkSalmon();
	for(int i=1; i<20; i++) {
		for(int j=1; j<20; j++) {
			leapIm->setPixelColor(x+i,y+j,color);
		}
	}
}
void showInteractionBox(InteractionBox interactionBox) {
	boxCenter = interactionBox.center();
	boxHeight = interactionBox.height();			// along leap y [mm]
	boxWidth = interactionBox.width();				// along leap x [mm]
	boxDepth = interactionBox.depth();				// along leap z [mm]
	//leapBox = new cShapeBox(boxWidth, boxDepth, boxHeight);
	//leapBox = new cShapeBox(boxWidth, boxHeight, boxDepth);
	/*camera->addChild(leapBox);
	leapBox->setEnabled(true);*/


	cShapeSphere* leftTopCorner;
	cShapeSphere* rightTopCorner;
	cShapeSphere* leftBotCorner;
	cShapeSphere* rightBotCorner;

	leftTopCorner = new cShapeSphere(0.05);
	rightTopCorner = new cShapeSphere(0.05);
	leftBotCorner = new cShapeSphere(0.05);
	rightBotCorner = new cShapeSphere(0.05);

	camera->addChild(leftTopCorner);
	camera->addChild(rightTopCorner);
	camera->addChild(leftBotCorner);
	camera->addChild(rightBotCorner);

	leftTopCorner->setEnabled(true);
	rightTopCorner->setEnabled(true);
	leftBotCorner->setEnabled(true);
	rightBotCorner->setEnabled(true);

	ratioW = screenW/2/boxWidth;
	ratioH = screenH/boxHeight;

	leftTopCorner->setLocalPos(-1.5,-boxWidth/200,boxHeight/200);
	rightTopCorner->setLocalPos(-1.5,boxWidth/200,boxHeight/200);
	leftBotCorner->setLocalPos(-1.5,-boxWidth/200,-boxHeight/200);
	rightBotCorner->setLocalPos(-1.5,boxWidth/200,-boxHeight/200);

	
}
#endif






