#include "LeapListener.h"

#include <iostream>
#include <string.h>
#include "Leap.h"
//#include "chai3d.h"

using namespace chai3d;
using namespace Leap;

LeapListener::LeapListener(void)
{
}

void LeapListener::onInit(const Controller& controller) {
  std::cout << "Initialized" << std::endl;
}

void LeapListener::onConnect(const Controller& controller) {
  std::cout << "Connected" << std::endl;
  //controller.enableGesture(Gesture::TYPE_SWIPE);
}

void LeapListener::onFrame(const Controller& controller) {  
  double t = static_cast<double>(controller.frame().timestamp()) * 1e-6;
  frame = controller.frame();
  //std::cout << "new frame" << std::endl;
  //pHandList = frame.hands();
  //handCount = pHandList.count();
  //printf("hand count: %i \n",handCount);


  camToOculusRot = cMatrix3d(0,-1,0,1,0,0,0,0,1);
  for (int i = 0; i < frame.hands().count(); i++) {
		pHand = frame.hands()[i];														// get hands in frame
		handID = pHand.id();															// get hand ID
		handCenterTemp = pHand.palmPosition();											// temporary pos of center of hand
		handCenter = cVector3d(handCenterTemp.x,handCenterTemp.y,handCenterTemp.z);		// pos of center of hand in Leap coord frame
		handCenter = camToOculusRot*handCenter;											// pos of center of hand in CAMERA coord frame

		//printf("hand center x: %f, hand center y: %f, hand center z: %f \n",handCenter(0), handCenter(1),handCenter(2));
		/*for (int j = 0; j < 5; j++) {
			pFinger = pHand.fingers()[j];
		}*/
   }

}

//void LeapListener::updateHandGraphics(cVector3d handCenter) {
//	handCenterSphere->setLocalPos(handCenter);
//}

LeapListener::~LeapListener(void)
{
}
