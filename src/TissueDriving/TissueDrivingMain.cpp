//#define WIN32

#include "TissueDrivingMain.h"

#define NUM_TUBES 3

// Variables
float Beta_des[NUM_TUBES];
float alpha_des[NUM_TUBES];
float posInit[NUM_TUBES];
float posIncrement[NUM_TUBES];
std::string state;
bool posReached[NUM_TUBES] = {false,false,false};
DeviceNotification *devNote[NUM_TUBES];
bool useNotifyVal[NUM_TUBES] = {true,true,true};
int notifyVal[NUM_TUBES] = {0,1,2};
bool returning = false;
bool velSet[NUM_TUBES] = {false,false,false};
bool posControl = false;

// Functions
void controlRobotPos(CTRControl robot, float deltaPos_des[NUM_TUBES], float deltaRot_des[NUM_TUBES]);


class Timer {
  public:
    Timer() {
      reset();
    }
    /// reset() makes the timer start over counting from 0.0 seconds.
    void reset() {
      unsigned __int64 pf;
      QueryPerformanceFrequency( (LARGE_INTEGER *)&pf );
      freq_ = 1.0 / (double)pf;
      QueryPerformanceCounter( (LARGE_INTEGER *)&baseTime_ );
    }
    /// seconds() returns the number of seconds (to very high resolution)
    /// elapsed since the timer was last created or reset().
    double seconds() {
      unsigned __int64 val;
      QueryPerformanceCounter( (LARGE_INTEGER *)&val );
      return (val - baseTime_) * freq_;
    }
    /// seconds() returns the number of milliseconds (to very high resolution)
    /// elapsed since the timer was last created or reset().
    double milliseconds() {
      return seconds() * 1000.0;
    }
  private:
    double freq_;
    unsigned __int64 baseTime_;
};

Timer t;
double StartTime = 0;
double CurrentTime = 0;
bool recording = false;

struct data {
	double timeStamp;
	double ForceData[3];
	double arcLength[3];
};

int main()
{
	/////////////////////////////////////////////////////////////////////
	// SETUP
	/////////////////////////////////////////////////////////////////////
	double ForceData[3];
	std::vector<data> storedData;
	data timeAndForce;

	//Set up file for storing data
	std::ofstream myFile("ForceTest.txt");

	//Set up force sensor 
	cForceSensor g_ForceSensor;
	g_ForceSensor.Set_Calibration_File_Loc("C:/Users/Tania/Desktop/Force_Sensor_2010/Debug/FT5904.cal");
	g_ForceSensor.Initialize_Force_Sensor("Dev2/ai0:5");
	Sleep(2000);
	g_ForceSensor.Zero_Force_Sensor();
	Sleep(2000);

	//Start time for the first loop through
	StartTime = t.seconds();

	// Start Motion Controllers
	robot.Init();
	posReached[0] = false;
	posReached[1] = false;
	posReached[2] = false;
	for(int i=0; i<NUM_TUBES; i++) {
		devNote[i] = new DeviceNotification(useNotifyVal[i],notifyVal[i]);
		//devNote[i]->SetNotification(useNotifyVal[i],notifyVal[i]);
	}


	/////////////////////////////////////////////////////////////////////
	// MAIN LOOP
	/////////////////////////////////////////////////////////////////////
	for (;;)
	{
		/////////////////////////////////////////////////////////////////////
		// FOR FORCE SENSING
		/////////////////////////////////////////////////////////////////////
		int i = g_ForceSensor.AcquireFTData();
		g_ForceSensor.GetForceReading(ForceData);
		CurrentTime = t.seconds();
		printf("ForceZ: %f\t Time: %f\r", ForceData[2], CurrentTime-StartTime);
		//printf("Time: %f\t Force1: %f\t Force2: %f\t Force3: %f\r" , CurrentTime-StartTime, ForceData[0], ForceData[1], ForceData[2]);

		if(recording && (CurrentTime-StartTime > 0.002)) {
			timeAndForce.timeStamp = CurrentTime;
			timeAndForce.ForceData[0] = ForceData[0];
			timeAndForce.ForceData[1] = ForceData[1];
			timeAndForce.ForceData[2] = ForceData[2];
			/*timeAndForce.arcLength[0] = robot.m_tubeControllers[0].transController->GetPosition();
			timeAndForce.arcLength[1] = robot.m_tubeControllers[1].transController->GetPosition();
			timeAndForce.arcLength[2] = robot.m_tubeControllers[2].transController->GetPosition();*/
			timeAndForce.arcLength[0] = robot.m_tubeControllers[0].transController->GetMM(robot.m_tubeControllers[0].transController->devParams);
			timeAndForce.arcLength[1] = robot.m_tubeControllers[1].transController->GetMM(robot.m_tubeControllers[1].transController->devParams);
			timeAndForce.arcLength[2] = robot.m_tubeControllers[2].transController->GetMM(robot.m_tubeControllers[2].transController->devParams);
			storedData.push_back(timeAndForce);	//put data into vector
			
			StartTime = t.seconds();	//start time over again
		}	

		/////////////////////////////////////////////////////////////////////
		// FOR MOTION CONTROL
		/////////////////////////////////////////////////////////////////////
		// State machine to control which tubes to insert
		if(recording) {		
			if(posReached[2] && posReached[1] && posReached[0]) {
				// Reset back
				posReached[0] = false;
				posReached[1] = false;
				posReached[2] = false;
				for(int i=0; i<NUM_TUBES; i++) {
					posInit[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
					/*if(!posControl) {
						robot.m_tubeControllers[i].transController->EnableDevice();
					}*/
				}
				// Change states
				if(state=="insertAllTubes"){
					state = "insertTwoTubes";
					posIncrement[0] = 10;
					posIncrement[1] = 10;
					posIncrement[2] = 0;
					if(!posControl) {
						robot.m_tubeControllers[0].transController->EnableDevice();
						robot.m_tubeControllers[1].transController->EnableDevice();
					}
				} else if(state=="insertTwoTubes") {
					state = "insertOneTube";
					posIncrement[0] = 10;
					posIncrement[1] = 0;
					posIncrement[2] = 0;
					if(!posControl) {
						robot.m_tubeControllers[0].transController->EnableDevice();
					}
				} else if(state=="insertOneTube") {
					state = "done";
				}
			}
			/////////////////////////////////////////////////////////////////////
			// FOR POSITION CONTROL
			/////////////////////////////////////////////////////////////////////
			if(posControl) {
				// Move tubes
				for(int i=0; i<NUM_TUBES; i++) {
					Beta_des[i] = posInit[i] + posIncrement[i];	// converted to mm
					controlRobotPos(robot, Beta_des, alpha_des);
				}
			} else {
			/////////////////////////////////////////////////////////////////////
			// FOR VELOCITY CONTROL
			/////////////////////////////////////////////////////////////////////
				for(int i=0; i<NUM_TUBES; i++) {
					Beta_des[i] = posInit[i] + posIncrement[i];
					// check current pos
					float currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
					if((abs(Beta_des[i])-abs(currPosMM))<0.005) {		// reached desired pos
						posReached[i] = true;
						robot.m_tubeControllers[i].transController->Stop();
						velSet[i] = false;
					} else if(velSet[i]!=true) {
						if(i==0) {
							//robot.m_tubeControllers[i].transController->ChangeMaxMotorVelocity(100);
							robot.m_tubeControllers[i].transController->SetVelocity(1000);
							velSet[i] = true;
						} else {
							//robot.m_tubeControllers[i].transController->ChangeMaxMotorVelocity(5);
							robot.m_tubeControllers[i].transController->SetVelocity(400);
							velSet[i] = true;
						}
					}
				}
			}



			/////////////////////////////////////////////////////////////////////
			// FOR SAVING DATA AND QUITTING
			/////////////////////////////////////////////////////////////////////
			// Check for completion
			if(state=="done") {
				printf("Stop recording \n");
				recording = false;
				//Write data to file
				for (int i = 1; i < storedData.size(); i++)
				{
					myFile << "timeStamp: " << storedData[i].timeStamp << "\t" << "\t";
					myFile << "forceX: " << storedData[i].ForceData[0] << "\t" << "\t";
					myFile << "forceY: " << storedData[i].ForceData[1] << "\t" << "\t";
					myFile << "forceZ: " << storedData[i].ForceData[2] << "\t" << "\t";
					myFile << "sTube0: " << storedData[i].arcLength[0] << "\t" << "\t";
					myFile << "sTube1: " << storedData[i].arcLength[1] << "\t" << "\t";
					myFile << "sTube2: " << storedData[i].arcLength[2] << "\n";
				}
				printf("Writing data to file.\n");
				// Stop motion control
				for(int i=0; i<NUM_TUBES; i++) {
					robot.m_tubeControllers[i].transController->Stop();
				}
				//Close file and quit
				myFile.close();
				printf("File closed\n");
			}
		/////////////////////////////////////////////////////////////////////
		// FOR RETURNING TO ORIGNIAL POSITION
		/////////////////////////////////////////////////////////////////////
		} else if(returning) {
			if(posReached[0] && posReached[1] && posReached[2]) {
				// Reset back
				posReached[0] = false;
				posReached[1] = false;
				posReached[2] = false;
				for(int i=0; i<NUM_TUBES; i++) {
					posInit[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
				}
				// Change states
				if(state=="insertAllTubes"){
					state = "insertTwoTubes";
					posIncrement[0] = -10;
					posIncrement[1] = -10;
					posIncrement[2] = 0;
					//Sleep(3000);
				} else if(state=="insertTwoTubes") {
					state = "insertOneTube";
					posIncrement[0] = -10;
					posIncrement[1] = 0;
					posIncrement[2] = 0;
					//Sleep(3000);
				} else if(state=="insertOneTube") {
					state = "done";
				}
			}
			// Move tubes
			for(int i=0; i<NUM_TUBES; i++) {
				Beta_des[i] = posInit[i] + posIncrement[i];	// converted to mm
				controlRobotPos(robot, Beta_des, alpha_des);
			}
			if(state=="done") {
				// Stop motion control
				for(int i=0; i<NUM_TUBES; i++) {
					robot.m_tubeControllers[i].transController->Stop();
				}
			}
		}
		/////////////////////////////////////////////////////////////////////
		// FOR KEY SELECTION
		/////////////////////////////////////////////////////////////////////
		if(_kbhit()) {
			int key = _getch();
			switch (key) {
				case 's':
				case 'S':
					//Start or stop recording data
					if(recording){
						//printf("Stop recording \n");
						//recording = false;
						////Write data to file
						//for (int i = 1; i < storedData.size(); i++)
						//{
						//	myFile << "timeStamp: " << storedData[i].timeStamp << "\t" << "\t";
						//	myFile << "forceX: " << storedData[i].ForceData[0] << "\t" << "\t";
						//	myFile << "forceY: " << storedData[i].ForceData[1] << "\t" << "\t";
						//	myFile << "forceZ: " << storedData[i].ForceData[2] << "\n";
						//}
						//printf("Writing data to file.\n");
						//// Stop motion control
						//for(int i=0; i<NUM_TUBES; i++) {
						//	robot.m_tubeControllers[i].transController->Stop();
						//}
					} else if (!recording) {
						recording = true;
						printf("Start recording \n");
						// Start motion control
						for(int i=0; i<NUM_TUBES; i++) {
							posInit[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
							Beta_des[i] = posInit[i];
							robot.m_tubeControllers[i].transController->EnableDevice();

							alpha_des[i] = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
							robot.m_tubeControllers[i].rotController->Stop();	
						}
						// Set position increment
						state = "insertAllTubes";
						posIncrement[0] = 10;
						posIncrement[1] = 10;
						posIncrement[2] = 10;
						//for(int i=0; i<NUM_TUBES; i++) {
						//	Beta_des[i] = posInit[i] + posIncrement[i];	// converted to mm
						//}
					}
					break;

				case 'r':	// return to original setup
					returning = true;
					// Start motion control
					for(int i=0; i<NUM_TUBES; i++) {
						posInit[i] = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);
						Beta_des[i] = posInit[i];
						robot.m_tubeControllers[i].transController->EnableDevice();

						alpha_des[i] = robot.m_tubeControllers[i].rotController->GetAngle(robot.m_tubeControllers[i].rotController->devParams);
						robot.m_tubeControllers[i].rotController->Stop();	
					}
					// Set position increment
					state = "insertAllTubes";
					posIncrement[0] = -10;
					posIncrement[1] = -10;
					posIncrement[2] = -10;
					break;

				case 'q':
				case 'Q':
					//Close file and quit
					myFile.close();
					printf("File closed\n");
					break;
			}
		}
	}

	return 0;
}

// For controlling robot pos
void controlRobotPos(CTRControl robot, float desPos[NUM_TUBES], float desRot[NUM_TUBES]) {
	for(int i=0; i<NUM_TUBES; i++){
		float currPosMM = robot.m_tubeControllers[i].transController->GetMM(robot.m_tubeControllers[i].transController->devParams);

		// Move insertion motor
		if(desPos[i]!=currPosMM) {
			float pos_desMM = desPos[i];
			// Convert desired pos from mm to encoder ticks
			int pos_des = robot.m_tubeControllers[i].transController->ConvertMMToPosition(pos_desMM, robot.m_tubeControllers[i].transController->devParams);
			// Set position of insertion motor
			robot.m_tubeControllers[i].transController->SetPosition(pos_des,devNote[i]);
		} 

		// Check to see if got close to des pos
		if((abs(desPos[i])-abs(currPosMM)) < 0.02) {
			posReached[i] = true;
		}
		/*long timeOut = 10;
		if(robot.m_tubeControllers[i].transController->IsPositionVerified(timeOut)){
			posReached[i] = true;
		}*/
	}
}