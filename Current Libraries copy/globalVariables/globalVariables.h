#ifndef globalVariables_h
#define globalVariables_h

#include <DetectionsBuffer.h>
//Variables from IntegratedCode

// Enum for different robot states
enum RobotState {
	WAIT_FOR_START,
	GET_BIG_BOXES,
	GET_SMALL_BOXES,
	DEPOSIT_BIG_BOXES,
	DEPOSIT_SMALL_BOXES,
	FOLLOW_LINE, // Has Counter
	GO_TO_RED_ZONE,
	GO_TO_BLUE_ZONE,
	GO_TO_GREEN_ZONE,
	GET_ROCKETS,
	CROSS_GAP,
	DEPOSIT_ROCKETS,
	PUSH_BUTTON,
	DISPLAY_LOGO,
	DONE,
	EMERGENCY_STOP
};

// RobotState Vals
inline RobotState prevState = WAIT_FOR_START;
inline RobotState currentState = WAIT_FOR_START;
inline int Follow_Line_Counter = 0;

// Obj Detection Variables
// Buffer size definitions - because apparently, memory allocation is still a thing. I miss Python.
#define STRING_SIZE 512 // Maximum size of String that can be passed from Jetson. About the size of an ancient Scroll.
#define MAX_CLASSNAME_LENGTH 15 // Maximum size of the class name char array
#define MAX_DIRECTION_LENGTH 6 // Maximum size of the direction char array. Left or Right, not much philosophy here

inline char dataBuffer[STRING_SIZE];
inline unsigned long previousMillis = 0;  // Stores the last time a request was made
inline const long interval = 10000;

// Debug mode - because apparently we love living on the edge
inline const int debugPin = 10; // The "Oh no, what did I break now?" pin
inline bool debugMode = false; // Schrödinger's debug mode
inline volatile bool printDebugFlag = false;

// RTOS Vals
// Emergency Stop Pin
// Emergency Stop Pin - The "Oh no, everything's on fire" button
inline const int emergencyStopPin = 2; // Example pin

// Event Group for Detections
#define BIT_NEW_DATA_AVAILABLE (1 << 0)
#define BIT_READ_DETECTIONS    (1 << 1)


//Variables for RoboDriver
// Objects for motor control and sensors
inline const int MaxAngle = 90;        // Maximum angle for turning
inline const int BaseSpeed = 125;      // Base speed for the motors
inline const int MotorRunDuration = 500; // Duration for running the motors in milliseconds

//Variables for Gyro
#define SERIAL_PORT Serial
#define SPI_PORT SPI // Your desired SPI port. Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define WIRE_PORT Wire // Your desired Wire port. Used when "USE_SPI" is not defined
#define AD0_VAL 1    // AD0 value


//Variables for pixy line detection
#define CAM_ANG 20  // camera angel relative to the plane where the line at
#define VER_FOV 40  // vertical fov of camera

#define HOR_PIXEL 78  // horizontal pixel
#define VER_PIXEL 51  // vertical pixel

#define CAM_HEIGHT 200  // height of camera from ground in milimeter

//Variables for Time Management
// WAIT_FOR_START ID == 0. It does not count toward timer.
inline long _startTime;
inline long _maxTime = 105000;
// With Buffer State Limits
//long _stateTimeLimits[19] = { 0, 10000, 10000, 5000, 4000, 1500, 2000, 1500, 5000, 3000, 10000, 5000, 15000, 5000, 15000, 2000, 3000, 0, 0 }; 
// Without Buffer State Limits
inline long _stateTimeLimits[19] = { 0, 10000, 11000, 7000, 4000, 3000, 2000, 3000, 5000, 3000, 10000, 5000, 16000, 5000, 16000, 2000, 3000, 0, 0 };
// Without Buffer State Limits - Timout Test
//long _stateTimeLimits[19] = { 0, 1000, 1100, 700, 400, 300, 200, 300, 500, 300, 1000, 500, 1600, 500, 1600, 200, 3000, 0, 0 }; 
inline int _statePriorities[19] = { 0, 6, 7, 5, 3, 2, 1, 2, 4, 2, 6, 4, 8, 4, 8, 1, 2, 0, 0 };
inline int _numStates = 19;
inline int _currentState;
inline long _stateStartTime;
inline long _stateEndTime;
inline long _currentstateMaxTime;
inline long _timeBank;
inline const long STATE_SWITCH_BUFFER = 500; // 0.5 seconds in milliseconds
inline bool _isStartStateCompleted = false;
inline const int WAIT_FOR_START_STATE_ID = 0;

inline bool timeOut = false;

//Variables for MotorDriver
// motor pins
#define RIGHT_FORWARD_PIN 5
#define RIGHT_BACKWARD_PIN 4
#define LEFT_FORWARD_PIN 3
#define LEFT_BACKWARD_PIN 2

#define RAMP_RATE 2 

inline int OS = 0;
inline int leftSpeed = 0;
inline int rightSpeed = 0;

inline int targetLeftSpeed = 0;
inline int targetRightSpeed = 0;

//Variables for DetectionsBuffer

// Global variables - Buffer
inline const int BUFFER_SIZE = 15; // Size of the buffer to store detections
inline Detection buffer[BUFFER_SIZE]; // Buffer to store detections
inline int bufferIndex; // Current index in the buffer
#endif
