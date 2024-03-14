#ifndef globalVariables_h
#define globalVariables_h

// Include necessary libraries
#include <DetectionsBuffer.h>

//Code integration Variables
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

    // Global Variables for robot states
    inline RobotState prevState = WAIT_FOR_START;
    inline RobotState currentState = WAIT_FOR_START;
    inline int Follow_Line_Counter = 0;

    // Buffer size definitions
    static const int STRING_SIZE = 512;
    static const int MAX_CLASSNAME_LENGTH = 15;
    static const int MAX_DIRECTION_LENGTH = 6;

    inline char dataBuffer[STRING_SIZE];
    inline unsigned long previousMillis = 0;
    inline const long interval = 10000;

    // Debugging
    inline const int debugPin = 10;
    inline bool debugMode = false;
    inline volatile bool printDebugFlag = false;

    // Emergency Stop Pin
    inline const int emergencyStopPin = 2;

    // Group Events
    static const int BIT_NEW_DATA_AVAILABLE = (1 << 0);
    static const int BIT_READ_DETECTIONS = (1 << 1);


    //RoboDriver
    inline const int MaxAngle = 90;
    inline const int BaseSpeed = 125;
    inline const int MotorRunDuration = 500;

    //GyroVariables 
    #define SERIAL_PORT Serial
    #define SPI_PORT SPI 
    #define CS_PIN 2     
    #define WIRE_PORT Wire 
    #define AD0_VAL 1  


//PixyLineDetectionVariables 
    // Constants for Pixy Line Detection
    inline const int CAM_ANG = 20;
    inline const int VER_FOV = 40;
    inline const int HOR_PIXEL = 78;
    inline const int VER_PIXEL = 51;
    inline const int CAM_HEIGHT = 200;

    // Member variables initialization
    inline double x0 = 0;
    inline double y0 = 0;
    inline double x1 = 0;
    inline double y1 = 0;


//TimeManagement Variables 
    // Global Time Management variables
    inline long _startTime;
    inline long _maxTime = 105000;
    inline long _stateTimeLimits[19] = { 0, 10000, 11000, 7000, 4000, 3000, 2000, 3000, 5000, 3000, 10000, 5000, 16000, 5000, 16000, 2000, 3000, 0, 0 };
    inline int _statePriorities[19] = { 0, 6, 7, 5, 3, 2, 1, 2, 4, 2, 6, 4, 8, 4, 8, 1, 2, 0, 0 };
    inline int _numStates = 19;
    inline int _currentState;
    inline long _stateStartTime;
    inline long _stateEndTime;
    inline long _currentstateMaxTime;
    inline long _timeBank;
    inline const long STATE_SWITCH_BUFFER = 500;
    inline bool _isStartStateCompleted = false;
    inline const int WAIT_FOR_START_STATE_ID = 0;

// MotorDriver Variables {
    // Constants for Motor Driver
#define RIGHT_FORWARD_PIN 5
#define RIGHT_BACKWARD_PIN 4
#define LEFT_FORWARD_PIN 3
#define LEFT_BACKWARD_PIN 2
#define RAMP_RATE 2 

// Motor Driver global variables
    inline int OS = 0;
    inline int leftSpeed = 0;
    inline int rightSpeed = 0;
    inline int targetLeftSpeed = 0;
    inline int targetRightSpeed = 0;

// DetectionsBuffer Variables 
    // Constants for Detections Buffer
    inline const int BUFFER_SIZE = 15;
    // Buffer to store detections
    inline Detection buffer[BUFFER_SIZE];
    // Current index in the buffer
    inline int bufferIndex;


//Lidar Variables {
    // Enum for serial select
    enum SerialSelect {
        S0,
        S1,
        S2,
        S3
    };
#endif
