#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <event_groups.h>
#include <DetectionsBuffer.h>
#include <Dictionary.h>
#include <SoftwareSerial.h>
#include "RobotStateController.h"




/**
 IMPORTANT NOTICE! (Yes, this is actually important)
 Seriously read this. If you don't read and follow this step your code will not compile.
 If you ask me about it and I find you did not read this I will laugh at you.

 For this code to run you need to go to FreeRTOSConfig.h in the FreeRTOS directory in your
 Arduino Libraries Directory, normally located in the Documents folder of Windows Machines,
 Documents/Arduino/libraries/FreeRTOS/FreeRTOSConfig.h and add this line to the 
 #define INCLUDE Statements:

 #define INCLUDE_eTaskGetState               	1

 I usually put it under the group beneath this comment:
 Set the following definitions to 1 to include the API function, or zero
 to exclude the API function.

 Now your code will compile. Good Job.

 --Jordan

**/


/**
 Note to everyone, this is the syntax for creating tasks

 BaseType_t xTaskCreate(TaskFunction_t pvTaskCode,
                       const char * const pcName,
                       unsigned short usStackDepth,
                       void *pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t *pvCreatedTask);

**/



RobotStateController stateMachine;

// Obj Detection Variables
// Buffer size definitions - because apparently, memory allocation is still a thing.
#define BUFFER_SIZE 512 // Maximum size of String that can be passed from Jetson. About the size of an ancient Scroll.
#define MAX_CLASSNAME_LENGTH 15 // Maximum size of the class name char array
#define MAX_DIRECTION_LENGTH 6 // Maximum size of the direction char array. Left or Right, not much philosophy here

char dataBuffer[BUFFER_SIZE];
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;  

Dictionary &class_names_dict = *(new Dictionary(11));
Dictionary &dir_names_dict = *(new Dictionary(2));
Dictionary &class_names_rev = *(new Dictionary(11));


// Debug mode - because apparently we love living on the edge
const int debugPin = 10; // The "Oh no, what did I break now?" pin
bool debugMode = false; // Schrödinger's debug mode
volatile bool printDebugFlag = false;


// RTOS Vals
// Emergency Stop Pin - The "Oh no, everything's on fire" button
const int emergencyStopPin  = 2; // Example pin


// Task Handlers - Like employees, but they never ask for a raise
TaskHandle_t MotorBoxTaskHandle;
TaskHandle_t SensorBoxTaskHandle;
TaskHandle_t debugTaskHandle;

// Mutex for RobotState
SemaphoreHandle_t stateMutex;
// Buffer Mutex
SemaphoreHandle_t bufferMutex;
// Serial Mutex
SemaphoreHandle_t serialMutex;

// Event Group for Detections


// Tasks
void MotorBoxStateManagement(void *pvParameters);
void SensorBox(void *pvParameters);
void readDetTask(void *pvParameters);
void processDetTask(void *pvParameters);
void DebugBox(void *pvParameters);



void setup() {
  // Let's set up our circus of tasks and hope they play nice together
  Serial.begin(9600);
  Serial2.begin(9600);
  stateMachine.init();
  clearBuffer();
  String dir_json = "{\"0\": \"left\", \"1\": \"right\"}";
  String class_json = "{\"0\": \"BigBox\", \"1\": \"BlueZone\", \"2\": \"Button\", \"3\": \"GreenZone\", \"4\": \"Nozzle\", \"5\": \"RedZone\", \"6\": \"Rocket\", \"7\": \"SmallBox\", \"8\": \"StartZone\", \"9\": \"WhiteLine\", \"10\": \"YellowLine\"}";
  String class_rev =  "{\"BigBox\": \"0\", \"BlueZone\": \"1\", \"Button\": \"2\", \"GreenZone\": \"3\", \"Nozzle\": \"4\", \"RedZone\": \"5\", \"Rocket\": \"5\", \"SmallBox\": \"7\", \"StartZone\": \"8\", \"WhiteLine\": \"9\", \"YellowLine\": \"10\"}";
  dir_names_dict.jload(dir_json);
  class_names_dict.jload(class_json);
  class_names_rev.jload(class_rev);

  // Create a mutex for state variable and serial
  stateMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();

  pinMode(emergencyStopPin, INPUT_PULLUP); // Set as input with pull-up
  pinMode(debugPin, INPUT_PULLUP); // Set debug pin as input with pull-up
  debugMode = (digitalRead(debugPin) == LOW); // Check if the pin is LOW (switch closed)
  Serial.println(debugMode);
  
  // Attach interrupts because we're fancy like that
  attachInterrupt(digitalPinToInterrupt(emergencyStopPin), emergencyStopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(debugPin), debugModeISR, FALLING);
  // When in doubt, just stop everything. It's the grown-up version of closing your eyes.


  stateMachine.xDetectionsEventGroup = xEventGroupCreate();

  Serial.println("Setup");
  // Creating our cast of tasks - it's like a talent show, but with more crashing
  xTaskCreate(MotorBoxStateManagement, "MotorBoxStateManagement", 1000, NULL, 3, &MotorBoxTaskHandle);
  xTaskCreate(SensorBox, "SensorBox", 128, NULL, 4, &SensorBoxTaskHandle);
  xTaskCreate(readDetTask, "readDetTask", 500, NULL, 2, &stateMachine.readDetTaskHandle);
  xTaskCreate(processDetTask, "processDetTask", 500, NULL, 1, &stateMachine.processDetTaskHandle);

  if (debugMode) {
    xTaskCreate(DebugBox, "DebugBox", 200, NULL, 5, &debugTaskHandle);
   }
  
  
  Serial.println("Setup 2");


  // For those who like to live dangerously: Uncomment these lines at your own risk.
  // I mean, what could possibly go wrong?
  // If you choose this ill-advised path comment out the other set of tasks.
  // Don't create both. you will break things.
  // if (debugMode) {
  //   xTaskCreate(SensorBox, "SensorBox", 1000, NULL, 5, &SensorBoxTaskHandle);
  //   xTaskCreate(readDetTask, "readDetTask", 1000, NULL, 4, &readDetTaskHandle);
  //   xTaskCreate(processDetTask, "processDetTask", 1000, NULL, 3, &processDetTaskHandle);
  //   xTaskCreate(MotorBoxStateManagement, "MotorBoxStateManagement", 128, NULL, 2, &MotorBoxTaskHandle);
  //   xTaskCreate(DebugBox, "DebugBox", 200, NULL, 1, &debugTaskHandle);
  //  } else {
  //     xTaskCreate(SensorBox, "SensorBox", 1000, NULL, 4, &SensorBoxTaskHandle);
  //     xTaskCreate(readDetTask, "readDetTask", 1000, NULL, 3, &readDetTaskHandle);
  //     xTaskCreate(processDetTask, "processDetTask", 1000, NULL, 2, &processDetTaskHandle);
  //     xTaskCreate(MotorBoxStateManagement, "MotorBoxStateManagement", 128, NULL, 1, &MotorBoxTaskHandle);
  //  }
  

  vTaskSuspend( stateMachine.readDetTaskHandle );
  vTaskSuspend( stateMachine.processDetTaskHandle );

}



// LEAVE THIS EMPTY. NO TOUCHING. AT ALL. UNDER ANY CIRCUMSTANCES. JUST DON'T DO IT.
// WITH THE WAY THIS CODE IS SET UP WE WILL NEVER REACH THIS SECTION OF THE CODE.
// -- Jordan
void loop() {
  // LEAVE THIS EMPTY. This is sacred ground. Trespassers will be turned into pseudo code.

}


// MotorBoxStateManagement - Because who needs a driver when you have a function?
void MotorBoxStateManagement(void *pvParameters) {

    for (;;) {
        Serial.println("Hi Motorbox");
        // Check if the current state is different from the previous state
        if (stateMachine.getCurrentState() != stateMachine.getPrevState()) {
            xSemaphoreTake(bufferMutex, portMAX_DELAY);
            clearBuffer();
            xSemaphoreGive(bufferMutex);
        }

        stateMachine.update();  // Update the state machine

    }
}


// SensorBox - because we're sensing more than just disappointment
void SensorBox(void *pvParameters){
  for (;;) {
    if (stateMachine.getCurrentRobotState() != DONE || stateMachine.getCurrentRobotState() != EMERGENCY_STOP) {
      // Serial.println("Sensor Task");
      vTaskDelay(3000 / portTICK_PERIOD_MS); // On for 3 seconds

    }
  }
}

  

// readDetTask: Reading is fundamental, even for robots. Especially when it involves detecting stuff.
void readDetTask(void *pvParameters) {
    for (;;) { // Infinite loop for the task
      EventBits_t uxBits = xEventGroupWaitBits(
              stateMachine.xDetectionsEventGroup,
              BIT_READ_DETECTIONS,
              pdTRUE,    // Clear BIT_READ_DETECTIONS on exit.
              pdFALSE,   // Wait for just BIT_READ_DETECTIONS.
              portMAX_DELAY); // Wait indefinitely.
          if (stateMachine.getCurrentRobotState() != DONE || stateMachine.getCurrentRobotState() != EMERGENCY_STOP) {
            if ((uxBits & BIT_READ_DETECTIONS) != 0) {
              
              vTaskDelay(pdMS_TO_TICKS(50));
              // Serial.println("RequestDetTask");
              Serial2.println("REQUEST");
              // Wait for a response with a timeout
              unsigned long startTime = millis();
              while (!Serial2.available() && millis() - startTime < 5000) {
                  // Waiting for response with 5 seconds timeout
                  vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent blocking CPU
              }
              
              // Read and store the response
              if (Serial2.available()) {
                  String data = Serial2.readStringUntil('\n');
                  data.toCharArray(dataBuffer, BUFFER_SIZE);
              }

              xEventGroupSetBits(stateMachine.xDetectionsEventGroup, BIT_NEW_DATA_AVAILABLE);
          }
        }
        taskYIELD();
    }
}


// processDetTask - processing... still processing... maybe get a coffee? Or even better, get me one.
void processDetTask(void *pvParameters) {
    for (;;) {
      EventBits_t uxBits = xEventGroupWaitBits(
              stateMachine.xDetectionsEventGroup,
              BIT_NEW_DATA_AVAILABLE,
              pdTRUE,    // Clear BIT_NEW_DATA_AVAILABLE on exit.
              pdFALSE,   // Wait for just BIT_NEW_DATA_AVAILABLE.
              portMAX_DELAY); // Wait indefinitely.
          if (stateMachine.getCurrentRobotState() != DONE || stateMachine.getCurrentRobotState() != EMERGENCY_STOP) {
            if ((uxBits & BIT_NEW_DATA_AVAILABLE) != 0) {
              vTaskDelay(pdMS_TO_TICKS(50));
              // Serial.println("ProcessDetTask");
              
              // Process the data in dataBuffer
              // Serial.println("Received Detections");
              // Serial.println(dataBuffer);
              xSemaphoreTake(bufferMutex, portMAX_DELAY);
              processDetections(dataBuffer);
              xSemaphoreGive(bufferMutex);
              xEventGroupSetBits(stateMachine.xDetectionsEventGroup, BIT_READ_DETECTIONS);
            }
          }
        // Yield to other tasks
        taskYIELD();
    }
}

// DebugBox - A.K.A. the 'what the heck is going on' box
void DebugBox(void *pvParameters) {
    for (;;) {
        // if (printDebugFlag) {
      if (stateMachine.getCurrentRobotState() != DONE || stateMachine.getCurrentRobotState() != EMERGENCY_STOP) {
        vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
        Serial.println("PrintDebug");
        
        // vTaskDelay(pdMS_TO_TICKS(2000)); // FreeRTOS delay
        eTaskState readtaskState = eTaskGetState(stateMachine.readDetTaskHandle);
        eTaskState processtaskState = eTaskGetState(stateMachine.processDetTaskHandle);
        // Serial.println(readtaskState);
        // Serial.println(processtaskState);

        if ((readtaskState <= 2) || (processtaskState <= 2)){
          Serial.println("DETECTIONS ON");
          printDetections();
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000)); // FreeRTOS delay
      }

        // Yield to other tasks
        taskYIELD();
    }
}






void processDetections(char data[]) {
    // Tokenize the data string into individual detections using strtok
    char* detection = strtok(data, ";");
    while (detection != NULL) {
        // Serial.println(detection);
        parseDetection(detection);
        detection = strtok(NULL, ";"); // Get next detection
    }
}


void parseDetection(char* detection) {
    char class_name[MAX_CLASSNAME_LENGTH];
    int class_key, dir_key;
    float depth_mm;
    float x;
    float horizontal_angle, timestamp;
    char direction[MAX_DIRECTION_LENGTH];
    char* token;
    char* rest = detection;

        token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        class_key = atoi(token);
    }

    // Continue for the rest of the fields
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        timestamp = atof(token);
    }

    // Continue for the rest of the fields
    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        depth_mm = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        x = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        horizontal_angle = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        dir_key = atoi(token);;
    }

    String ck(class_key);
    String dk(dir_key);
    
    String class_n = class_names_dict[ck];
    String dir_n = dir_names_dict[dk];

    class_n.toCharArray(class_name, MAX_CLASSNAME_LENGTH);
    dir_n.toCharArray(direction, MAX_DIRECTION_LENGTH);
    
    Detection newDetection(class_name, timestamp, depth_mm, x, horizontal_angle, direction);
    // if (!debugMode){
    //   printDetection(newDetection);
    // }
    
    addDetectionToBuffer(newDetection);

}


void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // // Print the latest detection 
    // Detection latest = getLatestDetection();
    // Serial.println("Latest Detection:");
    // printDetection(latest);

    // // Loop through and print all detections
    // Serial.println("All Detections:");
    // for (int i = 0; i < getBufferSize(); i++) {
    //     Detection d = getDetectionFromBuffer(i);
    //     printDetection(d);
    // }
}



void printDetection(const Detection& d) {
    if (strlen(d.class_name) > 0) { // Check if the detection is valid
        Serial.print("Class Name: ");
        Serial.println(d.class_name);
        Serial.print("Timestamp: ");
        Serial.println(d.timestamp, 2);
        Serial.print("Depth MM: ");
        Serial.println(d.depth_mm);
        Serial.print("X Component: ");
        Serial.println(d.x);
        Serial.print("Horizontal Angle: ");
        Serial.println(d.horizontal_angle);
        Serial.print("Direction: ");
        Serial.println(d.direction);
        Serial.println("-------------------");
    } else {
        Serial.println("No Detection Data");
    }
}


void emergencyStopISR() {
    stateMachine.setEmergencyState();
    Serial.println("Emergency Stop");
    // Disconnect Motors Here
   
}

void debugModeISR() {
    debugMode = true;
    Serial.println("Debug Mode On");
    // Disconnect Motors Here
   
}
