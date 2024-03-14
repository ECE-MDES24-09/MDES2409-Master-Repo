#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <task.h>
#include <event_groups.h>
#include <DetectionsBuffer.h>
#include <Dictionary.h>
#include <SoftwareSerial.h>
#include <TimeManagement.h>
#include <RoboDriver.h>
#include <globalVariables.h>
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

// Set up TimeManager. This robot is better at time management than me.
// And I built its time management system. Sigh.
TimeManagement timeManager;
// LineFollower lineFollower(200, 90);
Gyro gyro;

// Task Handlers - Like employees, but they never ask for a raise
TaskHandle_t readDetTaskHandle;
TaskHandle_t processDetTaskHandle;
TaskHandle_t SensorBoxTaskHandle;
TaskHandle_t debugTaskHandle;

// Mutex for RobotState
SemaphoreHandle_t stateMutex;
// Buffer Mutex
SemaphoreHandle_t bufferMutex;
// Serial Mutex
SemaphoreHandle_t serialMutex;

//Queue for Sensor Data
QueueHandle_t sensorQueue;

typedef struct {
    int intValue;
    float floatValue;
} sensorData;

// Event Group for Detections
EventGroupHandle_t xDetectionsEventGroup;

Dictionary &class_names_dict = *(new Dictionary(11));
Dictionary &dir_names_dict = *(new Dictionary(2));
Dictionary &class_names_rev = *(new Dictionary(11));

// Tasks
void MotorBoxStateManagement(void *pvParameters);
void SensorBox(void *pvParameters);
void readDetTask(void *pvParameters);
void processDetTask(void *pvParameters);
void DebugBox(void *pvParameters);
void StateTimeoutCallback(TimerHandle_t xTimer);

void setup() {
  // Let's set up our circus of tasks and hope they play nice together
  Serial.begin(9600);
  Serial2.begin(9600);
  clearBuffer();
  String dir_json = "{\"0\": \"left\", \"1\": \"right\"}";
  String class_json = "{\"0\": \"BigBox\", \"1\": \"BlueZone\", \"2\": \"Button\", \"3\": \"GreenZone\", \"4\": \"Nozzle\", \"5\": \"RedZone\", \"6\": \"Rocket\", \"7\": \"SmallBox\", \"8\": \"StartZone\", \"9\": \"WhiteLine\", \"10\": \"YellowLine\"}";
  String class_rev =  "{\"BigBox\": \"0\", \"BlueZone\": \"1\", \"Button\": \"2\", \"GreenZone\": \"3\", \"Nozzle\": \"4\", \"RedZone\": \"5\", \"Rocket\": \"5\", \"SmallBox\": \"7\", \"StartZone\": \"8\", \"WhiteLine\": \"9\", \"YellowLine\": \"10\"}";
  dir_names_dict.jload(dir_json);
  class_names_dict.jload(class_json);
  class_names_rev.jload(class_rev);
  
  //Inantiate Sensor Queue
  sensorQueue = xQueueCreate(10, sizeof(sensorData));

  //INitialize Gyro
  // gyro.gyro_init();

  // Create a mutex for state variable and serial
  stateMutex = xSemaphoreCreateMutex();
  bufferMutex = xSemaphoreCreateMutex();

  pinMode(emergencyStopPin, INPUT_PULLUP); // Set as input with pull-up
  pinMode(debugPin, INPUT_PULLUP); // Set debug pin as input with pull-up
  debugMode = (digitalRead(debugPin) == LOW); // Check if the pin is LOW (switch closed)
  // Serial.println(debugMode);
  
  // Attach interrupts because we're fancy like that
  attachInterrupt(digitalPinToInterrupt(emergencyStopPin), emergencyStopISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(debugPin), debugModeISR, FALLING);
  // When in doubt, just stop everything. It's the grown-up version of closing your eyes.


  xDetectionsEventGroup = xEventGroupCreate();


  // Creating our cast of tasks - it's like a talent show, but with more crashing
  xTaskCreate(MotorBoxStateManagement, "MotorBoxStateManagement", 1000, NULL, 1, &timeManager.MotorBoxTaskHandle);
  xTaskCreate(SensorBox, "SensorBox", 128, NULL, 2, &SensorBoxTaskHandle);
  xTaskCreate(readDetTask, "readDetTask", 500, NULL, 4, &readDetTaskHandle);
  xTaskCreate(processDetTask, "processDetTask", 500, NULL, 3, &processDetTaskHandle);

  if (debugMode) {
    xTaskCreate(DebugBox, "DebugBox", 200, NULL, 5, &debugTaskHandle);
   }
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



}



// LEAVE THIS EMPTY. NO TOUCHING. AT ALL. UNDER ANY CIRCUMSTANCES. JUST DON'T DO IT.
// WITH THE WAY THIS CODE IS SET UP WE WILL NEVER REACH THIS SECTION OF THE CODE.
// -- Jordan
void loop() {
  // LEAVE THIS EMPTY. Trespassers will be turned into pseudo code.

}


// MotorBoxStateManagement - Because who needs a driver when you have a function?
void MotorBoxStateManagement(void *pvParameters) {
  uint32_t notificationValue;
  const uint32_t MY_ULONG_MAX = 0xFFFFFFFF;

  for (;;) {
    if (xTaskNotifyWait(0x00, MY_ULONG_MAX, &notificationValue, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool timeout = notificationValue != 0;
      // State completed
      if (!timeout) {
        Serial.println("State completed normally");
      } else {
        Serial.println("We Timed out.");
      }
    }
    if (currentState != prevState) {
      Serial.println("Clearing Buffer");
      xSemaphoreTake(bufferMutex, portMAX_DELAY);
      clearBuffer();
      xSemaphoreGive(bufferMutex);
      if (timeManager.timeOut) {
        timeManager.timeOut = false;
      }
    }
    switch (currentState) {
      case WAIT_FOR_START:
        // Code to handle waiting for start
        // Green Light stuff, yada yada
        timeManager.startState(0); 
        wait_for_start();
        break;
      case GET_BIG_BOXES:
        // Code for getting big blocks
        timeManager.startState(1);  
        get_big_boxes();
        break;
      case GET_SMALL_BOXES:
        // Code for getting small blocks
        timeManager.startState(2);
        get_small_boxes();
        break;
      case DEPOSIT_BIG_BOXES:
        // Code for depositing big blocks
        timeManager.startState(7);  
        deposit_big_boxes();
        break;
      case DEPOSIT_SMALL_BOXES:
        // Code for depositing small blocks
        timeManager.startState(5);  
        deposit_small_boxes();
        break;
case FOLLOW_LINE:
        // Code to follow the yellow line
        switch (Follow_Line_Counter) {
          case 0:
            timeManager.startState(3); 
            break;
          case 1:
            timeManager.startState(8); 
            break;
          case 2:
            timeManager.startState(11); 
            break;
          case 3:
            timeManager.startState(13); 
            break;
          default:
              break;
        }

        follow_line();
        break;

      case GO_TO_RED_ZONE:
        // Code to go to the red zone
        // No touchdowns tho - Sorry.
        timeManager.startState(4); 
        go_to_red_zone();
        break;
      case GO_TO_BLUE_ZONE:
        // Code to go to the blue zone
        timeManager.startState(6); 
        go_to_blue_zone();
        break;
      case GO_TO_GREEN_ZONE:
        // Code to go to the green zone
        timeManager.startState(9); 
        go_to_green_zone();
        break;
      case GET_ROCKETS:
        // Code for getting rockets
        timeManager.startState(10); 
        get_rockets();
        break;
      case DEPOSIT_ROCKETS:
        // Code for depositing rockets
        timeManager.startState(14); 
        deposit_rockets();
        break;
      case CROSS_GAP:
        // Code to cross the gap
        timeManager.startState(12); 
        cross_gap();
        break;
      case PUSH_BUTTON:
        // Code to push stop timer button
        timeManager.startState(16); 
        push_button();
        break;
      case DISPLAY_LOGO:
        // Code to display the logo
        timeManager.startState(15); 
        display_logo();
        break;
      case DONE:
        // Code for stopping when all tasks completed
        timeManager.startState(17); 
        done();
        break;
      case EMERGENCY_STOP:
        // Code for emergency stop
        timeManager.startState(18); 
        emergency_stop();
        break;
      default:
        break;
    }
    prevState = currentState;
  }
}

// SensorBox - because we're sensing more than just disappointment
void SensorBox(void *pvParameters){
  for (;;) {
    if (currentState != DONE || currentState != EMERGENCY_STOP) {
      // Serial.println("Sensor Task");
      sensorData data;
      data.intValue = 42;
      data.floatValue = 0;
      //gyro.getGyroAng();
      
      if (xQueueSend(sensorQueue, &data, portMAX_DELAY) != pdPASS)
      {
        //Failed to send data to queue
      }

      vTaskDelay(300 / portTICK_PERIOD_MS); // On for 3 seconds
    }
  }
}

  

// readDetTask: Reading is fundamental, even for robots. Especially when it involves detecting stuff.
void readDetTask(void *pvParameters) {
    for (;;) { // Infinite loop for the task
      EventBits_t uxBits = xEventGroupWaitBits(
              xDetectionsEventGroup,
              BIT_READ_DETECTIONS,
              pdTRUE,    // Clear BIT_READ_DETECTIONS on exit.
              pdFALSE,   // Wait for just BIT_READ_DETECTIONS.
              portMAX_DELAY); // Wait indefinitely.
          if (currentState != DONE || currentState != EMERGENCY_STOP) {
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

              xEventGroupSetBits(xDetectionsEventGroup, BIT_NEW_DATA_AVAILABLE);
          }
        }
        taskYIELD();
    }
}


// processDetTask - processing... still processing... maybe get a coffee? Or even better, get me one.
void processDetTask(void *pvParameters) {
    for (;;) {
      EventBits_t uxBits = xEventGroupWaitBits(
              xDetectionsEventGroup,
              BIT_NEW_DATA_AVAILABLE,
              pdTRUE,    // Clear BIT_NEW_DATA_AVAILABLE on exit.
              pdFALSE,   // Wait for just BIT_NEW_DATA_AVAILABLE.
              portMAX_DELAY); // Wait indefinitely.
          if (currentState != DONE || currentState != EMERGENCY_STOP) {
            if ((uxBits & BIT_NEW_DATA_AVAILABLE) != 0) {
              vTaskDelay(pdMS_TO_TICKS(50));
              // Serial.println("ProcessDetTask");
              
              // Process the data in dataBuffer
              // Serial.println("Received Detections");
              // Serial.println(dataBuffer);
              xSemaphoreTake(bufferMutex, portMAX_DELAY);
              processDetections(dataBuffer);
              xSemaphoreGive(bufferMutex);
              xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
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
      
      if (currentState != DONE || currentState != EMERGENCY_STOP) {
        
        vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
        Serial.println("PrintDebug");
        
        // vTaskDelay(pdMS_TO_TICKS(2000)); // FreeRTOS delay
        eTaskState readtaskState = eTaskGetState(readDetTaskHandle);
        eTaskState processtaskState = eTaskGetState(processDetTaskHandle);
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
    float confidence;
    float depth_mm;
    float depth_in;
    float x, y, z;
    float horizontal_angle, timestamp;
    char direction[MAX_DIRECTION_LENGTH];
    char* token;
    char* rest = detection;

        token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        class_key = atoi(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        confidence = atof(token);
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
        y = atof(token);
    }

    token = strtok_r(rest, ",", &rest);
    if (token != NULL) {
        z = atof(token);
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
    
    Detection newDetection(class_name, confidence,timestamp, depth_mm, x, y, z, horizontal_angle, direction);
    if (!debugMode){
      printDetection(newDetection);
    }
    
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
        Serial.print("Confidence: ");
        Serial.println(d.confidence, 2);
        Serial.print("Timestamp: ");
        Serial.println(d.timestamp, 2);
        Serial.print("Depth MM: ");
        Serial.println(d.depth_mm);
        Serial.print("X Component: ");
        Serial.println(d.x);
        Serial.print("Y Component: ");
        Serial.println(d.y);
        Serial.print("Z Component: ");
        Serial.println(d.z);
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
    currentState = EMERGENCY_STOP;
    Serial.println("Emergency Stop");
    // Disconnect Motors Here
   
}

void debugModeISR() {
    debugMode = true;
    Serial.println("Debug Mode On");
    // Disconnect Motors Here
   
}


// wait_for_start - because patience is a virtue, or so I'm told
// State Number 0
// Current Max Time 0 seconds (Doesn't have time limit)
void wait_for_start() {
  
  xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("WAIT_FOR_START");
  Serial.println("Waiting to Start");

  sensorData receivedData;
  //Loop until sensor sends data
  while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
  {
  }
  Serial.print("Sensor int: ");
  Serial.println(receivedData.intValue);
  Serial.print("Sensor float: ");
  Serial.println(receivedData.floatValue);

  currentState = GET_BIG_BOXES;
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  timeManager.endState(0, false);
  
}

// get_big_boxes - It's like shopping, but for robots
// State Number 1
// Current Max Time 10 seconds
void get_big_boxes() {
  bool stateComplete = false;
  Detection most_recent;
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  Serial.println(stateComplete);
  Serial2.println("GET_BIG_BOXES");

  while ((timeManager.getRemainingTimeForState(1) > 0) && !stateComplete && !timeManager.timeOut){
    // vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println(timeManager.getRemainingTimeForState(1));
    Serial.println("Getting Big Boxes");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);
    
    // xSemaphoreTake(serialMutex, portMAX_DELAY);
    // xSemaphoreGive(serialMutex);
    most_recent = getLatestDetection();
    Serial.println("Most Recent Detection:");
    printDetection(most_recent);
    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
    }
  }
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(1, stateComplete);
  xSemaphoreGive(bufferMutex);
  currentState = GET_SMALL_BOXES;

}


// get_small_boxes - Because size isn't everything
// State Number 2
// Current Max Time 10 seconds
void get_small_boxes() {
  bool stateComplete = false;
  Serial2.println("GET_SMALL_BOXES");
  
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  while ((timeManager.getRemainingTimeForState(2) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(2));
    Serial.println("Getting Small Boxes");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    // timeManager.endState(2);
    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
    }
  } 
  currentState = FOLLOW_LINE;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(2, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// follow_line - Staying inside the lines in sometimes necessary. 
// Don't tell my kindergarten teacher I said that.
// State Numbers 3, 8, 11, 13
// Current Max Times 7, 5, 5, and 5 seconds respectively
void follow_line() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskSuspend( readDetTaskHandle );
  vTaskSuspend( processDetTaskHandle );
  Serial2.print("FOLLOW_LINE.");
  Serial2.println(Follow_Line_Counter);

  
  switch (Follow_Line_Counter) {
    case 0:
      line_follow(3, DEPOSIT_ROCKETS, GO_TO_RED_ZONE, stateComplete);      
      break;
    case 1:
      line_follow(8, GO_TO_GREEN_ZONE, Follow_Line_Counter, stateComplete);      
      break;
    case 2:
      line_follow(11, CROSS_GAP, Follow_Line_Counter, stateComplete);
      break;
    case 3:
      line_follow(13, DEPOSIT_ROCKETS, Follow_Line_Counter, stateComplete);
      break;
    default:
        break;
  }
  Follow_Line_Counter++;
}



// deposit_big_boxes - Making deposits, but sadly not in your bank account. Or mine.
// State Number 7
// Current Max Time 3 seconds
void deposit_big_boxes() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskSuspend( readDetTaskHandle );
  vTaskSuspend( processDetTaskHandle );
  Serial2.println("DEPOSIT_BIG_BOXES");

  while ((timeManager.getRemainingTimeForState(7) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(7));
    Serial.println("Depositing Big Boxes");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500)));   // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = FOLLOW_LINE;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(7, stateComplete);
  xSemaphoreGive(bufferMutex);
}


// deposit_small_boxes - Because every box deserves a home
// State Number 5
// Current Max Time 3 seconds
void deposit_small_boxes() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskSuspend( readDetTaskHandle );
  vTaskSuspend( processDetTaskHandle );
  Serial2.println("DEPOSIT_SMALL_BOXES");

  while ((timeManager.getRemainingTimeForState(5) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(5));
    Serial.println("Depositing Small Boxes");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500)));   // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = GO_TO_BLUE_ZONE;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(5, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// go_to_red_zone - Red: The color of urgency (or tomatoes)
// State Number 4
// Current Max Time 4 seconds
void go_to_red_zone() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
  Serial2.println("GO_TO_RED_ZONE");

  while ((timeManager.getRemainingTimeForState(4) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(4));
    Serial.println("Going to Red Zone");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = DEPOSIT_SMALL_BOXES;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(4, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// go_to_blue_zone - Feeling blue? Head here
// State Number 6
// Current Max Time 2 seconds
void go_to_blue_zone() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  Serial2.println("GO_TO_BLUE_ZONE");

  while ((timeManager.getRemainingTimeForState(6) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(6));
    Serial.println("Going to Blue Zone");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = DEPOSIT_BIG_BOXES;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(6, stateComplete);
  xSemaphoreGive(bufferMutex);
}


// go_to_green_zone - The eco-friendly zone
// State Number 9
// Current Max Time 3 seconds
void go_to_green_zone() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
  Serial2.println("GO_TO_GREEN_ZONE");

  while ((timeManager.getRemainingTimeForState(9) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(9));
    Serial.println("Going to Green Zone");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = GET_ROCKETS;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(9, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// get_rockets - It's not rocket science. Oh wait, yes it is!
// Credit for this one goes to one of my T-shirts.
// State Number 10
// Current Max Time 10 seconds
void get_rockets() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("GET_ROCKETS");

  while ((timeManager.getRemainingTimeForState(10) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(10));
    Serial.println("Getting Rockets");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = FOLLOW_LINE;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(10, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// deposit_rockets - One small step for gravity. One big leap for our robot.
// State Number 14
// Current Max Time 16 seconds
void deposit_rockets() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  vTaskResume( readDetTaskHandle );
  vTaskResume( processDetTaskHandle );
  Serial2.println("DEPOSIT_ROCKETS");

  while ((timeManager.getRemainingTimeForState(14) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(14));
    Serial.println("Depositing Rockets");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState =  DISPLAY_LOGO;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(14, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// cross_gap - Mind the gap!
// State Number 12
// Current Max Time 16 seconds
void cross_gap() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("CROSS_GAP");

  while ((timeManager.getRemainingTimeForState(12) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(12));
    Serial.println("Crossing Gap");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState = FOLLOW_LINE;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(12, stateComplete);
  xSemaphoreGive(bufferMutex);
}

// display_logo - Time for a commercial break
// State Number 5
// Current Max Time 2 seconds
void display_logo() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("DISPLAY_LOGO");

  while ((timeManager.getRemainingTimeForState(15) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(15));
    Serial.println("Displaying Logo");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState =  PUSH_BUTTON;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(15, stateComplete);
  xSemaphoreGive(bufferMutex);
}


// push_button - The big red button moment we've all been waiting for. 
// No, it won't launch missiles... I think.
// State Number 16
// Current Max Time 3 seconds
void push_button() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("PUSH_BUTTON");

  while ((timeManager.getRemainingTimeForState(16) > 0) && !stateComplete && !timeManager.timeOut ){
    Serial.println(timeManager.getRemainingTimeForState(16));
    Serial.println("Pushing Button");

    //Receiving Data from SensorQueue
    sensorData receivedData;
    //Loop until sensor sends data
    while(xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY) != pdPASS)
    {
    }
    Serial.print("Sensor int: ");
    Serial.println(receivedData.intValue);
    Serial.print("Sensor float: ");
    Serial.println(receivedData.floatValue);

    vTaskDelay(pdMS_TO_TICKS(150 + random(500))); // Delay for 1 to 5 seconds  // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
      }
  }
  currentState =  DONE;
  timeManager.endState(16, false);
  if (debugMode){
    vTaskSuspend( debugTaskHandle );
  }
}

// done - Congratulations, you've made it to the end!
// State Number 17
// Current Max Time 0 seconds (Doesn't have time limit)
void done() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("DONE");
  Serial.println("Done");
  long rTime = timeManager.getRemainingTime();
  long runTime = timeManager.getRunTime();
  Serial.print("Run Time: ");
  Serial.println(runTime);
  Serial.print("Remaining Time: ");
  Serial.println(rTime);
  vTaskSuspend( readDetTaskHandle );
  vTaskSuspend( processDetTaskHandle );
  vTaskSuspend( SensorBoxTaskHandle );
  if (debugMode){
    vTaskSuspend( debugTaskHandle );
  }
  vTaskDelay(1500 / portTICK_PERIOD_MS);
  // currentState =  WAIT_FOR_START;
  // Follow_Line_Counter = 0;
  // vTaskResume( readDetTaskHandle );
  // vTaskResume( processDetTaskHandle );
  // vTaskResume( SensorBoxTaskHandle );
  // if (debugMode){
  //   vTaskResume( debugTaskHandle );
  // }
  timeManager.endState(17, stateComplete);

}

// emergency_stop - In case of fire, break glass. Or just call this.
// State Number 18
// Current Max Time 0 seconds (Doesn't have time limit)
void emergency_stop() {
  bool stateComplete = false;
  // vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial2.println("EMERGENCY_STOP");
  Serial.println("EMERGENCY STOP");
  vTaskDelay(pdMS_TO_TICKS(1500 + random(4000))); // Delay for 1 to 5 seconds
  currentState =  DONE;
  timeManager.endState(18, stateComplete);
}


void line_follow(int stateId, RobotState nextState, int Follow_Line_Counter, bool stateComplete) {
  while ((timeManager.getRemainingTimeForState(stateId) > 0) && !stateComplete && !timeManager.timeOut) {
    Serial.println(timeManager.getRemainingTimeForState(stateId));
    Serial.print("Following Line.");
    Serial.println(Follow_Line_Counter);
    // if (lineFollower.followLine()) {
    //   stateComplete = true;
    // }

    vTaskDelay(pdMS_TO_TICKS(150 + random(1500))); // FreeRTOS delay
    long rand = random(100);
    Serial.println(rand);
    if (rand >= 65) {
      stateComplete = true;
      Serial.println(stateComplete);
    }
  }
  currentState = nextState;
  xSemaphoreTake(bufferMutex, portMAX_DELAY);
  timeManager.endState(stateId, stateComplete);
  xSemaphoreGive(bufferMutex);
}
