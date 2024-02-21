#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <event_groups.h>
#include <DetectionsBuffer.h>
#include <Dictionary.h>
#include <SoftwareSerial.h>


/**
 Note to everyone, this is the syntax for creating tasks

 BaseType_t xTaskCreate(TaskFunction_t pvTaskCode,
                       const char * const pcName,
                       unsigned short usStackDepth,
                       void *pvParameters,
                       UBaseType_t uxPriority,
                       TaskHandle_t *pvCreatedTask);

**/




// Enum for different robot states
enum RobotState {
  WAIT_FOR_START,
  GET_BIG_BOXES,
  GET_SMALL_BOXES,
  DEPOSIT_BIG_BOXES,
  DEPOSIT_SMALL_BOXES,
  FOLLOW_LINE, // Needs Counter
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

RobotState currentState = WAIT_FOR_START;
int Follow_Line_Counter = 0;



// Obj Detection Variables

#define BUFFER_SIZE 512 // Maximum size of String that can be passed from Jetson
#define MAX_CLASSNAME_LENGTH 15 // Maximum size of the class name char array
#define MAX_DIRECTION_LENGTH 6 // Maximum size of the direction char array

char dataBuffer[BUFFER_SIZE];
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;  

Dictionary &class_names_dict = *(new Dictionary(11));
Dictionary &dir_names_dict = *(new Dictionary(2));
Dictionary &class_names_rev = *(new Dictionary(11));


// Debug Mode Setup
const int debugPin = 10; // Debug mode toggle pin
bool debugMode = false; // Global flag for debug mode
volatile bool printDebugFlag = false;


// RTOS Vals

// Task Handlers
TaskHandle_t readDetTaskHandle;
TaskHandle_t processDetTaskHandle;
TaskHandle_t printTaskHandle;
// Mutex for RobotState
SemaphoreHandle_t stateMutex;
// Event Group for Detections
#define BIT_NEW_DATA_AVAILABLE (1 << 0)
#define BIT_READ_DETECTIONS    (1 << 1)
EventGroupHandle_t xDetectionsEventGroup;
// Tasks
void MotorBoxStateManagement(void *pvParameters);
void SensorBox(void *pvParameters);
void readDetTask(void *pvParameters);
void processDetTask(void *pvParameters);
void printDebug(void *pvParameters);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial2.begin(9600);
  clearBuffer();
  String dir_json = "{\"0\": \"left\", \"1\": \"right\"}";
  String class_json = "{\"0\": \"BigBox\", \"1\": \"BlueZone\", \"2\": \"Button\", \"3\": \"GreenZone\", \"4\": \"Nozzle\", \"5\": \"RedZone\", \"6\": \"Rocket\", \"7\": \"SmallBox\", \"8\": \"StartZone\", \"9\": \"WhiteLine\", \"10\": \"YellowLine\"}";
  String class_rev =  "{\"BigBox\": \"0\", \"BlueZone\": \"1\", \"Button\": \"2\", \"GreenZone\": \"3\", \"Nozzle\": \"4\", \"RedZone\": \"5\", \"Rocket\": \"5\", \"SmallBox\": \"7\", \"StartZone\": \"8\", \"WhiteLine\": \"9\", \"YellowLine\": \"10\"}";
  dir_names_dict.jload(dir_json);
  class_names_dict.jload(class_json);
  class_names_rev.jload(class_rev);

  // Create a mutex for state variable
  stateMutex = xSemaphoreCreateMutex();

  pinMode(debugPin, INPUT_PULLUP); // Set debug pin as input with pull-up
  debugMode = (digitalRead(debugPin) == LOW); // Check if the pin is LOW (switch closed)
  Serial.println(debugMode);


  xDetectionsEventGroup = xEventGroupCreate();


  // Wait for everything to stabilize
  // delay(60000); // Use this delay if starting at the same time as the Jetson
  
  xTaskCreate(MotorBoxStateManagement, "MotorBoxStateManagement", 128, NULL, 4, NULL);
  xTaskCreate(SensorBox, "SensorBox", 128, NULL, 3, NULL);
  xTaskCreate(readDetTask, "readDetTask", 1000, NULL, 2, &readDetTaskHandle);
  xTaskCreate(processDetTask, "processDetTask", 1000, NULL, 1, &processDetTaskHandle);

  if (debugMode) {
    xTaskCreate(printDebug, "printDebug", 1000, NULL, 5, NULL);
   }

}



// LEAVE THIS EMPTY. NO TOUCHING. AT ALL. UNDER ANY CIRCUMSTANCES. JUST DON'T DO IT.
// WITH THE WAY THIS CODE IS SET UP WE WILL NEVER REACH THIS SECTION OF THE CODE.
void loop() {
  // put your main code here, to run repeatedly:

}



void MotorBoxStateManagement(void *pvParameters) {
  for (;;) {
    switch (currentState) {
      case WAIT_FOR_START:
        // Code to handle waiting for start
        // Green Light stuff, yada yada
        wait_for_start();
        break;
      case GET_BIG_BOXES:
        // Code for getting big blocks
        get_big_boxes();
        break;
      case GET_SMALL_BOXES:
        // Code for getting small blocks
        get_small_boxes();
        break;
      case DEPOSIT_BIG_BOXES:
        // Code for depositing big blocks
        deposit_big_boxes();
        break;
      case DEPOSIT_SMALL_BOXES:
        // Code for depositing small blocks
        deposit_small_boxes();
        break;
      case FOLLOW_LINE:
        // Code to follow the yellow line
        follow_line();
        break;
      case GO_TO_RED_ZONE:
        // Code to go to the red zone
        go_to_red_zone();
        break;
      case GO_TO_BLUE_ZONE:
        // Code to go to the blue zone
        go_to_blue_zone();
        break;
      case GO_TO_GREEN_ZONE:
        // Code to go to the green zone
        go_to_green_zone();
        break;
      case GET_ROCKETS:
        // Code for getting rockets
        get_rockets();
        break;
      case DEPOSIT_ROCKETS:
        // Code for depositing rockets
        deposit_rockets();
        break;
      case CROSS_GAP:
        // Code to cross the gap
        cross_gap();
        break;
      case PUSH_BUTTON:
        // Code to push stop timer button
        push_button();
        break;
      case DISPLAY_LOGO:
        // Code to display the logo
        display_logo();
        break;
      case DONE:
        // Code for stopping when all tasks completed
        done();
        break;
      case EMERGENCY_STOP:
        // Code for emergency stop
        emergency_stop();
        break;
      default:
        break;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}

void SensorBox(void *pvParameters){
  for (;;) {
    Serial.println("Sensor Task");
    vTaskDelay(3000 / portTICK_PERIOD_MS); // On for 3 seconds
    // Serial.println("Task 2 has ended");
  }
}

  


void readDetTask(void *pvParameters) {
    for (;;) { // Infinite loop for the task
      EventBits_t uxBits = xEventGroupWaitBits(
              xDetectionsEventGroup,
              BIT_READ_DETECTIONS,
              pdTRUE,    // Clear BIT_READ_DETECTIONS on exit.
              pdFALSE,   // Wait for just BIT_READ_DETECTIONS.
              portMAX_DELAY); // Wait indefinitely.

          if ((uxBits & BIT_READ_DETECTIONS) != 0) {
            
            vTaskDelay(pdMS_TO_TICKS(50));
            Serial.println("ReadTask");
            vTaskDelay(pdMS_TO_TICKS(5000));
            xEventGroupSetBits(xDetectionsEventGroup, BIT_NEW_DATA_AVAILABLE);
    
            
            // Serial2.println("REQUEST");

            // // Wait for a response with a timeout
            // unsigned long startTime = millis();
            // while (!Serial2.available() && millis() - startTime < 5000) {
            //     // Waiting for response with 5 seconds timeout
            //     vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to prevent blocking CPU
            // }
            
            // // Read and store the response
            // if (Serial2.available()) {
            //     String data = Serial2.readStringUntil('\n');
            //     data.toCharArray(dataBuffer, BUFFER_SIZE);
            // }
        }

        taskYIELD();
    }
}


void processDetTask(void *pvParameters) {
    for (;;) {
      EventBits_t uxBits = xEventGroupWaitBits(
              xDetectionsEventGroup,
              BIT_NEW_DATA_AVAILABLE,
              pdTRUE,    // Clear BIT_NEW_DATA_AVAILABLE on exit.
              pdFALSE,   // Wait for just BIT_NEW_DATA_AVAILABLE.
              portMAX_DELAY); // Wait indefinitely.

          if ((uxBits & BIT_NEW_DATA_AVAILABLE) != 0) {
            vTaskDelay(pdMS_TO_TICKS(50));
            Serial.println("ProcessTask");
            vTaskDelay(pdMS_TO_TICKS(5000));
            // Process the data in dataBuffer
            // Serial.println("Received Detections");
            // // Serial.println(dataBuffer);
            // processDetections(dataBuffer);
            xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
        }
        // Yield to other tasks
        taskYIELD();
    }
}


void printDebug(void *pvParameters) {
    for (;;) {
        // if (printDebugFlag) {
        vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
        Serial.println("PrintDebug");
        vTaskDelay(pdMS_TO_TICKS(5000)); // FreeRTOS delay
        // }

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

    addDetectionToBuffer(newDetection);

}


void printDetections() {
    // Print the closest detection
    Detection closest = getClosestDetection();
    Serial.println("Closest Detection:");
    printDetection(closest);

    // Print the latest detection 
    Detection latest = getLatestDetection();
    Serial.println("Latest Detection:");
    printDetection(latest);

    // Loop through and print all detections
    Serial.println("All Detections:");
    for (int i = 0; i < getBufferSize(); i++) {
        Detection d = getDetectionFromBuffer(i);
        printDetection(d);
    }
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



void wait_for_start() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Waiting to Start");
  xSemaphoreTake(stateMutex, portMAX_DELAY);
  currentState = GET_BIG_BOXES;
  xSemaphoreGive(stateMutex);
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  
}

void get_big_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Getting Big Boxes");
  currentState = GET_SMALL_BOXES;
  xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
  vTaskDelay(5000 / portTICK_PERIOD_MS);

}

void get_small_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Getting Small Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = FOLLOW_LINE;
}

void follow_line() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Following Line");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  switch (Follow_Line_Counter) {
    case 0:
      currentState = GO_TO_RED_ZONE;
      Follow_Line_Counter++;
      break;
    case 1:
      currentState = GO_TO_GREEN_ZONE;
      Follow_Line_Counter++;
      break;
    case 2:
      currentState = CROSS_GAP;
      Follow_Line_Counter++;
      break;
    case 3:
      currentState = DEPOSIT_ROCKETS;
      Follow_Line_Counter++;
      break;
    default:
        break;
  }
}


void deposit_big_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Depositing Big Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = FOLLOW_LINE;
}

void deposit_small_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Depositing Small Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = GO_TO_BLUE_ZONE;
}


void go_to_red_zone() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Going to Red Zone");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = DEPOSIT_SMALL_BOXES;
}

void go_to_blue_zone() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Going to Blue Zone");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = DEPOSIT_BIG_BOXES;
}


void go_to_green_zone() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Going to Green Zone");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = GET_ROCKETS;
}

void get_rockets() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Getting Rockets");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = FOLLOW_LINE;
}

void deposit_rockets() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Depositing Rockets");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState =  PUSH_BUTTON;
}


void cross_gap() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Crossing Gap");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = FOLLOW_LINE;
}


void display_logo() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Displaying Logo");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState =  PUSH_BUTTON;
}


void push_button() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Pushing Button");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState =  DONE;
}


void done() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Done");
  vTaskDelay(10000 / portTICK_PERIOD_MS);
  currentState =  WAIT_FOR_START;
}


void emergency_stop() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("EMERGENCY STOP");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState =  DONE;
}