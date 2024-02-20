#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <DetectionsBuffer.h>
#include <Dictionary.h>
#include <SoftwareSerial.h>
#define BUFFER_SIZE 512
#define MAX_DETECTION_LENGTH 15


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
  WAITING_TO_START,
  GET_BIG_BOXES,
  GET_SMALL_BOXES,
  DEPOSIT_BIG_BOXES,
  DEPOSIT_SMALL_BOXES,
  FOLLOW_LINE,
  GO_TO_RED_ZONE,
  GO_TO_BLUE_ZONE,
  GO_TO_GREEN_ZONE,
  PICK_UP_ROCKETS,
  CROSS_GAP,
  DEPOSIT_ROCKETS,
  PUSH_BUTTON,
  DISPLAY_LOGO,
  DONE,
  EMERGENCY_STOP
};

// Task Handlers
TaskHandle_t readDetTaskHandle;
TaskHandle_t processDetTaskHandle;
TaskHandle_t printTaskHandle;
SemaphoreHandle_t stateMutex;

char dataBuffer[BUFFER_SIZE];

volatile bool newDataAvailable = false;
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;  
volatile bool printDebugFlag = false;
volatile bool readDetectionsFlag = true;
int ignore_list[11];
bool ignorebBox, ignoresBox, ignoresZone, ignorerZone, ignorebZone, ignoregZone = false;


Dictionary &class_names_dict = *(new Dictionary(11));
Dictionary &dir_names_dict = *(new Dictionary(2));
Dictionary &class_names_rev = *(new Dictionary(11));


RobotState currentState = WAITING_TO_START;


void TaskStateManagement(void *pvParameters);
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

  xTaskCreate(TaskStateManagement, "StateManagement", 128, NULL, 4, NULL);
  xTaskCreate(readDetTask, "readDetTask", 1000, NULL, 3, &readDetTaskHandle);
  xTaskCreate(processDetTask, "processDetTask", 1000, NULL, 2, &processDetTaskHandle);
  xTaskCreate(printDebug, "printDebug", 1000, NULL, 1, &printTaskHandle);

  // Wait for everything to stabilize
  // delay(60000); // Use this delay if starting at the same time as the Jetson
  delay(2000); // Use this delay if starting after Jetson is set up and has been running


}


// LEAVE THIS EMPTY. NO TOUCHING. AT ALL. UNDER ANY CIRCUMSTANCES. JUST DON'T DO IT.
// WITH THE WAY THIS CODE IS SET UP WE WILL NEVER REACH THIS SECTION OF THE CODE.
void loop() {
  // put your main code here, to run repeatedly:

}



void TaskStateManagement(void *pvParameters) {
  for (;;) {
    switch (currentState) {
      case WAITING_TO_START:
        // Code to handle waiting to start
        break;
      case GET_BIG_BLOCKS:
        // Code for getting big blocks
        break;
      case GET_SMALL_BLOCKS:
        // Code for getting small blocks
        break;
      case FOLLOW_YELLOW_LINE:
        // Code to follow the yellow line
        break;
      case GO_TO_RED_ZONE:
        // Code to go to the red zone
        break;
      // ... add cases for other states as needed ...
      default:
        break;
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); // Adjust delay as needed
  }
}


void readDetTask(void *pvParameters) {
    for (;;) { // Infinite loop for the task
        if (readDetectionsFlag) {

          vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay

          Serial.println("ReadTask");
          readDetectionsFlag = false;
            
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
            //     newDataAvailable = true; // Set the flag to indicate new data
            //     readDetectionsFlag = false;
            // }
        }

        taskYIELD();
    }
}


void processDetTask(void *pvParameters) {
    for (;;) {
      if (newDataAvailable) {
        vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
        Serial.println("ProcessTask");
        // Process the data in dataBuffer
        // Serial.println("Received Detections");
        // // Serial.println(dataBuffer);
        // processDetections(dataBuffer);
        // newDataAvailable = false; // Reset the flag after processing
        // printDebugFlag = true;
        }
        // Yield to other tasks
        taskYIELD();
    }
}

void printDebug(void *pvParameters) {
    for (;;) {
        if (printDebugFlag) {
            vTaskDelay(pdMS_TO_TICKS(50)); // FreeRTOS delay
            printDetections();
            printDebugFlag = false; // Reset the flag
            readDetectionsFlag = true;
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
    char class_name[MAX_DETECTION_LENGTH];
    int class_key, dir_key;
    float confidence;
    float depth_mm;
    float depth_in;
    float x, y, z;
    float horizontal_angle, timestamp;
    char direction[6];
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

    class_n.toCharArray(class_name, MAX_DETECTION_LENGTH);
    dir_n.toCharArray(direction, 6);

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



void wait_to_start() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Waiting to Start");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = GET_BIG_BOXES;
}

void get_big_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Getting Big Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = GET_SMALL_BOXES;
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
  currentState = FOLLOW_LINE;
}


void deposit_big_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Depositing Big Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = DEPOSIT_SMALL_BOXES;
}

void deposit_small_boxes() {
  vTaskDelay(100 / portTICK_PERIOD_MS);
  Serial.println("Depositing Small Boxes");
  vTaskDelay(5000 / portTICK_PERIOD_MS);
  currentState = FOLLOW_LINE;
}

