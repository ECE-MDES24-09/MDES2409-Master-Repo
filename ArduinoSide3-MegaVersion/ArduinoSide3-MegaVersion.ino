/**

  This is the version of the communication code between Arduino and Jetson for the Arduino Mega.
  DO NOT USE IT ON AN UNO. IT WILL NOT COMPILE. IT USES TOO MUCH MEMORY FOR THE UNO. DON'T DO IT. SERIOUSLY. DON'T.
  This version is optimized with a dictionary data structure, and so the detections are sent through with 
  an integer value for the class name and direction and are then returned to their original names via a 
  dictionary.

**/

#include <pt.h>
#include <DetectionsBuffer.h>
#include <Dictionary.h>
#include <SoftwareSerial.h>
#define BUFFER_SIZE 512
#define MAX_DETECTION_LENGTH 15

SoftwareSerial mySerial(10, 11); // RX, TX pins // This is from the UNO version. Leaving it Just in Case.
char dataBuffer[BUFFER_SIZE];
volatile bool newDataAvailable = false;
unsigned long previousMillis = 0;  // Stores the last time a request was made
const long interval = 10000;      
volatile bool printDetectionsFlag = false;
volatile bool readDetectionsFlag = true;
int ignore_list[11];
volatile bool ignorebBox, ignoresBox, ignoresZone, ignorerZone, ignorebZone, ignoregZone = false;


Dictionary &class_names_dict = *(new Dictionary(11));
Dictionary &dir_names_dict = *(new Dictionary(2));
Dictionary &class_names_rev = *(new Dictionary(11));



struct pt ptRead, ptProcess, ptPrint;


void setup() {
    Serial.begin(9600);
    Serial2.begin(9600);
    clearBuffer();
    String dir_json = "{\"0\": \"left\", \"1\": \"right\"}";
    String class_json = "{\"0\": \"BigBox\", \"1\": \"BlueZone\", \"2\": \"Button\", \"3\": \"GreenZone\", \"4\": \"Nozzle\", \"5\": \"RedZone\", \"6\": \"Rocket\", \"7\": \"SmallBox\", \"8\": \"StartZone\", \"9\": \"WhiteLine\", \"10\": \"YellowLine\"}";
    String class_rev =  "{\"BigBox\": \"0\", \"BlueZone\": \"1\", \"Button\": \"2\", \"GreenZone\": \"3\", \"Nozzle\": \"4\", \"RedZone\": \"5\", \"Rocket\": \"5\", \"SmallBox\": \"7\", \"StartZone\": \"8\", \"WhiteLine\": \"9\", \"YellowLine\": \"10\"}";
    dir_names_dict.jload(dir_json);
    class_names_dict.jload(class_json);
    class_names_rev.jload(class_rev);
    // Wait for everything to stabilize
    // delay(60000); // Use this delay if starting at the same time as the Jetson
    delay(2000); // Use this delay if starting after Jetson is set up and has been running

    PT_INIT(&ptRead);
    PT_INIT(&ptProcess);
    PT_INIT(&ptPrint);
}


void loop() {
    protothreadRead(&ptRead);
    protothreadProcess(&ptProcess);
    protothreadPrint(&ptPrint);
}


static int protothreadRead(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (readDetectionsFlag) {
          delay(50);
          Serial2.println("REQUEST");
          // Serial.println("REQUEST");

          // Wait for a response with a timeout
          unsigned long startTime = millis();
          while (!Serial2.available() && millis() - startTime < 5000) {
            // Waiting for response with 5 seconds timeout
          }
          // Read and store the response
          if (Serial2.available()) {
              String data = Serial2.readStringUntil('\n');
              data.toCharArray(dataBuffer, BUFFER_SIZE);
              newDataAvailable = true; // Set the flag to indicate new data
              readDetectionsFlag = false;
          }
        }

        PT_YIELD(pt);  // Allow other protothreads to run
    }

    PT_END(pt);
}


static int protothreadProcess(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        if (newDataAvailable) {
            // Process the data in dataBuffer
            Serial.println("Received Detections");
            // Serial.println(dataBuffer);
            processDetections(dataBuffer);
            newDataAvailable = false; // Reset the flag after processing
            printDetectionsFlag = true;
        }

        PT_YIELD(pt);  // Allow other protothreads to run
    }

    PT_END(pt);
}

static int protothreadPrint(struct pt* pt) {
    PT_BEGIN(pt);

    while (1) {
        // Check if there are detections to print
        if (printDetectionsFlag) {
            printDetections();
            printDetectionsFlag = false; // Reset the flag
            readDetectionsFlag = true;
        }

        PT_YIELD(pt); // Allow other protothreads to run
    }

    PT_END(pt);
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
