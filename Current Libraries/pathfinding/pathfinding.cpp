

// Global variables
TimeManager timeManager;
RobotState currentState;
SemaphoreHandle_t bufferMutex;
DetectionsBuffer detectionBuffer; // Instantiate DetectionsBuffer class

// Function prototypes
bool findBestPath();
void line_follow(int stateId, RobotState nextState, int Follow_Line_Counter, bool &stateComplete);

void setup()
{
    // Setup code here
}

void loop()
{
    // Main loop code here
    line_follow(0, RobotState(), 0, false);
}

void line_follow(int stateId, RobotState nextState, int Follow_Line_Counter, bool &stateComplete)
{
    while ((timeManager.getRemainingTimeForState(stateId) > 0) && !stateComplete && !timeManager.timeOut)
    {
        Serial.println(timeManager.getRemainingTimeForState(stateId));
        Serial.print("Following Line.");
        Serial.println(Follow_Line_Counter);
        RoboDriver roboDriver; // Create an instance of RoboDriver
        if (roboDriver.followLineUntilTurn())
        {
            stateComplete = true;
        }
        // Update path twice a second
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay for 500 milliseconds

        if (!findBestPath(roboDriver))  // could be used to end state early
        {
             stateComplete = true;
        }
    }
    currentState = nextState;
    xSemaphoreTake(bufferMutex, portMAX_DELAY);
    timeManager.endState(stateId, stateComplete);
    xSemaphoreGive(bufferMutex);
}

bool findBestPath(RoboDriver roboDriver)
{
    // Get the leftmost detection
    Detection leftmostDetection = detectionBuffer.getLeftmostDetection();
    Detection closestDetection = detectionBuffer.getClosestDetection();

    if (closestDetection <= 7) // will be changed from zero after testing
    {
        roboDriver.stopTheMotors();
        return false; // not sure if correct but may be needed for ended early for extra time
    }

    // Adjust the robot's direction based on the horizontal offset (x) of the leftmost detection
    // For simplicity, let's assume the robot should move towards the object with the smallest x value
    if (leftmostDetection.x < -2)
    {
        // Object detected on the left, turn left
        // has to turn a little less than value to be straight assuming 
        roboDriver.turn(leftmostDetection.x + 2);
    }
    else if (leftmostDetection.x > -2)
    {
        // Object detected on the right, turn right
        // has to turn a little more than value to be straight
        roboDriver.turn(leftmostDetection.x + 2);
    }
    else
    {
        // No object detected or leftmostDetection is , continue straight
        // Set motors to drive forward
        roboDriver.startTheMotors();
    }

    // Always return true for a straight line path
    return true;
}