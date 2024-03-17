//
// Created by jorda on 03/15/2024.
//

#include "RobotStateController.h"


/**
 * These are the the functions for the State Struct
 **/


/**
 * Boolean Equals Operators
**/
bool operator==(const State &lhs, const State &rhs) {
    return lhs.currState == rhs.currState
           && lhs.followLineCounter == rhs.followLineCounter
           && lhs.func == rhs.func
           && lhs.prev == rhs.prev
           && lhs.next == rhs.next;
}

bool operator!=(const State &lhs, const State &rhs) {
    return !(lhs == rhs);
}


RobotState State::getCurrentState() const {
    return currState;
}

StateFunc State::getFunc() const {
    return func;
}

void State::setFunc(StateFunc newFunc) {
    func = newFunc;
}

State *State::getPrev() const {
    return prev;
}

void State::setPrev(State *newPrev) {
    prev = newPrev;
}

State *State::getNext() const {
    return next;
}

void State::setNext(State *newNext) {
    next = newNext;
}

State::~State() = default;

State::State(const RobotState state, const int follow_line_counter, const StateFunc func, State *const prev,
             State *const next): currState(state),
                                 followLineCounter(follow_line_counter),
                                 func(func),
                                 prev(prev),
                                 next(next) {
}

/**
 * These are the the functions for the RobotStateController Class
 */
RobotStateController::RobotStateController() {
    pinMode(lightSensorPin, INPUT);
    // Initialize the robotCurrentState to the first state.
    robotCurrentState = &states[0];
    // Since there's no previous state for the first state, set it to nullptr.
    robotPrevState = nullptr;
    // Initialize the robotNextState to the second state in the array.
    robotNextState = states[0].getNext();
}

RobotStateController::~RobotStateController() = default;

void RobotStateController::update() {
    RobotState currentState = getCurrentRobotState();
    if (currentState == FOLLOW_LINE) {
        int followLineCounter = robotCurrentState->followLineCounter;
        Serial2.print(getStateName(robotCurrentState->getCurrentState()));
        Serial2.print(".");
        Serial2.println(followLineCounter);
        Serial.print(getStateName(robotCurrentState->getCurrentState()));
        Serial.print(".");
        Serial.println(followLineCounter);
    } else {
        Serial2.println(getStateName(robotCurrentState->getCurrentState()));
        Serial.println(getStateName(robotCurrentState->getCurrentState()));
    }
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
}

void RobotStateController::setState(State *newState) {
    robotCurrentState = newState;
    robotPrevState = robotCurrentState->prev;
    robotNextState = robotCurrentState->next;
    update();
}

void RobotStateController::proceed() {
    robotPrevState = robotCurrentState;
    robotCurrentState = robotCurrentState->next;
    robotNextState = robotCurrentState->next;
    update();
}

void RobotStateController::goBack() {
    robotNextState = robotCurrentState;
    robotCurrentState = robotCurrentState->prev;
    robotPrevState = robotCurrentState->prev;
    update();
}

void RobotStateController::reset() {
    setState(&states[0]);
}
void RobotStateController::init() {
    robotControl.init();
	robotControl.cruisin();
}

State *RobotStateController::getPrevState() {
    return robotCurrentState->prev;
}

State *RobotStateController::getCurrentState() {
    return robotCurrentState;
}

State *RobotStateController::getNextState() {
    return robotCurrentState->next;
}

RobotState RobotStateController::getCurrentRobotState() {
    return robotCurrentState->currState;
}

State *RobotStateController::getState(RobotState searchState, int lineFollowCounter=0) {
    switch (searchState) {
        case WAIT_FOR_START: return &states[0];
        case GET_BIG_BOXES: return &states[1];
        case GET_SMALL_BOXES: return &states[2];
        case DEPOSIT_BIG_BOXES: return &states[7];
        case DEPOSIT_SMALL_BOXES: return &states[5];
        case FOLLOW_LINE:
            switch (lineFollowCounter) {
                case 0:
                    return &states[3];
                case 1:
                    return &states[8];
                case 2:
                    return &states[11];
                case 3:
                    return &states[13];
                default: return &states[18];
            }
        case GO_TO_RED_ZONE: return &states[4];
        case GO_TO_BLUE_ZONE: return &states[6];
        case GO_TO_GREEN_ZONE: return &states[9];
        case GET_ROCKETS: return &states[10];
        case DEPOSIT_ROCKETS: return &states[14];
        case CROSS_GAP: return &states[12];
        case DISPLAY_LOGO: return &states[15];
        case PUSH_BUTTON: return &states[16];
        case DONE: return &states[17];
        default: return &states[18];
    }
}


int RobotStateController::getStateNum(RobotState searchState, int lineFollowCounter=0) {
    switch (searchState) {
        case WAIT_FOR_START: return 0;
        case GET_BIG_BOXES: return 1;
        case GET_SMALL_BOXES: return 2;
        case DEPOSIT_BIG_BOXES: return 7;
        case DEPOSIT_SMALL_BOXES: return 5;
        case FOLLOW_LINE:
            switch (lineFollowCounter) {
                case 0:
                    return 3;
                case 1:
                    return 8;
                case 2:
                    return 11;
                case 3:
                    return 13;
                default: return 18;
            }
        case GO_TO_RED_ZONE: return 4;
        case GO_TO_BLUE_ZONE: return 6;
        case GO_TO_GREEN_ZONE: return 9;
        case GET_ROCKETS: return 10;
        case DEPOSIT_ROCKETS: return 14;
        case CROSS_GAP: return 12;
        case DISPLAY_LOGO: return 15;
        case PUSH_BUTTON: return 16;
        case DONE: return 17;
        case EMERGENCY_STOP: return 18;
        default: return 18;
    }
}


void RobotStateController::setEmergencyState() {
    State* tmpPrev = robotCurrentState;
    State* tmpNext = robotCurrentState->next;
    setState(&states[18]);
    robotCurrentState->prev = tmpPrev;
    robotCurrentState->next = tmpNext;
}

void RobotStateController::setGyroData(float gyro_X, float gyro_Y, float gyro_Z) {
	robotControl.gyro_X = gyro_X;
	robotControl.gyro_Y = gyro_Y;
	robotControl.gyro_Z = gyro_Z;
}

GyroData RobotStateController::getGyroData() {
    GyroData data;
    data.x = robotControl.gyro_X;
    data.y = robotControl.gyro_Y;
    data.z = robotControl.gyro_Z;
    return data;
}

int RobotStateController::getLineFollowCounter() {
    return robotCurrentState->followLineCounter;
}

void RobotStateController::turnRight(){
    TickType_t startTick = xTaskGetTickCount();
	TickType_t delayTicks = pdMS_TO_TICKS(250);
	robotControl.motorDriver.setSpeed(150,-150);
	while ((xTaskGetTickCount() - startTick) < delayTicks) {
        robotControl.motorDriver.startMove();
		vTaskDelay(1 / portTICK_PERIOD_MS);
    }
	
}

void RobotStateController::turnLeft(){
    TickType_t startTick = xTaskGetTickCount();
	TickType_t delayTicks = pdMS_TO_TICKS(250);
	robotControl.motorDriver.setSpeed(-150,150);
	while ((xTaskGetTickCount() - startTick) < delayTicks) {
		robotControl.motorDriver.startMove();
		vTaskDelay(1 / portTICK_PERIOD_MS);    
		}
}

void RobotStateController::turnTo(Detection detection) {
	while (detection.horizontal_angle <= -0.5) {
		
	}
}



/**
 * These are the State Functions.
 **/
// wait_for_start - because patience is a virtue, or so I'm told
// State Number 0
// Current Max Time 0 seconds (Doesn't have time limit)
void RobotStateController::wait_for_start() {
    xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
    vTaskResume(readDetTaskHandle);
    vTaskResume(processDetTaskHandle);
	Serial.println("In Task");
    vTaskDelay(10 / portTICK_PERIOD_MS);
	robotControl.cruisin();
    int lightSensorValue = analogRead(lightSensorPin);
    Serial.print("Light Sensor Value: ");
    Serial.println(lightSensorValue);
    while (lightSensorValue > threshold || robotControl.ColorSensor()>200) {
        lightSensorValue = analogRead(lightSensorPin);
        Serial.print("Light Sensor Value: ");
        Serial.println(lightSensorValue);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
	robotControl.motorDriver.setSpeed(-150,150);
	for(int i = 0; i<1400; i++){
		robotControl.motorDriver.startMove();
		vTaskDelay(1 / portTICK_PERIOD_MS);		
	}
	robotControl.motorDriver.setSpeed(0,0);
	for(int i = 0; i<500; i++){
		robotControl.motorDriver.startMove();
		vTaskDelay(1 / portTICK_PERIOD_MS);	
	}
    proceed();
}

// get_big_boxes - It's like shopping, but for robots
// State Number 1
// Current Max Time 10 seconds
void RobotStateController::get_big_boxes() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    Serial.println("In Task");
    xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// get_small_boxes - Because size isn't everything
// State Number 2
// Current Max Time 10 seconds
void RobotStateController::get_small_boxes() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    Serial.println("In Task");
    proceed();
}

// follow_line - Staying inside the lines in sometimes necessary.
// Don't tell my kindergarten teacher I said that.
// State Numbers 3, 8, 11, 13
// Current Max Times 7, 5, 5, and 5 seconds respectively
void RobotStateController::follow_line() {
	int followLineCounter = robotCurrentState->followLineCounter;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskSuspend( readDetTaskHandle );
    vTaskSuspend( processDetTaskHandle );
    Serial.println("In Task");
	if (followLineCounter < 2) {
		while(abs(robotControl.GetPixyAngle())<40){
			robotControl.lineFollow(200, 0);
		}
		robotControl.motorDriver.setSpeed(200,200);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(0, 0);
		for(int i = 255;i>0;i--){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);		
		}
		robotControl.motorDriver.setSpeed(-150,150);
		for(int i = 0; i<1400; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
	}
	if (followLineCounter == 2) {
		while(abs(robotControl.GetPixyAngle())<40){
		  robotControl.lineFollow(150, 0);
		}
		robotControl.motorDriver.setSpeed(0, 0);
		for(int i = 0;i<256;i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(150,150);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(50,50);
		for(int i = 0; i<7500; i++){
			robotControl.motorDriver.startMove();
			robotControl.myservo.write(93);
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(-50,-50);
		for(int i = 0; i<700; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<500; i++){
			robotControl.motorDriver.startMove();
			robotControl.myservo.write(120);
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(255,255);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(100,100);
		while(robotControl.USDistance()>17){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(-150,150);
		for(int i = 0; i<1400; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}

		robotControl.motorDriver.setSpeed(-100,-100);
		while(robotControl.USDistance()<36){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
		robotControl.motorDriver.setSpeed(0,0);
		for(int i = 0; i<1000; i++){
			robotControl.motorDriver.startMove();
			vTaskDelay(1 / portTICK_PERIOD_MS);
		}
	}
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// deposit_big_boxes - Making deposits, but sadly not in your bank account. Or mine.
// State Number 7
// Current Max Time 3 seconds
void RobotStateController::deposit_big_boxes() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskSuspend( readDetTaskHandle );
    vTaskSuspend( processDetTaskHandle );
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}


// deposit_small_boxes - Because every box deserves a home
// State Number 5
// Current Max Time 3 seconds
void RobotStateController::deposit_small_boxes() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskSuspend( readDetTaskHandle );
    vTaskSuspend( processDetTaskHandle );
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// go_to_red_zone - Red: The color of urgency (or tomatoes)
// State Number 4
// Current Max Time 4 seconds
void RobotStateController::go_to_red_zone() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}


// go_to_blue_zone - Feeling blue? Head here
// State Number 6
// Current Max Time 2 seconds
void RobotStateController::go_to_blue_zone() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// go_to_green_zone - The eco-friendly zone
// State Number 9
// Current Max Time 3 seconds
void RobotStateController::go_to_green_zone() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// get_rockets - It's not rocket science. Oh wait, yes it is!
// Credit for this one goes to one of my T-shirts.
// State Number 10
// Current Max Time 10 seconds
void RobotStateController::get_rockets() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// deposit_rockets - One small step for gravity. One big leap for our robot.
// State Number 14
// Current Max Time 16 seconds
void RobotStateController::deposit_rockets() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xEventGroupSetBits(xDetectionsEventGroup, BIT_READ_DETECTIONS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}


// cross_gap - Mind the gap!
// State Number 12
// Current Max Time 16 seconds
void RobotStateController::cross_gap() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    robotControl.crossGap();
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// display_logo - Time for a comme	ial break
// State Number 5
// Current Max Time 2 seconds
void RobotStateController::display_logo() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// push_button - The big red button moment we've all been waiting for.
// No, it won't launch missiles... I think.
// State Number 16
// Current Max Time 3 seconds
void RobotStateController::push_button() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

// done - Congratulations, you've made it to the end!
// State Number 17
// Current Max Time 0 seconds (Doesn't have time limit)
void RobotStateController::done() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    vTaskSuspend( readDetTaskHandle );
    vTaskSuspend( processDetTaskHandle );
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    vTaskResume( readDetTaskHandle );
    vTaskResume( processDetTaskHandle );
    //proceed();
}

// emergency_stop - In case of fire, break glass. Or just call this.
// State Number 18
// Current Max Time 0 seconds (Doesn't have time limit)
void RobotStateController::emergency_stop() {
    vTaskDelay(100 / portTICK_PERIOD_MS);
    Serial.println("In Task");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    proceed();
}

const char *RobotStateController::getStateName(RobotState state) {
    switch (state) {
        case WAIT_FOR_START: return "WAIT_FOR_START";
        case GET_BIG_BOXES: return "GET_BIG_BOXES";
        case GET_SMALL_BOXES: return "GET_SMALL_BOXES";
        case DEPOSIT_BIG_BOXES: return "DEPOSIT_BIG_BOXES";
        case DEPOSIT_SMALL_BOXES: return "DEPOSIT_SMALL_BOXES";
        case FOLLOW_LINE: return "FOLLOW_LINE";
        case GO_TO_RED_ZONE: return "GO_TO_RED_ZONE";
        case GO_TO_BLUE_ZONE: return "GO_TO_BLUE_ZONE";
        case GO_TO_GREEN_ZONE: return "GO_TO_GREEN_ZONE";
        case GET_ROCKETS: return "GET_ROCKETS";
        case DEPOSIT_ROCKETS: return "DEPOSIT_ROCKETS";
        case CROSS_GAP: return "CROSS_GAP";
        case DISPLAY_LOGO: return "DISPLAY_LOGO";
        case PUSH_BUTTON: return "PUSH_BUTTON";
        case DONE: return "DONE";
        case EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN_STATE";
    }
}
