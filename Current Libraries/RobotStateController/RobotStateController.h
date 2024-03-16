//
// Created by jorda on 03/15/2024.
//

#ifndef ROBOTSTATECONTROLLER_H
#define ROBOTSTATECONTROLLER_H

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <event_groups.h>
#include "robot_control.h"
#include <globalVariables.h>

class RobotStateController;

// Function pointer type for state handling methods in RobotStateController
typedef void (RobotStateController::* StateFunc)();

// Function pointer type for event handling methods in RobotStateController
typedef void (RobotStateController::* EventFunc)(Event);

// Structure representing a state of the robot
struct State {
    RobotState currState; // Current state of the robot
    int followLineCounter; // Counter for line following logic
    StateFunc func; // Function to call for the current state
    State* prev; // Pointer to previous state
    State* next; // Pointer to next state
    bool stateComplete = false; // Flag to indicate if state is complete
    bool timeOut = false; // Flag to indicate if state has timed out

    // Constructor for State structure
    State(RobotState state, int follow_line_counter, StateFunc func, State* prev, State* next);

    // Destructor for State structure
    ~State();

    // Equality operator overloads for State comparison
    friend bool operator==(const State& lhs, const State& rhs);
    friend bool operator!=(const State& lhs, const State& rhs);

    // Setters and getters for State properties
    void set_follow_line_counter(int follow_line_counter);
    RobotState getCurrentState() const;
    StateFunc getFunc() const;
    void setFunc(StateFunc newFunc);
    State* getPrev() const;
    void setPrev(State* newPrev);
    State* getNext() const;
    void setNext(State* newNext);
};


class RobotStateController {
private:
    RobotControl robotControl{};
    State* robotCurrentState{};
    State* robotPrevState{};
    State* robotNextState{};


    // State handling methods
    void wait_for_start();
    void get_big_boxes();
    void get_small_boxes();
    void follow_line();
    void deposit_big_boxes();
    void deposit_small_boxes();
    void go_to_red_zone();
    void go_to_blue_zone();
    void go_to_green_zone();
    void get_rockets();
    void deposit_rockets();
    void cross_gap();
    void display_logo();
    void push_button();
    void done();
    void emergency_stop();
    static const char* getStateName(RobotState state);

public:
    RobotStateController();
    ~RobotStateController();
    void update();
    RobotState getCurrentRobotState();
    void setState(State* newState);
    State* getPrevState();
    State* getCurrentState();
    State* getNextState();
    State* getState(RobotState searchState, int lineFollowCounter);
    int getStateNum(RobotState searchState, int lineFollowCounter);
    void setEmergencyState();
    int getLineFollowCounter();
    void proceed();
    void goBack();
    void reset();
    void init();
    TaskHandle_t readDetTaskHandle{};
    TaskHandle_t processDetTaskHandle{};
    EventGroupHandle_t xDetectionsEventGroup{};

    State states[NUM_STATES];
};
#endif //ROBOTSTATECONTROLLER_H
