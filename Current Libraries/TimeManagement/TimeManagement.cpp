#include "TimeManagement.h"

void StateTimerCallback(TimerHandle_t xTimer);
void BufferTimerCallback(TimerHandle_t xTimer);



TimeManagement::TimeManagement(): _isStartStateCompleted(false) {
	bool maxSuff = isMaxTimeSufficient();
	Serial.println(maxSuff);
   _stateTimers = new TimerHandle_t[19];
   _bufferTimer = NULL;
   _timeBank = 0;

}


TimeManagement::~TimeManagement() {
	for (int i = 0; i < _numStates; ++i) {
		xTimerDelete(_stateTimers[i], portMAX_DELAY);
    }
    delete[] _stateTimers;
    delete[] _stateTimeLimits;
}


bool TimeManagement::isMaxTimeSufficient() const {
    long totalStateTime = 0;
    for (int i = 0; i < _numStates; ++i) {
        totalStateTime += _stateTimeLimits[i];
    }
    return _maxTime >= totalStateTime;
}

void TimeManagement::startTimer() {
	_startTime = millis();
}

void TimeManagement::startState(int stateId) {
	_stateStartTime = millis();
}


void TimeManagement::endState(int stateId,  bool allocate) {
	Serial.print("Hi End ");
	Serial.println(stateId);
	_stateEndTime = millis();
	long elapsedTime = _stateEndTime - _stateStartTime;
	Serial.print("Elapsed Time for Task: ");
	Serial.println(elapsedTime);
	Serial.print("Max Time for Task: ");
	Serial.println(_stateTimeLimits[stateId]);
	long addToBank = _stateTimeLimits[stateId] - elapsedTime;
	if (allocate) {
		_timeBank += addToBank;
		allocateTime(stateId);
	}
}



void TimeManagement::allocateTime(int stateId) {
	Serial.println("Hi Allocate");
	Serial.print("Timebank: ");
	Serial.println(_timeBank);
}


long TimeManagement::getRemainingTimeForState(int stateId) const {
	// Return the remaining time for a specific state
	long stateCurrentTime = millis();

	long stateElapsedTime = stateCurrentTime - _stateStartTime;
	
	return _stateTimeLimits[stateId] - stateElapsedTime;
}

long TimeManagement::getRemainingTime() const {
	long currentTime = millis();

	return currentTime - _startTime;
}

long TimeManagement::getTimeBank() const {
		Serial.println("Hi GetBank");
    return _timeBank;
}