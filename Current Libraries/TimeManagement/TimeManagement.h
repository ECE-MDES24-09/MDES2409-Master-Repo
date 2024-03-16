#ifndef TimeManagement_h
#define TimeManagement_h

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <timers.h>
<<<<<<< HEAD
#include <globalVariables.h>
=======

>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c
class TimeManagement {
public:
   TimeManagement::TimeManagement(); // Constructor
  ~TimeManagement(); // Destructor
  void TimeManagement::startState(int stateId);
  void TimeManagement::endState(int stateId, bool allocate);
  void TimeManagement::allocateTime(int stateId);
  long TimeManagement::getRemainingTime() const; 
  void TimeManagement::setTimeForState(int state, long time); 
  long TimeManagement::getRemainingTimeForState(int stateId) const; 
  bool TimeManagement::isMaxTimeSufficient() const;
  void TimeManagement::startTimer();
  long TimeManagement::getTimeBank() const;


private:
<<<<<<< HEAD
=======
  // WAIT_FOR_START ID == 0. It does not count toward timer.
  long _startTime;
  long _maxTime = 105000;
  long _stateTimeLimits[19] = { 0, 10000, 10000, 5000, 4000, 1500, 2000, 1500, 5000, 3000, 10000, 5000, 15000, 5000, 15000, 2000, 3000, 0, 0 }; 
  int _numStates = 19;
  int _currentState; 
  long _stateStartTime;
  long _stateEndTime;
  long _currentstateMaxTime;
  long _timeBank;
  const long STATE_SWITCH_BUFFER = 500; // 0.5 seconds in milliseconds
  bool _isStartStateCompleted;
  const int WAIT_FOR_START_STATE_ID = 0;
>>>>>>> 1b510070ff16c63498ab9274048c425d8414488c
  TimerHandle_t* _stateTimers;
  TimerHandle_t _bufferTimer;
};

#endif
