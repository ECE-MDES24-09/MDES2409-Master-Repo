#ifndef TimeManagement_h
#define TimeManagement_h

#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <timers.h>
#include <globalVariables.h>
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
  TimerHandle_t* _stateTimers;
  TimerHandle_t _bufferTimer;
};

#endif
