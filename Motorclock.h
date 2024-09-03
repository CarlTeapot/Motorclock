#ifndef Motorclock_h
#define Motorclock_h
#include <stdint.h> 
#include <Arduino.h>
#include "Buzzer.h"
#include "Servo.h"
#include "Stepper.h"
using namespace std;

class Motorclock {
private:
  int hour;
    int minutes;
    uint8_t servoP;
    uint8_t stepperN1P;
    uint8_t stepperN2P;
    uint8_t stepperN3P;
    uint8_t stepperN4P;
    uint8_t buzzerP;
    uint8_t button1P;
    uint8_t button2P;
    uint8_t rgbRP;
    uint8_t rgbGP;
    uint8_t rgbBP;
    Servo servo;
    Stepper stepper;
    int getCurrentHour();
    int getCurrentMinutes();
public:
   Motorclock(int hour, int minutes, uint8_t servoP, uint8_t stepperN1P, uint8_t
    stepperN2P, uint8_t stepperN3P, uint8_t stepperN4P, uint8_t buzzerP, uint8_t
    button1P, uint8_t button2P, uint8_t rgbRP, uint8_t rgbGP, uint8_t rgbBP);

     String showTime();
    void setup();
    void setTimer(double minutes);

    void setTimerByHand();
    
    void setAlarmByHand();

    void setAlarm(int hour, int minutes);

    void checkAlarm();

    double setStopWatch();

    double setStopWatchByHand();
};

#endif