#include "Motorclock.h"
#include "Buzzer.h"
#include "Servo.h"
#include <Arduino.h>
#include "Stepper.h"
using namespace std;
int clickCount;
int timeCount;
int lastTurnTime = 0;
int alarmHour = -1;
int alarmMinutes = -1;
bool alarmSet = false;
const int stepsPerRevolution = 512;
// Constructor


    void button1Interrupt() {
      timeCount++;
    }
    void button2Interrupt() {
      clickCount++;
    }

Motorclock:: Motorclock(int hour, int minutes, uint8_t servoP, uint8_t stepperN1P, uint8_t
    stepperN2P, uint8_t stepperN3P, uint8_t stepperN4P, uint8_t buzzerP, uint8_t
    button1P, uint8_t button2P, uint8_t rgbRP, uint8_t rgbGP, uint8_t rgbBP) : stepper(512, stepperN1P, stepperN2P, stepperN3P, stepperN4P)

    {
    this->hour = hour;
    this->minutes = minutes;
    this->servoP = servoP;
    this->stepperN1P = stepperN1P;
    this->stepperN2P = stepperN2P;
    this->stepperN3P = stepperN3P;
    this->stepperN4P = stepperN4P;
    this->buzzerP = buzzerP;
    this->button1P = button1P;
    this->button2P = button2P;
    this->rgbRP = rgbRP;
    this->rgbGP = rgbGP;
    this->rgbBP = rgbBP;
    }

    void Motorclock::setup() {
      stepper.setSpeed(5);
      Serial.begin(9600);
        pinMode(button1P, INPUT);
    pinMode(button2P, INPUT);

    pinMode(rgbRP, OUTPUT);
    pinMode(rgbGP, OUTPUT);
    pinMode(rgbBP, OUTPUT);

    pinMode(servoP, OUTPUT);

    pinMode(stepperN1P, OUTPUT);
    pinMode(stepperN2P, OUTPUT);
    pinMode(stepperN3P, OUTPUT);
    pinMode(stepperN4P, OUTPUT);

    pinMode(buzzerP, OUTPUT);
      attachInterrupt(digitalPinToInterrupt(button1P), button1Interrupt, FALLING);
      attachInterrupt(digitalPinToInterrupt(button2P), button2Interrupt, FALLING);

    servo.attach(servoP); 
    }
    String Motorclock::showTime() {
      int currentHour = ((millis() / (60000 * 60)) + hour) % 24;
      int currentMinutes =  ((millis() / 60000) % 60 + minutes) % 60;
      stepper.step(currentMinutes * stepsPerRevolution / 60);
      lastTurnTime = stepsPerRevolution * currentMinutes/ 60;
      servo.write((currentHour * 60 + currentMinutes) * 180 / 3600);

      String time = String(currentHour) + ":" + String(currentMinutes);
      return time;
    }
    
    void Motorclock::setTimer(double minutes) {
      digitalWrite(rgbRP, HIGH);
      digitalWrite(rgbGP, LOW);
      digitalWrite(rgbBP, LOW);
      delay(60000 * minutes);
      int currentCounter = clickCount;
      while (currentCounter == clickCount) {
           tone(buzzerP, 85);
      }
      noTone(buzzerP);
    } 

    void Motorclock::setTimerByHand() {
        int minutes; 
        int currentCounter = clickCount;
        int timeCounter = timeCount;
        while (currentCounter == clickCount) {
            delay(10);
        }
        
        setTimer(timeCount - timeCounter);
    }
    
    void Motorclock::setAlarmByHand() 
    {
      
      int currentCounter = clickCount;
      int timeCounter = timeCount;
      while(currentCounter==clickCount)
      {
        delay(10);
      }
      int hourCount = timeCount-timeCounter;

      currentCounter=clickCount;
      timeCounter=timeCount;
      while(currentCounter==clickCount)
      {
        delay(10);
      }
      int minutesCount= ((timeCount - timeCounter) * 10) % 60 ;
      
      setAlarm(hourCount,minutesCount);

    }

    void Motorclock::setAlarm(int targetHour, int targetMinutes)
     {
       digitalWrite(rgbRP, LOW);
      digitalWrite(rgbGP, HIGH);
      digitalWrite(rgbBP, LOW);
        alarmHour = targetHour;
        alarmMinutes = targetMinutes;
        alarmSet = true;
     
    }


    void Motorclock::checkAlarm() {
      if (!alarmSet)
        return;

      int currentHour = (millis() / (60000 * 60)) + hour;
      int currentMinutes = ((millis() / 60000) % 60 + minutes) % 60;
      if (alarmHour == currentHour && alarmMinutes == currentMinutes ) {
        int currentCounter = clickCount;
        while (currentCounter == clickCount) {
           tone(buzzerP, 85);
        }
      noTone(buzzerP);
      alarmSet = false;
      }
    }

    double Motorclock::setStopWatch() 
    {
       digitalWrite(rgbRP, LOW);
      digitalWrite(rgbGP, LOW);
      digitalWrite(rgbBP, HIGH);
      long now = millis();
      double degree;
      int currentCounter = clickCount;
      while(currentCounter == clickCount)
      {
        tone(buzzerP,85);
        
      }
      noTone(buzzerP);
      int mins = ((millis() - now) / 60000);
      

      stepper.step(-lastTurnTime);
      int mapped = 2048 / 60 * mins;
      stepper.step(mapped);
      delay(1000);
      stepper.step(-mapped);
      stepper.step(lastTurnTime);
     
      return mins;
    }
     int Motorclock::getCurrentHour() {
      int currentHour = millis() / (60000 * 60);
      return (hour + currentHour) % 24;
    }
     int Motorclock::getCurrentMinutes() {
      int currentMinutes = (millis() / 60000) % 60;
      return (minutes + currentMinutes) % 24;
    }
    double Motorclock::setStopWatchByHand() 
    {
      int timeCounter = timeCount;
      while (timeCounter == timeCount) {
      }
      return(setStopWatch());
    }



