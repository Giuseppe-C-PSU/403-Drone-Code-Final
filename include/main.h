#ifndef MAIN_H
#define MAIN_H

#include "motors.h"

#define HAVE_DATALINK     0
#define HAVE_IMU          1
#define HAVE_RC_RECEIVER  0
#define HAVE_MOTORS       0
#define HAVE_THERMAL      1
#define HAVE_PRINTS       0 // MUST BE ON FOR DRONE TO FLY WITH CONTROLLER (ROLL ONLY I THINK!?!?) :) (*Shrug*)

#define HAVE_EKF          0

const unsigned long intervalIMU = 10;
const unsigned long intervalRC = 10;
const unsigned long intervalMotors = 10;
const unsigned long intervalDatalink = 20;
const unsigned long intervalThermal = 200;
const unsigned long intervalEKF = 10;

unsigned long previousMillisIMU = 0;
unsigned long previousMillisRC = 0;
unsigned long previousMillisMotors = 0;
unsigned long previousMillisDatalink = 0;
unsigned long previousMillisThermal = 0;
unsigned long previousMillisEKF = 0;

unsigned long previousMillis = 0;
const long interval = 500;

uint16_t pwm[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};
uint16_t MotorDataGCS[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};

#endif