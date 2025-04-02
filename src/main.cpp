#include <Arduino.h>


#include "rc_pilot_reading.h"
#include "motors.h"
#include "rc_pilot.h"
#include "sensor_prelim.h"
#include "controller.h"
#include "datalink.h"
#include "EKF.h"
#include "sensors.h"



#define HAVE_DATALINK     1
#define HAVE_IMU          1
#define HAVE_RC_RECEIVER  1
#define HAVE_MOTORS       1
#define HAVE_THERMAL      0
#define HAVE_PRINTS       0
#define HAVE_EKF          0
#define USE_RC_IN_PACE    1

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

float dt = 0.01;

Motors motors;
extern RC_PILOT rc;
Controller cntrl;
Dlink datalink;
wifi ether;
EKF ekf;
extern Sensors sens;

unsigned long previousMillis = 0;
const long interval = 500;


uint16_t pwm[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};
uint16_t MotorDataGCS[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};

void setup()
{
  // initialize peripherals 
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.print("Setup has started");
  
    #if HAVE_RC_RECEIVER
      rc_setup();
    #endif

    #if HAVE_IMU
      pozyx_setup();
    #endif

    #if HAVE_MOTORS
      motors.init();
    #endif

    #if HAVE_DATALINK
    datalink.init();
    #endif

    #if HAVE_THERMAL
      thermal_setup();
    #endif

    pinMode(LED_BUILTIN, OUTPUT);

}


void loop()
{

  /* The order should be:
  1-) Read all sensors and data
  2-) Perform calculations
  3-) Motor actuation
  4-) Update data on GCS
  */
 // sensor-> rc->thermal->->motors

  unsigned long currentMillis = millis();
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  #if HAVE_IMU

  if (currentMillis - previousMillisIMU >= intervalIMU) {
    previousMillisIMU = currentMillis;
    pozyx_loop(); 
  }

    #if HAVE_EKF
      ekf.predict(dt);
    #endif

  #endif 

  #if HAVE_THERMAL
    thermal_loop();
  #endif

  #if HAVE_RC_RECEIVER
  if (currentMillis - previousMillisRC >= intervalRC) {
    previousMillisRC = currentMillis;
    rc.update();

    if(rc.rc_in.AUX2 > 1500)
    {
      pwm[0] = cntrl.pwmout_3;
      pwm[1] = cntrl.pwmout_0;
      pwm[2] = cntrl.pwmout_2;
      pwm[3] = cntrl.pwmout_1;
    }
    else{
      pwm[0] = 900;
      pwm[1] = 900;
      pwm[2] = 900;
      pwm[3] = 900;
    } 
  }
  #endif

  #if HAVE_MOTORS
  if (currentMillis - previousMillisMotors >= intervalMotors) {
    previousMillisMotors = currentMillis;
    
    #if USE_RC_IN_PACE
      cntrl.controller_loop(1);
    #else
      cntrl.controller_loop(0);
    #endif
    motors.update(pwm);
  }
  #endif

  #if HAVE_DATALINK
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    datalink.recv_update();
    #if USE_RC_IN_PACE
      datalink.send_update(true);
    #else
      datalink.send_update(false);
    #endif
  #endif

  #if HAVE_EKF
    if (currentMillis - previousMillisEKF >= intervalEKF)
    {
      previousMillisEKF = currentMillis;
      struct obDatalink_ref* ob = &obDatalink;
      struct datalinkMessageOptitrack_ref* mocap = ob->optitrack;
      float z[MEAS_DIM] = {sens.data.quat[0],sens.data.quat[1],sens.data.quat[2],sens.data.quat[3],mocap->pos_x,mocap->pos_y,mocap->pos_z};
      // Serial.print('z = ');
      // Serial.print(z[0]);
      // Serial.print(',');
      // Serial.print(z[1]);
      // Serial.print(',');
      // Serial.print(z[2]);
      // Serial.print(',');
      // Serial.print(z[3]);
      // Serial.println();
      
      // Serial.print(mocap->pos_x);
      // Serial.print(",\t");
      // Serial.print(mocap->pos_y);
      // Serial.print(",\t");
      // Serial.print(mocap->pos_z);
      // Serial.print("\n");
  
      ekf.update(z);
    }
  #endif

  #if HAVE_PRINTS
    // rc.print();
    // ekf.printState();
    // cntrl.print();
    sens.print();
  #endif
  

  
}