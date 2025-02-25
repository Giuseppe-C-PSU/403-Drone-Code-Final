#include <Arduino.h>


#include "rc_pilot_reading.h"
#include "motors.h"
#include "rc_pilot.h"
#include "sensor_prelim.h"
#include "controller.h"


Motors motors;
extern RC_PILOT rc;
Controller cntrl;

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
    rc_setup();

    pozyx_setup();

    motors.init();

    // thermal_setup();

    pinMode(LED_BUILTIN, OUTPUT);

}


void loop()
{

    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    rc_reciever_loop();
    // delay(1000);

    if(rc.rc_in.AUX2 > 1500)
    {
      // pwm[0] = rc.rc_in.THR;
      // pwm[1] = rc.rc_in.THR;
      // pwm[2] = rc.rc_in.THR;
      // pwm[3] = rc.rc_in.THR;

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

    // Serial.print("Pozyx\n");
    pozyx_loop();
    // // Serial.print("\n");
    // // delay(1000);

    cntrl.controller_loop();

    motors.update(pwm);

    // cntrl.print();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        cntrl.print();
        // rc.print();
        //sens.print();
    }


    // Serial.print("Thermal Camera\n");
    // thermal_loop();
    // Serial.print("\n");
    delay(10);

    
}