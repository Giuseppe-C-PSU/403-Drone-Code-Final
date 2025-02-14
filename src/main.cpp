#include <Arduino.h>


#include "../include/rc_pilot_reading.h"
#include "../include/motors.h"
#include "../include/rc_pilot.h"
#include "../include/sensor_prelim.h"
#include "../include/controller.h"

Motors motors;
extern RC_PILOT rc;
Controller cntrl;

unsigned long previousMillis = 0;
const long interval = 500;


uint16_t pwm[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};
uint16_t MotorDataGCS[4] = {MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT,MIN_PWM_OUT};

void setup()
{

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

    cntrl.controller_loop();

    if(rc.rc_in.AUX2 > 1500)
    {
      // pwm[0] = rc.rc_in.THR;
      // pwm[1] = rc.rc_in.THR;
      // pwm[2] = rc.rc_in.THR;
      // pwm[3] = rc.rc_in.THR;

      pwm[0] = 1050;
      pwm[1] = 1050;
      pwm[2] = 1050;
      pwm[3] = 1050;
    }
    else{
      pwm[0] = 900;
      pwm[1] = 900;
      pwm[2] = 900;
      pwm[3] = 900;
    } 


    motors.update(pwm);
  

    cntrl.controller_loop();
    

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        cntrl.print();
    }

    // Serial.print("Pozyx\n");
    pozyx_loop();
    // Serial.print("\n");
    // delay(1000);


    // Serial.print("Thermal Camera\n");
    // thermal_loop();
    // Serial.print("\n");
    delay(10);

    
}