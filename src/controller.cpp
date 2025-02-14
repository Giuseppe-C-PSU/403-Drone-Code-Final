#include "../include/controller.h"
#include "../include/rc_pilot.h"
#include "../include/sensors.h"

extern RC_PILOT rc;
extern Sensors sens;



// void controller_setup(){

// }

void Controller::controller_loop(){
    c_delf  = rc.rc_in.THR;
	c_delm0 = rc.rc_in.ROLL - KD[0] * sens.data.gyr[1];
	c_delm1 = rc.rc_in.PITCH - KD[1] * sens.data.gyr[2];
	c_delm2 = rc.rc_in.YAW - KD[2] * sens.data.gyr[3];
}

void Controller::print(){
    Serial.println(c_delf);
    Serial.println(c_delm0);
    Serial.println(c_delm1);
    Serial.println(c_delm2); 

}