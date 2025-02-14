// #include "../include/controller.h"
// #include "../include/rc_pilot.h"
// #include "../include/sensors.h"

// extern RC_PILOT rc;
// extern Sensors sens;

// uint16_t KD[3] = {1,1,1};

// // void controller_setup(){

// // }

// void controller_loop(){
//     int16_t c_delf  = rc.rc_in.THR;
// 	int16_t c_delm0 = rc.rc_in.ROLL - KD[0] * sens.data.gyr[1];
// 	int16_t c_delm1 = rc.rc_in.PITCH - KD[1] * sens.data.gyr[2];
// 	int16_t c_delm2 = rc.rc_in.YAW - KD[2] * sens.data.gyr[3];


// }