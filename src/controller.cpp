#include "controller.h"
#include "rc_pilot.h"
#include "sensors.h"

#define DEAD_BAND 0.05

#define MIN_PWM 1000
#define MAX_PWM 2000
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

const float DEG2RAD_TERM = 3.1415926535897932384626433832795028841/180;

extern RC_PILOT rc;
extern Sensors sens;




float Controller::applyDeadband(float value, float deadband) {
    if (fabs(value) < deadband) {
        return 0.0;
    }
    return value;
}

float Controller::cumulativeMovingAverage(float newValue, float* average, int count) {
    *average = ((*average) * count + newValue) / (count + 1);
    return *average;
}



void Controller::controller_loop() {
    if (c_delf == nullptr) {
        c_delf = new float;
        c_delm0 = new float;
        c_delm1 = new float;
        c_delm2 = new float;
    }


    
    *c_delf  = ( applyDeadband((rc.rc_in.THR - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX / 4.0), DEAD_BAND) );
    *c_delm2 = ( applyDeadband((rc.rc_in.YAW - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX / 4.0), DEAD_BAND) * 0.2 - KD[2] * ( sens.data.gyr[2] * DEG2RAD_TERM) );
    *c_delm0 = ( applyDeadband((rc.rc_in.ROLL - rc.rc_in.ROLL_MID) / (rc.rc_in.ROLL_MAX / 4.0), DEAD_BAND ) * 0.2 - ( sens.data.euler[0] * DEG2RAD_TERM ) * KP[0] ) - KD[0] * ( sens.data.gyr[0] * DEG2RAD_TERM );
	*c_delm1 = ( applyDeadband((rc.rc_in.PITCH - rc.rc_in.PITCH_MID) / (rc.rc_in.PITCH_MAX / 4.0), DEAD_BAND ) * 0.2 + ( sens.data.euler[1] * DEG2RAD_TERM ) * KP[1] ) + KD[1] * ( sens.data.gyr[1] * DEG2RAD_TERM );

    mixer();
}


void Controller::print(){
    // Serial.print(rc.rc_in.THR);
    // Serial.print(", ");
    // Serial.print(rc.rc_in.ROLL);
    // Serial.print(", ");
    // Serial.print(rc.rc_in.PITCH);
    // Serial.print(", ");
    // Serial.print(rc.rc_in.YAW); 
    // Serial.print("\n");

    Serial.print(*c_delf);
    Serial.print(", ");
    Serial.print(*c_delm0);
    Serial.print(", ");
    Serial.print(*c_delm1);
    Serial.print(", ");
    Serial.print(*c_delm2); 
    Serial.println();

    // Serial.print(*thr_pwm);
    // Serial.print(", ");
    // Serial.print(*roll_pwm);
    // Serial.print(", ");
    // Serial.print(*pitch_pwm);
    // Serial.print(", ");
    // Serial.print(*yaw_pwm); 
    // Serial.print("\n");

    // Serial.print(pwmout_0);
    // Serial.print(", ");
    // Serial.print(pwmout_1);
    // Serial.print(", ");
    // Serial.print(pwmout_2);
    // Serial.print(", ");
    // Serial.print(pwmout_3);
    // Serial.print("\n");

}

void Controller::mixer(){
    if(thr_pwm == nullptr) {
        thr_pwm = new float;
        roll_pwm = new float;
        pitch_pwm = new float;
        yaw_pwm = new float;
    }

    		// only throttle and roll
		*thr_pwm = ( *c_delf - ( -1 ) ) * ( 2000 - 1000 ) / ( 1 - ( -1 ) ) + 1000; // from [-1,1] to [1000,2000]
		*roll_pwm = ( *c_delm0 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
		*pitch_pwm = ( *c_delm1 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
		*yaw_pwm = ( *c_delm2 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]

        // float thr_pwm = c_delf;
		// float roll_pwm = c_delm0;
		// float pitch_pwm = c_delm1;
		// float yaw_pwm = c_delm2;

		pwmout_0 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm - *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ); // front-right CW
		pwmout_1 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm + *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ); // back-right  CCW
		pwmout_2 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm + *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ); // back-left   CW
		pwmout_3 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm - *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ); // front-left CCW


}