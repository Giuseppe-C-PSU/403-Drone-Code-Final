#include "controller.h"
#include "rc_pilot.h"
#include "sensors.h"

#define DEAD_BAND 0.05

#define MIN_PWM 900
#define MAX_PWM 1900
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

extern RC_PILOT rc;
extern Sensors sens;




float Controller::applyDeadband(float value, float deadband) {
    if (fabs(value) < deadband) {
        return 0.0;
    }
    return value;
}

float Controller::movingAverage(float newValue, float* buffer, int bufferSize) {
    float sum = 0.0;
    for (int i = bufferSize - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
        sum += buffer[i];
    }
    buffer[0] = newValue;
    sum += newValue;
    return sum / bufferSize;
}

void Controller::controller_loop() {
    if (c_delf == nullptr) {
        c_delf = new float;
        c_delm0 = new float;
        c_delm1 = new float;
        c_delm2 = new float;
    }

    *c_delf  = ((rc.rc_in.THR - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX / 4.0));
    *c_delm0 = ((rc.rc_in.ROLL - rc.rc_in.ROLL_MID) / (rc.rc_in.ROLL_MAX / 4.0) - KD[0] * sens.data.gyr[0]) + TRIM[0];
    *c_delm1 = ((rc.rc_in.PITCH - rc.rc_in.PITCH_MID) / (rc.rc_in.PITCH_MAX / 4.0) + KD[1] * sens.data.gyr[1]) + TRIM[1];
    *c_delm2 = ((rc.rc_in.YAW - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX / 4.0) - KD[2] * sens.data.gyr[2]) + TRIM[2];

    *c_delf  = applyDeadband(movingAverage(*c_delf, thrBuffer, 10), DEAD_BAND);
    *c_delm0 = applyDeadband(movingAverage(*c_delm0, rollBuffer, 10), DEAD_BAND);
    *c_delm1 = applyDeadband(movingAverage(*c_delm1, pitchBuffer, 10), DEAD_BAND);
    *c_delm2 = applyDeadband(movingAverage(*c_delm2, yawBuffer, 10), DEAD_BAND);

    mixer();
}


// void controller_setup(){

// }

// void Controller::controller_loop() {
//     // Normalize the input values to [-1, 1]
//     // c_delf  = (rc.rc_in.THR - 1500) / 500.0;
//     // c_delm0 = (rc.rc_in.ROLL - 1500) / 500.0;
//     // c_delm1 = (rc.rc_in.PITCH - 1500) / 500.0;
//     // c_delm2 = (rc.rc_in.YAW - 1500) / 500.0;

//     // c_delf  = ((static_cast<int>(rc.rc_in.THR) + TC[0] - 1500) / 500.0 - KD[0]) + TRIM[0];
//     // c_delm0 = ((static_cast<int>(rc.rc_in.ROLL) + TC[1] - 1500) / 500.0 - KD[1] * sens.data.gyr[1]) + TRIM[1];
//     // c_delm1 = ((static_cast<int>(rc.rc_in.PITCH) + TC[2] - 1500) / 500.0 - KD[2] * sens.data.gyr[2]) + TRIM[2];
//     // c_delm2 = ((static_cast<int>(rc.rc_in.YAW) + TC[3] - 1500) / 500.0 - KD[3] * sens.data.gyr[3]) + TRIM[3] ;

//     // c_delf  = ((static_cast<int>(rc.rc_in.THR) + TC[0] - 1500) / 500.0) + TRIM[0];
//     // c_delm0 = ((static_cast<int>(rc.rc_in.ROLL) + TC[1] - 1500) / 500.0) + TRIM[1];
//     // c_delm1 = ((static_cast<int>(rc.rc_in.PITCH) + TC[2] - 1500) / 500.0) + TRIM[2];
//     // c_delm2 = ((static_cast<int>(rc.rc_in.YAW) + TC[3] - 1500) / 500.0) + TRIM[3];

//     c_delf  = ( ( (rc.rc_in.THR) - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX/4.0) );
//     c_delm0 = ( ( (rc.rc_in.ROLL) - rc.rc_in.ROLL_MID) / (rc.rc_in.ROLL_MAX/4.0) );
//     c_delm1 = ( ( (rc.rc_in.PITCH) - rc.rc_in.PITCH_MID) / (rc.rc_in.PITCH_MAX/4.0) );
//     c_delm2 = ( ( (rc.rc_in.YAW) - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX/4.0) );


//     // c_delf  = rc.rc_in.THR;
//     // c_delm0 = rc.rc_in.ROLL - KD[0] * sens.data.gyr[1];
//     // c_delm1 = rc.rc_in.PITCH - KD[1] * sens.data.gyr[2];
//     // c_delm2 = rc.rc_in.YAW - KD[2] * sens.data.gyr[3];
//     mixer();
// }

void Controller::print(){
    Serial.print(rc.rc_in.THR);
    Serial.print(", ");
    Serial.print(rc.rc_in.ROLL);
    Serial.print(", ");
    Serial.print(rc.rc_in.PITCH);
    Serial.print(", ");
    Serial.print(rc.rc_in.YAW); 
    Serial.print("\n");

    Serial.print(*c_delf);
    Serial.print(", ");
    Serial.print(*c_delm0);
    Serial.print(", ");
    Serial.print(*c_delm1);
    Serial.print(", ");
    Serial.print(*c_delm2); 
    Serial.print("\n");

    Serial.print(*thr_pwm);
    Serial.print(", ");
    Serial.print(*roll_pwm);
    Serial.print(", ");
    Serial.print(*pitch_pwm);
    Serial.print(", ");
    Serial.print(*yaw_pwm); 
    Serial.print("\n");

    Serial.print(pwmout_0);
    Serial.print(", ");
    Serial.print(pwmout_1);
    Serial.print(", ");
    Serial.print(pwmout_2);
    Serial.print(", ");
    Serial.print(pwmout_3);
    Serial.print("\n");

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

		pwmout_0 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm - *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ) - 100; // front-right CW
		pwmout_1 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm + *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ) - 100; // back-right  CCW
		pwmout_2 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm + *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ) - 100; // back-left   CW
		pwmout_3 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm - *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ) - 100; // front-left CCW


}