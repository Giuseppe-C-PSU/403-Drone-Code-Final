#include "controller.h"
#include "rc_pilot.h"
#include "sensors.h"
#include "datalink.h"


#define DEAD_BAND 0.05

#define MIN_PWM 1000
#define MAX_PWM 2000
#define LIMIT(x,xl,xu) ((x)>=(xu)?(xu):((x)<(xl)?(xl):(x)))

const float DEG2RAD_TERM = 3.1415926535897932384626433832795028841/180;

extern RC_PILOT rc;
extern Sensors sens;

struct obDatalink_ref* ob = &obDatalink;
struct datalinkMessageHITLSim2Onboard_ref* s2o = ob->sim2onboard;


// Prelim Functions
float Controller::applyDeadband(float value, float deadband) {
    if (fabs(value) < deadband) {
        return 0.0;
    }
    return value;
}

float mapStickToAngle(float stick_value, float stick_min, float stick_mid, float stick_max, float angle_max) {
    float angle;
    if (stick_value < stick_mid) {
        angle = -angle_max * (stick_mid - stick_value) / (stick_mid - stick_min);
    } else {
        angle = angle_max * (stick_value - stick_mid) / (stick_max - stick_mid);
    }
    return angle;
}

float mapStickToRate(float stick_value, float stick_min, float stick_mid, float stick_max, float rate_max) {
    float rate;
    if (stick_value < stick_mid) {
        rate = -rate_max * (stick_mid - stick_value) / (stick_mid - stick_min);
    } else {
        rate = rate_max * (stick_value - stick_mid) / (stick_max - stick_mid);
    }
    return rate;
}

// Controller
void Controller::controller_loop(bool value){
    if(value){
        angle_p[0] = s2o->k_p_x;
        angle_p[1] = s2o->k_p_y;
        angle_i[0] = s2o->k_i_x;
        angle_i[1] = s2o->k_i_y;
        angle_d[0] = s2o->k_d_x;
        angle_d[1] = s2o->k_d_y;
    }

    //Reciever Code:
    c_delf = applyDeadband((rc.rc_in.THR - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX / 4.0), DEAD_BAND);
    c_delm2 = applyDeadband((rc.rc_in.YAW - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX / 4.0), DEAD_BAND) * 0.2;
    
    angle_controller_loop();
    mixer();
}

void Controller::angle_controller_loop(){
    angle_des[0] = DEG2RAD_TERM * mapStickToAngle(rc.rc_in.ROLL, rc.rc_in.ROLL_MIN, rc.rc_in.ROLL_MID, rc.rc_in.ROLL_MAX, max_angle);
    angle_des[1] = DEG2RAD_TERM * mapStickToAngle(rc.rc_in.PITCH, rc.rc_in.PITCH_MIN, rc.rc_in.PITCH_MID, rc.rc_in.PITCH_MAX, max_angle);

    for(int i = 0; i < 2; i++){
        // Calculate errors
        angle_err[i] = angle_des[i] - (sens.data.euler[i] * DEG2RAD_TERM);

        // Calculate Integrals
        if(rc.rc_in.AUX2 > 1500){
            angle_int_err[i] += angle_err[i] * angle_dt;
        }else{
            angle_int_err[i] = 0;
        }

        // P Controller
        angle_con[i] = (angle_p[i] * angle_err[i]);

        // I Controller
        angle_con[i] += (angle_i[i] * angle_int_err[i]);
    
        // D Controller
        angle_con[i] += angle_d[i] * (-1.0 * sens.data.gyr[i] * DEG2RAD_TERM);
    }

    // Add to c_delm
    c_delm0 = angle_con[0];
    c_delm1 = angle_con[1];
}


void Controller::mixer(){
    if(thr_pwm == nullptr) 
    {
        thr_pwm = new float;
        roll_pwm = new float;
        pitch_pwm = new float;
        yaw_pwm = new float;
    }
    
    	// only throttle and roll
		*thr_pwm = ( c_delf - ( -1 ) ) * ( 2000 - 1000 ) / ( 1 - ( -1 ) ) + 1000; // from [-1,1] to [1000,2000]
		*roll_pwm = ( c_delm0 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
		*pitch_pwm = ( c_delm1 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]
		*yaw_pwm = ( c_delm2 - ( -1 ) ) * ( 500 - ( -500 ) ) / ( 1 - ( -1 ) ) + ( -500 ); // from [-1,1] to [-500,500]

		pwmout_0 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm - *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ); // front-right CW
		pwmout_1 = ( unsigned short ) LIMIT( *thr_pwm - *roll_pwm + *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ); // back-right  CCW
		pwmout_2 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm + *pitch_pwm + *yaw_pwm, MIN_PWM, MAX_PWM ); // back-left   CW
		pwmout_3 = ( unsigned short ) LIMIT( *thr_pwm + *roll_pwm - *pitch_pwm - *yaw_pwm, MIN_PWM, MAX_PWM ); // front-left CCW

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

    Serial.print(angle_des[1]);
    Serial.print(", ");
    Serial.print(angle_err[1]);
    Serial.print(", ");
    Serial.print(angle_int_err[1]);
    Serial.print(", ");
    Serial.print(angle_con[1]);
    Serial.println();

    // Serial.print(c_delf);
    // Serial.print(", ");
    // Serial.print(c_delm0);
    // Serial.print(", ");
    // Serial.print(c_delm1);
    // Serial.print(", ");
    // Serial.print(c_delm2); 
    // Serial.println();

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
