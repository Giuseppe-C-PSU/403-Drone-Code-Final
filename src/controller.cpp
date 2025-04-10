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



struct obDatalink_ref* ob = &obDatalink;
struct datalinkMessageHITLSim2Onboard_ref* s2o = ob->sim2onboard;

void Controller::controller_loop(int value){
    if(value == 1){
        have_rc_in_pace = 1;
    }else{
        have_rc_in_pace = 0;
    }

    //Reciever Code:
    c_delf = applyDeadband((rc.rc_in.THR - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX / 4.0), DEAD_BAND);
    // c_delm0 = applyDeadband((rc.rc_in.ROLL - rc.rc_in.ROLL_MID) / (rc.rc_in.ROLL_MAX / 4.0), DEAD_BAND );
    // c_delm1 = applyDeadband((rc.rc_in.PITCH - rc.rc_in.PITCH_MID) / (rc.rc_in.PITCH_MAX / 4.0), DEAD_BAND );
    c_delm2 = applyDeadband((rc.rc_in.YAW - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX / 4.0), DEAD_BAND) * 0.2;

    //rate_controller_loop();
    angle_controller_loop();
    mixer();
}

void Controller::angle_controller_loop(){
    float desired_roll = DEG2RAD_TERM * mapStickToAngle(rc.rc_in.ROLL, rc.rc_in.ROLL_MIN, rc.rc_in.ROLL_MID, rc.rc_in.ROLL_MAX, max_angle);
    float desired_pitch = DEG2RAD_TERM * mapStickToAngle(rc.rc_in.PITCH, rc.rc_in.PITCH_MIN, rc.rc_in.PITCH_MID, rc.rc_in.PITCH_MAX, max_angle);
    // desired_roll = c_delm0;
    // desired_pitch = c_delm1;

    if(rc.rc_in.AUX2 > 1500){
        //Calculate error integrals
        angle_integral_error[0] += roll_error * angle_dt;
        angle_integral_error[1] += pitch_error * angle_dt;
    }else{
        angle_integral_error[0] = 0;
        angle_integral_error[1] = 0;
    }

    roll_error = desired_roll - (sens.data.euler[0] * DEG2RAD_TERM);
    pitch_error = desired_pitch - (sens.data.euler[1] * DEG2RAD_TERM);

    

    // P Controller:
    float desired_angle_roll = (s2o->k_p_x * roll_error);
    float desired_angle_pitch = (s2o->k_p_y * pitch_error);

    // I Controller:
    desired_angle_roll += (s2o->k_i_x * angle_integral_error[0]);
    desired_angle_pitch += (s2o->k_i_y * angle_integral_error[1]);

    // D Controller:
    desired_angle_roll += s2o->k_d_x * (sens.data.gyr[0] * DEG2RAD_TERM);
    desired_angle_pitch += s2o->k_d_y * (sens.data.gyr[1] * DEG2RAD_TERM);

    // Add to c_delf:
    c_delm0 = desired_angle_roll;
    c_delm1 = desired_angle_pitch;

}

void Controller::rate_controller_loop() {

    
    if (have_rc_in_pace == 1)
    {

        //Integral Error:
        if(rc.rc_in.AUX2 > 1500){
            for(int i = 0; i < 3; i++)
            {
            rate_integral_error[i] += (sens.data.euler[i] * DEG2RAD_TERM )* rate_dt;
            }
        }else{
            for(int i = 0; i < 3; i++)
            {
            rate_integral_error[i] = 0;
            }
        }

        float roll_des = mapStickToRate(c_delm0,rc.rc_in.ROLL_MIN,rc.rc_in.ROLL_MID,rc.rc_in.ROLL_MAX,15);
        float pitch_des = mapStickToRate(c_delm1,rc.rc_in.PITCH_MIN,rc.rc_in.PITCH_MID,rc.rc_in.PITCH_MAX,15);
        float yaw_des = mapStickToRate(c_delm2,rc.rc_in.YAW_MIN,rc.rc_in.YAW_MID,rc.rc_in.YAW_MAX,15);


        if(sens.data.gyr[0] > 70){
            sens.data.gyr[0] = 70;
        }else if(sens.data.gyr[0] < -70){
            sens.data.gyr[0] = -70;
        }

        float roll_err = roll_des - sens.data.gyr[0];
        float pitch_err = pitch_des - sens.data.gyr[1];
        float yaw_err = yaw_des - sens.data.gyr[2];

        // P-Gain:
        c_delf  = c_delf;
        c_delm0 = (roll_err * DEG2RAD_TERM) * s2o->k_p_x;
        c_delm1 = c_delm1 + ( sens.data.euler[1] * DEG2RAD_TERM ) * s2o->k_p_y;
        c_delm2 = c_delm2;

        // I-Gain:
        c_delf = c_delf;
        c_delm0 = c_delm0 - s2o->k_i_x * rate_integral_error[0];
        c_delm1 = c_delm1 + s2o->k_i_y * rate_integral_error[1];
        c_delm2 = c_delm2;

        // D_Gain:
        c_delf = c_delf;
        c_delm0 = c_delm0 - s2o->k_d_x * ( sens.data.gyr[0] * DEG2RAD_TERM );
        c_delm1 = c_delm1 + s2o->k_d_y * ( sens.data.gyr[1] * DEG2RAD_TERM );
        c_delm2 = c_delm2;


    }else{
        c_delf  = ( applyDeadband((rc.rc_in.THR - rc.rc_in.THR_MID) / (rc.rc_in.THR_MAX / 4.0), DEAD_BAND) );
        c_delm0 = ( applyDeadband((rc.rc_in.ROLL - rc.rc_in.ROLL_MID) / (rc.rc_in.ROLL_MAX / 4.0), DEAD_BAND ) * 0.02 - ( s2o->nav_phi ) * KP[0] ) - KD[0] * ( s2o->nav_w_x );
        c_delm1 = ( applyDeadband((rc.rc_in.PITCH - rc.rc_in.PITCH_MID) / (rc.rc_in.PITCH_MAX / 4.0), DEAD_BAND ) * 0.02 + ( s2o->nav_theta  ) * KP[1] ) + KD[1] * ( s2o->nav_w_y );
        c_delm2 = ( applyDeadband((rc.rc_in.YAW - rc.rc_in.YAW_MID) / (rc.rc_in.YAW_MAX /    4.0), DEAD_BAND) );
    }
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

    Serial.print(c_delf);
    Serial.print(", ");
    Serial.print(c_delm0);
    Serial.print(", ");
    Serial.print(c_delm1);
    Serial.print(", ");
    Serial.print(c_delm2); 
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
