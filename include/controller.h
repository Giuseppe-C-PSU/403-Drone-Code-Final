#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifndef C_PI
#define C_PI 3.14159265358979323846264338327950288419716939937511
#endif

#include <cstdint>

class Controller {
private:

    
    // ~~~~~~~~~~~~~~~~ GAINS ~~~~~~~~~~~~~~~~~~~~
    // Arduino Gains
    // float KD[3] = {0.1, 0.1, 0.1};
    // float KP[3] = {0.13, 0.13, 0.13};

    // Pace Gains:
    // float KD[3] = {0.1, 0.1, 0.1};
    // float KP[3] = {0.05, 0.05, 0.05};

    // Pace Attitude Gains
    float KD[3] = {0.1, 0.1, 0.1};
    float KP[3] = {0.05, 0.05, 0.5};
    float KI[3] = {0.01, 0.01, 0.01};
    

    // ~~~~~~~~~~~~~~~~ CONTROLLERS ~~~~~~~~~~~~~~~~~~~

    // Rate controller vars

    bool have_rc_in_pace;

    float applyDeadband(float value, float deadband);

    double hmodRad(double h) {

        double dh;
        int i;
    
        if (h > 0)
            i = (int)(h / (2 * C_PI) + 0.5);
        else
            i = (int)(h / (2 * C_PI) - 0.5);
        dh = h - C_PI * 2 * i;
    
        return dh;
    }

    // Angle controller vars
    float angle_des[2] = {0,0}; // Desired angle from controller
    float angle_err[2] = {0,0}; // Difference between true angle and desired angle
    float angle_int_err[2] = {0,0}; // Integral term of the error of the angle
    float angle_p[2] = {0.02,0.02}; // Angle proportional gain (Base 0.01) (Faster : 0.02)
    float angle_i[2] = {0.02,0.02}; // Angle integral gain (Base 0.01) (Faster : 0.02)
    float angle_d[2] = {0.0104,0.0104}; // Angle derivative gain (Base 0.0013) (Faster : 0.0104)
    float angle_con[2] = {0,0}; // Angle controller 
    float max_angle = 10; // Largest angle possible from controller
    float angle_dt = 0.01; // Time step for integral error

    // Rate controller vars
    float rate_des[3] = {0,0,0}; // Desired rate from the Angle Controller (or the mapping for YAW)
    float rate_err[3] = {0,0,0}; // Difference between true rate and desired rate
    float rate_int_err[3] = {0,0,0}; // Integral term of the error of the rate
    float rate_p[3] = {0,0,0}; // Rate proportional gain
    float rate_i[3] = {0,0,0}; // Rate integral gain
    float rate_d[3] = {0,0,0}; // Rate derivative gain
    float rate_con[3] = {0,0,0}; // Rate controller
    float max_rate = 15; // Largest possible angular rate from controller
    float rate_dt = 0.01; // Time step for integral error

public:
    float* thr_pwm;
    float* roll_pwm;
    float* pitch_pwm;
    float* yaw_pwm;
    float pwmout_0;
    float pwmout_1;
    float pwmout_2;
    float pwmout_3;
    float c_delf;
    float c_delm0;
    float c_delm1;
    float c_delm2;
    bool angle2rate = 0; // Change this rate to 1 to use the angle output as the rate input
    void controller_loop(bool value);
    void angle_controller_loop();
    void rate_controller_loop();
    void print();
    void mixer();
};

#endif