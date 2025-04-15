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
    float angle_p[2] = {1.0,-1.0}; // Angle proportional gain 
    float angle_i[2] = {0,0}; // Angle integral gain
    float angle_d[2] = {0,0}; // Angle derivative gain
    float angle_con[2] = {0,0}; // Angle controller 
    float max_angle = 10; // Largest angle possible from controller
    float angle_dt = 0.01; // Time step for integral error

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
    void controller_loop(bool value);
    void angle_controller_loop();
    void print();
    void mixer();
};

#endif