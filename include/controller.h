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
    float angle_KP[3] = {0,0,0};
    float angle_KD[3] = {0,0,0};
    float angle_KI[3] = {0,0,0};

    float angle_IE[3] = {0,0,0};

    float desired_roll;
    float desired_pitch;
    float desired_yaw;
    float roll_error;
    float pitch_error;
    float max_angle = 10;




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
    float rate_integral_error[3] = {0,0,0};
    float angle_integral_error[3] = {0,0,0};
    float rate_dt = 0.01;
    float angle_dt = 0.01;
    void controller_loop(int value);
    void rate_controller_loop();
    void angle_controller_loop();
    void print();
    void mixer();
};

#endif