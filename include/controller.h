#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifndef C_PI
#define C_PI 3.14159265358979323846264338327950288419716939937511
#endif

#include <cstdint>

class Controller {
private:
    // Arduino Gains
    // float KD[3] = {0.1, 0.1, 0.1};
    // float KP[3] = {0.13, 0.13, 0.13};

    // Pace Gains:
    // float KD[3] = {0.1, 0.1, 0.1};
    // float KP[3] = {0.05, 0.05, 0.05};

    // Pace Attitude Gains
    float KD[3] = {0.1, 0.1, 0.1};
    float KP[3] = {0.05, 0.05, 0.5};

    // float KD[3] = {0, 0, 0};
    // float KP[3] = {0, 0, 0};
    // float KD[3] = {3, 3, 3};
    // float KP[3] = {5, 5, 5};
    // float TRIM[4] = {0, 0, 0, 0}; // Front Right, Back Right, Back Left, Front Left

    float thrBuffer[10] = {0};
    float rollBuffer[10] = {0};
    float pitchBuffer[10] = {0};
    float yawBuffer[10] = {0};
    float TRIM[3] = {0,0,0};

    float applyDeadband(float value, float deadband);
    float cumulativeMovingAverage(float newValue, float* buffer, int bufferSize);

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

    void controller_loop(int value);
    void print();
    void mixer();
};

#endif