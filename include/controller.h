#ifndef CONTROLLER_H
#define CONTROLLER_H

// void controller_setup();

#include <cstdint>



class Controller{
private:
    // float KD[3] = {0.05,0.1,0.1};
    float KDI[4] = {0,-0.01,-0.11,-0.04};
    float c_delf;
	float c_delm0;
	float c_delm1;
	float c_delm2;
    // float delftrim = 0.01;
public:
    float thr_pwm;
    float roll_pwm;
    float pitch_pwm;
    float yaw_pwm;
    float pwmout_0;
    float pwmout_1;
    float pwmout_2;
    float pwmout_3;
    void controller_loop();
    void print();
    void mixer();
};


#endif
