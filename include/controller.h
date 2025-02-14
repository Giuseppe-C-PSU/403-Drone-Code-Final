#ifndef CONTROLLER_H
#define CONTROLLER_H

// void controller_setup();

#include <cstdint>



class Controller{
private:
    int KD[3] = {1,1,1};
public:

    int16_t c_delf;
	int16_t c_delm0;
	int16_t c_delm1;
	int16_t c_delm2;


    void controller_loop();
    void print();
};


#endif
