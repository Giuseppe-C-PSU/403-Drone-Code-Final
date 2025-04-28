/***
 * Copyright 2022, The Pennsylvania State University, All Rights Reserved.
 * Unauthorized use and/or redistribution is disallowed.
 * This library is distributed without any warranty; without even
 * the implied warranty of fitness for a particular purpose.
 *
 * Pennsylvania State University Unmanned Aerial System Research Laboratory (PURL)
 * Department of Aerospace Engineering
 * 229 Hammond
 * The Pennsylvania State University
 * University Park, PA 16802
 * http://purl.psu.edu
 *
 * Contact Information:
 * Dr. Vitor T. Valente (vitor.valente@psu.edu)
 *
 * EndCopyright
 ***/

#include "../include/thermal.h"
#include "sensor_prelim.h"
#include "navigation.h"

extern nav nav;
uint16_t mlx90640Frame[834];

Thermal::Thermal(){

}

Thermal::~Thermal(){

}

void Thermal::init(){
    Wire1.begin();
    Wire1.setClock(400000); //Increase I2C clock speed to 400kHz
    delay(100);
    if (this->is_connected() == false)
    {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
    }
    Serial.println("MLX90640 online!");
}

void Thermal::update(){
    for (byte x = 0 ; x < 2 ; x++) //Read both subpages
    {
        int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
        if (status < 0)
        {
            Serial.print("GetFrame Error: ");
            Serial.println(status);
        }
    }
}

void Thermal::print(){
    float total = 0;
    float avg = 0;
    for (int x = 0 ; x < 768 ; x++)
    {
        //Serial.print("Pixel ");
        //Serial.print(x);
        //Serial.print(": ");
        u_int16_t why = mlx90640Frame[x] - 65400;
        if (x%24 == 0){
            Serial.println();
        }
        Serial.print(why);
        Serial.print(", ");
        total+=why;
    

        // char c = '&';
        // if (why < 40) c = ' ';
        // else if (why < 48) c = '.';
        // else if (why < 56) c = '-';
        // else if (why < 64) c = '*';
        // else if (why < 72) c = '+';
        // else if (why < 80) c = 'x';
        // else if (why < 88) c = '%';
        // else if (why < 96) c = '#';
        // else if (why > 96) c = 'X';
        // else if (why < 0) c = 'HS';
        // Serial.print(c);
        
        // if(why>96)
        // {
        //     moveServo(90);
        // }


        
        // Serial.print(why-65400);
        // Serial.print(",");
    }
    Serial.println();
    avg = total/768;
    Serial.print("Average temp: ");
    Serial.print(avg); 
    Serial.println();

    if(avg > 65) // change this number to whatevre would work for the thing
    {
        fireDetected = true;
    }
    else if (avg > 65 && deploySuppressant)
    {
        moveServo(90);
        Serial.print("Servo at 90");
        Serial.println();
    }
    else
    {
        moveServo(0);
        Serial.print("Servo at 0");
        Serial.println();
    }
        
    
}

bool Thermal::is_connected()
{
    Wire1.beginTransmission((uint8_t)MLX90640_address);
    if (Wire1.endTransmission() != 0)
        return (false); //Sensor did not ACK
    return (true);
}


// @ # % & * ? + = 