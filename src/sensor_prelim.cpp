#include "sensors.h"
#include "sensor_prelim.h"
#include "Arduino.h"
#include "thermal.h"
// #include "Adafruit_MLX90640.h>

#define SERVO_PIN 11
Sensors sens;
Thermal therm;

float q[4] = {0,0,0,0};

void pozyx_setup(){
    sens.init();
    pinMode(LED_BUILTIN, OUTPUT);
    // delay(100);
    Serial.println("Initialization Over");
    // delay(1000);
}

void pozyx_loop(){
    //delay(500);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    sens.update();
    euler2quat(sens.data.euler[0],sens.data.euler[0],sens.data.euler[0], q);
    sens.data.quat[0] = q[0];
    sens.data.quat[1] = q[1];
    sens.data.quat[2] = q[2];
    sens.data.quat[3] = q[3];

    //sens.print(); // should be commented out for flight
}

void thermal_setup(){
    therm.is_connected();
    therm.init();
    pinMode(SERVO_PIN, OUTPUT);  // Set servo pin as an output
}

void thermal_loop(){
    therm.update();
    therm.print();
}

void euler2quat(float phi, float theta, float psi, float q[4] )
{
    float cphi = cos(phi * 0.5f);
    float sphi = sin(phi * 0.5f);
    float ctheta = cos(theta * 0.5f);
    float stheta = sin(theta * 0.5f);
    float cpsi = cos(psi * 0.5f);
    float spsi = sin(psi * 0.5f);

    q[0] = cphi * ctheta * cpsi + sphi * stheta * spsi;
    q[1]= sphi * ctheta * cpsi - cphi * stheta * spsi;
    q[2]= cphi * stheta * cpsi + sphi * ctheta * spsi;
    q[3] = cphi * ctheta * spsi - sphi * stheta * cpsi;
}

void moveServo(int angle) {
    int pulseWidth = map(angle, 0, 180, 500, 2500); // Convert angle to pulse width (500–2500µs)
    
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth); // Send the pulse for the required duration
    digitalWrite(SERVO_PIN, LOW);
}