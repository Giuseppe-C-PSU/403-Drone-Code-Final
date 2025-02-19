// #include "src/sensor_stuff/sensor_prelim.h"
// #include "src/sensor_stuff/sensors.h"
// #include "src/EKF_stuff/EKF.h"
#include "sensors.h"
#include "sensor_prelim.h"
#include "EKF.h"
#include <Arduino.h>
//EKF ekf;
extern Sensors sens;
//EKF ekf;

void setup()
{
  Serial.begin(9600);
  pozyx_setup();

}


void loop()
{
  Serial.print("Pozyx: ");
  pozyx_loop();
  delay(100);

}