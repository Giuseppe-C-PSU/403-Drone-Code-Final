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


// Adafruit_MLX90640 mlx;
// float frame[32*24]; // buffer for full frame of temperatures
// //#define PRINT_TEMPERATURES
// #define PRINT_ASCIIART


// void thermal_setup() {
//     while (!Serial) delay(10);
//     //Serial.begin(115200);
//     delay(100);

//     Serial.println("Adafruit MLX90640 Simple Test");
//     if (! mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
//         Serial.println("MLX90640 not found!");
//         while (1) delay(10);
//     }
//     Serial.println("Found Adafruit MLX90640");

//     Serial.print("Serial number: ");
//     Serial.print(mlx.serialNumber[0], HEX);
//     Serial.print(mlx.serialNumber[1], HEX);
//     Serial.println(mlx.serialNumber[2], HEX);

//     //mlx.setMode(MLX90640_INTERLEAVED);
//     mlx.setMode(MLX90640_CHESS);
//     Serial.print("Current mode: ");
//     if (mlx.getMode() == MLX90640_CHESS) {
//         Serial.println("Chess");
//     } else {
//         Serial.println("Interleave");
//     }

//     mlx.setResolution(MLX90640_ADC_18BIT);
//     Serial.print("Current resolution: ");
//     mlx90640_resolution_t res = mlx.getResolution();
//     switch (res) {
//         case MLX90640_ADC_16BIT: Serial.println("16 bit"); break;
//         case MLX90640_ADC_17BIT: Serial.println("17 bit"); break;
//         case MLX90640_ADC_18BIT: Serial.println("18 bit"); break;
//         case MLX90640_ADC_19BIT: Serial.println("19 bit"); break;
//     }

//     mlx.setRefreshRate(MLX90640_2_HZ);
//     Serial.print("Current frame rate: ");
//     mlx90640_refreshrate_t rate = mlx.getRefreshRate();
//     switch (rate) {
//         case MLX90640_0_5_HZ: Serial.println("0.5 Hz"); break;
//         case MLX90640_1_HZ: Serial.println("1 Hz"); break;
//         case MLX90640_2_HZ: Serial.println("2 Hz"); break;
//         case MLX90640_4_HZ: Serial.println("4 Hz"); break;
//         case MLX90640_8_HZ: Serial.println("8 Hz"); break;
//         case MLX90640_16_HZ: Serial.println("16 Hz"); break;
//         case MLX90640_32_HZ: Serial.println("32 Hz"); break;
//         case MLX90640_64_HZ: Serial.println("64 Hz"); break;
//     }
// }

// void thermal_loop() {
//     if (mlx.getFrame(frame) != 0) {
//         Serial.println("Failed");
//         return;
//     }
//     Serial.println("===================================");
//     Serial.print("Ambient temperature = ");
//     Serial.print(mlx.getTa(false)); // false = no new frame capture
//     Serial.println(" degC");
//     Serial.println();
//     Serial.println();
//     for (uint8_t h=0; h<24; h++) {
//         for (uint8_t w=0; w<32; w++) {
//         float t = frame[h*32 + w];
//     #ifdef PRINT_TEMPERATURES
//         Serial.print(t, 1);
//         Serial.print(", ");
//     #endif
//     #ifdef PRINT_ASCIIART
//         char c = '&';
//         if (t < 20) c = ' ';
//         else if (t < 23) c = '.';
//         else if (t < 25) c = '-';
//         else if (t < 27) c = '*';
//         else if (t < 29) c = '+';
//         else if (t < 31) c = 'x';
//         else if (t < 33) c = '%';
//         else if (t < 35) c = '#';
//         else if (t < 37) c = 'X';
//         Serial.print(c);
//     #endif
//         }
//         Serial.println();
//     }
// }