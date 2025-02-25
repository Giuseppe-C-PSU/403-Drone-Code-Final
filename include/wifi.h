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
 * Dr. Thanakorn Khamvilai
 * Email : thanakorn.khamvilai@psu.edu
 *
 * EndCopyright
 ***/

#ifndef AERSP_WIFI
#define AERSP_WIFI

#include <WiFiS3.h>
#include <Arduino.h>

#define BUFFERSIZE 1024

extern char ssid[];    // your network SSID (name)
extern char pass[];    // your network password (use for WPA, or use as key for WEP)

extern int status;
extern int keyIndex;             // your network key index number (needed only for WEP)

extern unsigned int localPort;      // local port to listen on

extern unsigned char buffer[BUFFERSIZE]; //buffer to hold incoming packet

extern WiFiUDP Udp;

void printWifiStatus();
void WifiSetup();

#endif