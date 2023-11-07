#include <Arduino.h>

#ifndef configFile
#define configFile
// ----------- CONFIGURATION CONSTANTS ----------- //
#define FrontSensorPollDivsor 3

#define AdjustmentTimeoutLimit 20

#define StartSpeed 240
#define MinAdjustSpeed 200
#define MaxAdjustSpeed 255
#define AdjustValue 1

// ----------- PIN CONSTANTS ----------- //

// Wheel Motor (L)
const byte H1A = 4; // Blue
const byte H2A = 7; // Yellow
const byte H12EN = 5; // Green

// Wheel motor (R)
const byte H3A = 2; // White
const byte H4A = 8; // Brown
const byte H34EN = 6; // Black

// Front Ultrasonic Sensor
const byte FRONT_MAX_DISTANCE = 30;
const byte FRONT_ECHO = A2; // Orange
const byte FRONT_TRIG = A3; // Yellow

// Left Ultrasonic Sensor
const byte MAX_DISTANCE = 25;
const byte LEFT_ECHO = A5; // White
const byte LEFT_TRIG = A4; // Black

// Right Ultrasoinc Sensor
const byte RIGHT_ECHO = A1; // Green
const byte RIGHT_TRIG = A0; // Blue

// Debug LED
const byte DEBUG_1 = 11; // Black wire, blue LED
const byte DEBUG_2 = 13; // Brown wire, Green LED
// const byte DEBUG_B = ;

#endif