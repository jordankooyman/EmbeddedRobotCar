#include <Arduino.h>

#ifndef configFile
#define configFile

//#define SerialDebugMode
// ----------- CONFIGURATION CONSTANTS ----------- //
// Ultrasonic Sensors
#define FrontSensorPollDivsor 3
#define UltrasonicMaxDistance 40.0
#define UltrasonicTimeoutReturnDistance UltrasonicMaxDistance

// State Change Parameters
#define MinimumSensorDistance 0.1
#define FrontStopDistance 5
#define FrontTurnApproachingDistance 30
#define FrontTurnStartDistance 25
#define TurnEndingDelay 250
#define FinishTurnDelay 100
#define TurnEndingDeviation 10

// Motor Speed Parameters
#define StartSpeed (MaxAdjustSpeed + MinAdjustSpeed) / 2
#define RampSpeedIncrement 2
#define MinimumSpeed 100
#define MinAdjustSpeed 100
#define MaxAdjustSpeed 150
#define TurnSpeedSlow 0
#define TurnSpeedFast MaxAdjustSpeed


// Course Correction Parameters
#define AdjustmentTimeoutLimit 20
#define AdjustValue 2
#define SignificantSensorDeviation 2.0
#define SignificantCorrectionDeviation 0.2
#define MaxAngledDistance 27.0
#define SensorChangeMultiplier 2.0

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
const byte FRONT_ECHO = A2; // Orange
const byte FRONT_TRIG = A3; // Yellow

// Left Ultrasonic Sensor
const byte LEFT_ECHO = A5; // White
const byte LEFT_TRIG = A4; // Black

// Right Ultrasoinc Sensor
const byte RIGHT_ECHO = A1; // Green
const byte RIGHT_TRIG = A0; // Blue

// Debug LED
const byte DEBUG_RED = 11; // Black wire, Red LED
const byte DEBUG_GREEN = 13; // Brown wire, Green LED
const byte DEBUG_BLUE = 3; // Red wire, Blue LED
// const byte DEBUG_B = ;

#endif