#include <Arduino.h>

#ifndef configFile
#define configFile

//#define SerialDebugMode
//#define DisableTurn


// ----------- CONFIGURATION CONSTANTS ----------- //
// Ultrasonic Sensors
#define FrontSensorPollDivsor 2
#define UltrasonicMaxDistance 40.0
#define UltrasonicTimeoutReturnDistance UltrasonicMaxDistance
#define FrontSensorOffsetFromSideSensors -1.1
#define FrontSensorPreturnOffset 4

// State Change Parameters
#define MinimumSensorDistance 0.01
#define FrontStopDistance 1.5
#define FrontTurnApproachingDistance 25
#define FrontTurnStartDistance 17
#define TurnStopSensorDeviation 2.5
#define TurnExitedDeviation 30
#define TurnEndingDelayMS 500
#define TurnEndCountDelay 10

// Motor Speed Parameters
#define AdjustValue 2
#define LeftMotorOffset 21
#define LeftMotorSpeedCompensation 0
#define RightMotorSpeedCompensation 0
#define StartSpeed (MaxAdjustSpeed + MinAdjustSpeed) / 2
#define ForwardSpeed 225
#define CourseCorrectCorrectionFactor 20
#define RampSpeedIncrement 2
#define MinimumSpeed 120
#define MinAdjustSpeed 140
#define MaxAdjustSpeed 235
#define TurnSpeed ForwardSpeed
#define TurnSoftwarePWMMaxCount 1



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


// Check validity of parametric configurations
#if MinAdjustSpeed < MinimumSpeed
#error MinAdjustSpeed must be greater than or equal to MinimumSpeed
#endif
#if MaxAdjustSpeed > 255
#error MaxAdjustSpeed must be less than or equal to 255
#endif
#if TurnSpeedFast > 255
#error TurnSpeedFast must be less than or equal to 255
#endif
#if TurnSpeedSlow > TurnSpeedFast
#error TurnSpeedSlow must be less than or equal to TurnSpeedFast
#endif
#if TurnSpeedSlow < 0
#error TurnSpeedSlow must either be 0 or greater than or equal to MinimumSpeed
#endif
#if TurnSpeedSlow > 0 && TurnSpeedSlow < MinimumSpeed
#error TurnSpeedSlow must either be 0 or greater than or equal to MinimumSpeed
#endif
#if LeftMotorSpeedCompensation < 0 && RightMotorSpeedCompensation < 0
#error Only adjust LeftMotorSpeedCompensation or RightMotorSpeedCompensation, not both
#endif
#endif // End Header Guard