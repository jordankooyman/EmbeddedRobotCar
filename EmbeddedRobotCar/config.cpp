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
#define MinimumSensorDistance 0.1
#define FrontStopDistance 5
#define FrontTurnApproachingDistance 13
#define FrontTurnStartDistance 25
#define FinishTurnDelay 100
#define TurnStopSensorDeviation 3
#define TurnExitedDeviation 10
#define TurnEndingDelayMS 200
#define TurnEndCountDelay 20

// Motor Speed Parameters
#define LeftMotorOffset 20
#define LeftMotorSpeedCompensation 0
#define RightMotorSpeedCompensation 0
#define StartSpeed (MaxAdjustSpeed + MinAdjustSpeed) / 2
#define ForwardSpeed 225
#define correctionFactor 10
#define RampSpeedIncrement 2
#define MinimumSpeed 120
#define MinAdjustSpeed 120
#define MaxAdjustSpeed 235
#define TurnSpeed MinAdjustSpeed



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