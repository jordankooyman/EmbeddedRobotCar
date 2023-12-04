// File: config.cpp
// Class: CEN 4930, Fall 2023, CRN 84929
// Contributors: Jordan Kyooyman, David West
// Description: Configuration file for CEN 4930 Final Project
//              Note - All distances are in inches
//              Note - All Times are in milliseconds
// -----------------------------------------------------------

#include <Arduino.h>

#ifndef configFile
#define configFile

// --------------- DEBUGGING MODES --------------- //
//#define SerialDebugMode
//#define DisableTurn
//#define LEDTurnDebugging
//#define LEDForwardDebugging


// ----------- CONFIGURATION CONSTANTS ----------- //
// Ultrasonic Sensors
#define FrontSensorPollDivsor 2                 // Frequency for front sensor polling. Poll once every X loops
#define UltrasonicMaxDistance 40.0              // Maximum distance for ultrasonic sensors
#define UltrasonicTimeoutReturnDistance UltrasonicMaxDistance  // Timeout distance for ultrasonic sensors
#define FrontSensorOffsetFromSideSensors -1.1   // Difference in distance from front or side sensors to the
                                                //   center pivot point
#define FrontSensorPreturnOffset 4              // In a turn, the distance the front wall must increase before

// State Change Parameters
#define MinimumSensorDistance 0.01              // Minimum distance for sensor data to be considered valid
#define FrontStopDistance 1.5                   // Distance from wall before triggering stop state to prevent
                                                //   wall ramming
#define FrontTurnApproachingDistance 25         // Distance from wall before triggering turnApproaching state
#define FrontTurnStartDistance 17               // Distance from wall before triggering turnLeft/turnRight state
#define TurnStopSensorDeviation 2.5             // Additional distance from former front, now side wall to 
                                                //   trigger endTurn state
#define TurnExitedDeviation 30                  // Sum of left and right sensor required to consider the turn 
                                                //   ended and switch to forward state
#define TurnEndingDelay 500                   // Stop duration at end of turn to kill momentum
#define TurnEndCountDelay 10                    // Minimum number of loops in a turn

// Motor Speed Parameters
#define AdjustValue 2                           // Amount to change motor speed with increment/decrementSpeed fn
#define LeftMotorOffset 21                      // Difference in motor power. positive = left stronger than right
#define StartSpeed (MaxAdjustSpeed + MinAdjustSpeed) / 2  // Initial speed for motors.
#define ForwardSpeed 225                        // Speed to travel during forward state
#define CourseCorrectCorrectionFactor 20        // Forward State: Amount to decrease motor speed to turn towards
#define RampSpeedIncrement 2                    // Amount to increase motor speed per loop during acceleration.
#define MinimumSpeed 120                        // Slowest speed available before motor stall. Must take into 
                                                //   account LeftMotorOffset
#define MinAdjustSpeed 140                      // Minimum speed that can a motor can be set
#define MaxAdjustSpeed 235                      // Maximum speed that a motor can be set
#define TurnSpeed ForwardSpeed                  // Motor speed at which to turn.
#define TurnSoftwarePWMMaxCount 1               // Count controlled motor pulsing during turn. 
                                                //   Larger value = slower turn



// ----------- PIN CONSTANTS ----------- //

// Wheel Motor (L)
const byte H1A = 4; // Blue Wire
const byte H2A = 7; // Yellow Wire
const byte H12EN = 5; // Green Wire

// Wheel motor (R)
const byte H3A = 2; // White Wire
const byte H4A = 8; // Brown Wire
const byte H34EN = 6; // Black Wire

// Front Ultrasonic Sensor
const byte FRONT_ECHO = A2; // Orange Wire
const byte FRONT_TRIG = A3; // Yellow Wire

// Left Ultrasonic Sensor
const byte LEFT_ECHO = A5; // White Wire
const byte LEFT_TRIG = A4; // Black Wire

// Right Ultrasonic Sensor
const byte RIGHT_ECHO = A1; // Green Wire
const byte RIGHT_TRIG = A0; // Blue Wire

// Debug RGB LED
const byte DEBUG_RED = 11; // Black wire, Red LED
const byte DEBUG_GREEN = 13; // Brown wire, Green LED
const byte DEBUG_BLUE = 3; // Red wire, Blue LED


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
#ifdef LEDTurnDebugging
#ifdef LEDForwardDebugging
#error Only use 1 LED Debugging Mode at a time
#endif
#endif
#endif // End Header Guard