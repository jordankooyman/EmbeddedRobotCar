// File: ultrasonic.cpp
// Class: CEN 4930, Fall 2023, CRN 84929
// Contributors: Jordan Kyooyman, David West
// Description: Class implimentation for ultrasonic sensor component. 
// -----------------------------------------------------------

#include <Arduino.h>
#include "config.cpp"

#ifndef ultrasonicFile
#define ultrasonicFile

// ---------- ULTRASONIC SENSOR CLASS DECLARATIONS ----------- //

class UltrasonicSensor
{
private:
  byte triggerPin;            // Arduino pin for ultrasonic trigger
  byte echoPin;               // Arduino pin for ultrasonic echo
  int maxDuration;            // Max duration to wait for an echo pulse until timeout

public:

  // ----------------- CONSTRUCTORS ----------------- //
  /* Default Constructor
   */
  UltrasonicSensor()
  {
    triggerPin = 0;
    echoPin = 0;
    maxDuration = UltrasonicMaxDistance * 74 * 2 * 12; 
  }


  /* Constructor for 3-pin ultrasonic sensors
   * where the echo and trigger pin are one pin.
   */
  UltrasonicSensor( byte sensorPin )
  {
    triggerPin = sensorPin;
    echoPin = sensorPin;
    maxDuration = UltrasonicMaxDistance * 74 * 2 * 12; 
  }


  /* Constructor for 4-pin ultrasonic sensors
   * where the echo and trigger pins are their own pins.
   */
  UltrasonicSensor( byte tPin, byte ePin )
  {
    triggerPin = tPin;
    echoPin = ePin;
    maxDuration = UltrasonicMaxDistance * 74 * 2 * 12; 
  }


  // ----------------- CLASS DECLARATIONS AND DEFINITIONS ----------------- //
  /*
   * Measures distance to closest object in a cone.
   * Returns distance in inches (double).
   */
  double measureInches()
  {
    double distance = measure() / 74.0;

    // If measurement is outside acceptable range, return max distance
    if ( distance < 0.01 || distance > UltrasonicTimeoutReturnDistance )
      return UltrasonicTimeoutReturnDistance;
    
    return distance;
  } // end measureInches()


  /*
   * Measures distance to closest object in cone.
   * Returns distance in centimeters (double).
   */
  double measureCentimeters()
  {
    double distance = measure() / 29.1;

    if( distance < 0.01 )
      return UltrasonicTimeoutReturnDistance;
    
    return distance;
  } // end measureCentimeters()


private: 
  /*
   * Creates an ultrasonic pulse and measures duration 
   * of time before echo returns. Then called by
   * measureInches() or measureCentimeters() to convert
   * to distance.
   *
   * Returns half of the time taken for the pulse to 
   * rebound and return (IE approx time for pulse to 
   * reach an object) or 0 if no object is in range.
   */
  unsigned long measure() 
  {
    noInterrupts();					// An interupt will throw off the measurement
    unsigned long duration = 0;

    // Begin Measurement
    pinMode( echoPin, INPUT );
    digitalWrite( echoPin, LOW );
    pinMode( triggerPin, OUTPUT );
    digitalWrite( triggerPin, LOW );

    delayMicroseconds(5);

    digitalWrite( triggerPin, HIGH );
    delayMicroseconds(10);
    digitalWrite( triggerPin, LOW );

    // Wait for Response (with timeout)
    pinMode( echoPin, INPUT );
    duration = pulseIn( echoPin, HIGH, maxDuration );

    interrupts();        // Reenable Interrupts

    if( duration > 0 )   // If there is a reading, account for round trip time
      duration /= 2.0;

    return duration;
  } // end measure()

}; // end Ultrasonic class

#endif