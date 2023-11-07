#include <Arduino.h>
#include "config.cpp"

#ifndef ultrasonicFile
#define ultrasonicFile
// ---------- ULTRASONIC SENSOR CLASS DECLARATIONS ----------- //

class UltrasonicSensor {
private:
  byte triggerPin;
  byte echoPin;
  int maxDuration;

public:
  /*
   * Default Constructor
   */
  UltrasonicSensor()
  {
    triggerPin = 0;
    echoPin = 0;
    maxDuration = MAX_DISTANCE * 12 * 74 * 2;
  }


  /*
   * 3-Pin Constructor
   */
  UltrasonicSensor( byte sensorPin )
  {
    triggerPin = sensorPin;
    echoPin = sensorPin;
    maxDuration = MAX_DISTANCE * 12 * 74 * 2;
  }

  /*
   * 4-Pin Constructor
   */
  UltrasonicSensor( byte tPin, byte ePin )
  {
    triggerPin = tPin;
    echoPin = ePin;
    maxDuration = MAX_DISTANCE * 12 * 74 * 2;
  }


  /*
   * Measures distance to closest object in cone.
   * Returns distance in inches (double).
   */
  double measureInches()
  {
    return  measure() / 74.0;
  }


  /*
   * Measures distance to closest object in cone.
   * Returns distance in centimeters (double).
   */
  double measureCentimeters()
  {
    return measure() / 29.1;
  }

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

    pinMode( echoPin, INPUT );
    digitalWrite( echoPin, LOW );
    pinMode( triggerPin, OUTPUT );
    digitalWrite( triggerPin, LOW );

    delayMicroseconds(5);

    digitalWrite( triggerPin, HIGH );
    delayMicroseconds(10);
    digitalWrite( triggerPin, LOW );

    pinMode( echoPin, INPUT );
    duration = pulseIn( echoPin, HIGH );

    interrupts();

    if( duration > 0 )
      duration /= 2.0;

    return duration;
  } // end measure()
};

#endif