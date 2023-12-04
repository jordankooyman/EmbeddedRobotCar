// File: motor.cpp
// Class: CEN 4930, Fall 2023, CRN 84929
// Contributors: Jordan Kyooyman, David West
// Description: Class implimentation for motor component. 
// -----------------------------------------------------------

#include <Arduino.h>
#include "config.cpp"

#ifndef motorFile
#define motorFile

// ----------------- MOTOR CLASS  ----------------- //

class Motor 
{
private:
  byte _pin1;                // Input pin 1
  byte _pin2;                // Input pin 2
  byte _enable;              // Enable pin
  byte _speed;               // Motor speed (effectively 100 - 255 to drive the motor)
  byte _MaxAdjustSpeed;      // Maximum speed to allow the motor to drive; avoids overflowing
  byte _MinAdjustSpeed;      // Minimum speed to allow the motor to drive; avoids underpowering

public:
  enum MotorDirection 
  {	
    MotorStop,          // Motor free rotating:    Ground to both input pins
    MotorForward,       // Motor driving forward:  Power to pin 1, ground to pin 2
    MotorReverse,       // Motor driving backward: Ground to pin 1, power to pin 2
    MotorLock           // Motor locked:           Power to both input pins
  }; // enum MotorDirection

  // ----------------- CONSTRUCTORS ----------------- //
  
  /*Default Constructor
   */
  Motor() : _pin1(0), _pin2(0), _enable(0), _speed(0), _MinAdjustSpeed(MinAdjustSpeed), _MaxAdjustSpeed(MaxAdjustSpeed) {}
  
  
  /* Parameterized Constructor
  */
  Motor( byte pin1, byte pin2, byte enable )
    : _pin1( pin1 ), _pin2 ( pin2 ), _enable( enable ), _speed(0) 
  {
      pinMode( _pin1,   OUTPUT );
      pinMode( _pin2,   OUTPUT );
      pinMode( _enable, OUTPUT );
  
      _MaxAdjustSpeed = MaxAdjustSpeed;
      _MinAdjustSpeed = MinAdjustSpeed;
  } // end Motor prop constructor


  // ----------------- CLASS DECLARATIONS AND DEFINITIONS ----------------- //
  /* Returns current motor speed.
   */
  inline byte getSpeed() { return _speed; } 


  /* Sets speed for the motor. 
   * Speed can either be 0 or between MinAdjustSpeed and MaxAdjustSpeed.
   * Values between 0 and MinAdjustSpeed will be changed to MinAdjustSpeed.
   * Values greater than MaxAdjustSpeed will be changed to MaxAdjustSpeed.
   */
  inline void setSpeed( byte speed ) 
  { 
    if (speed == 0 || (speed <= _MaxAdjustSpeed && speed >= _MinAdjustSpeed))
      _speed = speed; 
      
    else if (speed < _MinAdjustSpeed)
      _speed = _MinAdjustSpeed;
      
    else if (speed > _MaxAdjustSpeed)
      _speed = _MaxAdjustSpeed;
    
    analogWrite(  _enable, _speed ); 
  }


  /* Increments speed by AdjustValue.
   */
  void incrementSpeed() 
  {
    if (_speed + AdjustValue < _MaxAdjustSpeed) 
      _speed += AdjustValue;
    
    analogWrite(  _enable, _speed );
  }


  /* Decrements speed by AdjustValue.
   */
  void decrementSpeed() 
  {
    if (_speed - AdjustValue > _MinAdjustSpeed) 
      _speed -= AdjustValue;
    
    analogWrite(  _enable, _speed );
  }


  /* Changes the state of the motor's direction.
   */
  void run( MotorDirection direction ) 
  {
    switch( direction ) 
    {
      case MotorStop: // Free rotating Motor Stop
        digitalWrite( _pin1,   LOW );
        digitalWrite( _pin2,   LOW );
        digitalWrite( _enable, LOW );
        break;
      case MotorForward:
        digitalWrite( _pin1,   HIGH   );
        digitalWrite( _pin2,   LOW    );
        analogWrite(  _enable, _speed );
        break;
      case MotorReverse:
        digitalWrite( _pin1,   LOW    );
        digitalWrite( _pin2,   HIGH   );
        analogWrite(  _enable, _speed );
        break;
      case MotorLock: // Locked Motor Stop
        digitalWrite( _pin1,   HIGH );
        digitalWrite( _pin2,   HIGH );
        digitalWrite( _enable, HIGH );
    } // end switch-case for motor direction

  } // end run

}; // end Motor class

#endif