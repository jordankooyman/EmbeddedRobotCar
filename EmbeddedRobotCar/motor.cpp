#include <Arduino.h>
#include "config.cpp"

#ifndef motorFile
#define motorFile
// ---------- MOTOR CLASS DECLARATIONS ----------- //

class Motor {
private:
  byte _pin1;
  byte _pin2;
  byte _enable;
  byte _speed;
  byte _offset;
  byte _MaxAdjustSpeed;
  byte _MinAdjustSpeed;

public:
  enum MotorDirection {	
    MotorStop, 
    MotorForward, 
    MotorReverse,
    MotorLock
  }; // enum MotorDirection

  // Default Constructor
  Motor() : _pin1(0), _pin2(0), _enable(0), _speed(0), _offset(0), _MinAdjustSpeed(MinAdjustSpeed), _MaxAdjustSpeed(MaxAdjustSpeed) {}
  
  
  // Parameterized Constructor
  Motor( byte pin1, byte pin2, byte enable, byte offset )
    : _pin1( pin1 ), _pin2 ( pin2 ), _enable( enable ), _offset( _offset ), _speed(0) {
      pinMode( _pin1, OUTPUT );
      pinMode( _pin2, OUTPUT );
      pinMode( _enable, OUTPUT );
      if ( offset > 0 ) {
        _MaxAdjustSpeed = MaxAdjustSpeed - offset;
        _MinAdjustSpeed = MinAdjustSpeed;
      }
      else if (offset < 0) {
        _MaxAdjustSpeed = MaxAdjustSpeed;
        _MinAdjustSpeed = MinAdjustSpeed - offset;
      }
      else {
        _MaxAdjustSpeed = MaxAdjustSpeed;
        _MinAdjustSpeed = MinAdjustSpeed;
      }
  } // end Motor prop constructor
  
  
  // Get/Set speed
  inline byte getSpeed() { return _speed; } 
  
  inline void setSpeed( byte speed ) 
  { 
    speed = speed - _offset;
    if (speed == 0 || (speed <= _MaxAdjustSpeed && speed >= _MinAdjustSpeed))
      _speed = speed; 
    else if (speed < _MinAdjustSpeed)
      _speed = _MinAdjustSpeed;
    else if (speed > _MaxAdjustSpeed)
      _speed = _MaxAdjustSpeed;
    analogWrite(  _enable, _speed ); 
  }

  void incrementSpeed() {
    if (_speed < _MaxAdjustSpeed) {
      _speed += AdjustValue;
    }
    analogWrite(  _enable, _speed );
  }
  
  void decrementSpeed() {
    if (_speed > _MinAdjustSpeed) {
      _speed -= AdjustValue;
    }
    analogWrite(  _enable, _speed );
  }

  void run( MotorDirection dir ) {
    switch( dir ) {
      case MotorStop: // Free Running Motor Stop
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
      case MotorLock: // Fast Motor Stop
        digitalWrite( _pin1,   HIGH );
        digitalWrite( _pin2,   HIGH );
        digitalWrite( _enable, HIGH );
    } // set direction
  } // end run
};

#endif