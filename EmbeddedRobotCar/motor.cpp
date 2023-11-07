#include <Arduino.h>

#ifndef motorFile
#define motorFile
// ---------- MOTOR CLASS DECLARATIONS ----------- //

class Motor {
private:
  byte _pin1;
  byte _pin2;
  byte _enable;
  byte _speed;

public:
  enum MotorDirection {	
    MotorStop, 
    MotorForward, 
    MotorReverse
  }; // enum MotorDirection

  // Default Constructor
  Motor() : _pin1(0), _pin2(0), _enable(0), _speed(0) {}
  
  
  // Parameterized Constructor
  Motor( byte pin1, byte pin2, byte enable )
    : _pin1( pin1 ), _pin2 ( pin2 ), _enable( enable ), _speed(0) {
      pinMode( _pin1, OUTPUT );
      pinMode( _pin2, OUTPUT );
      pinMode( _enable, OUTPUT );
  } // end Motor prop constructor
  
  
  // Get/Set speed
  inline byte getSpeed() { return _speed; } 
  
  inline void setSpeed( byte speed ) 
  { 
    _speed = speed; 
    analogWrite(  _enable, _speed ); 
  }

  void incrementSpeed() {
    if (_speed < MaxAdjustSpeed) {
      _speed += AdjustValue;
    }
    analogWrite(  _enable, _speed );
  }
  
  void decrementSpeed() {
    if (_speed > MinAdjustSpeed) {
      _speed -= AdjustValue;
    }
    analogWrite(  _enable, _speed );
  }

  void run( MotorDirection dir ) {
    switch( dir ) {
      case MotorStop:
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
        digitalWrite( _enable, _speed );
    } // set direction
  } // end run
};

#endif