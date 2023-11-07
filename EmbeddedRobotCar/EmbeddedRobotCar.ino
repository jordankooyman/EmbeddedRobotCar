#include "config.cpp"
#include "motor.cpp"
#include "ultrasonic.cpp"

Motor leftMotor;
Motor rightMotor;
UltrasonicSensor frontSensor;
UltrasonicSensor leftSensor;
UltrasonicSensor rightSensor;

enum State {
  StateStop,
  StateForward,
  StateTurnLeft,
  StateTurnRight
}; // enum State

State currentState;
int sensorCount;
float leftSensorDistance;
float rightSensorDistance;
float frontSensorDistance;


void setup() 
{
  leftMotor  = Motor( H1A, H2A, H12EN ); 
  rightMotor = Motor( H3A, H4A, H34EN ); 

  frontSensor = UltrasonicSensor( FRONT_TRIG, FRONT_ECHO );
  leftSensor  = UltrasonicSensor( LEFT_TRIG,  LEFT_ECHO  );
  rightSensor = UltrasonicSensor( RIGHT_TRIG, RIGHT_ECHO );

  pinMode( DEBUG_1, OUTPUT );
  pinMode( DEBUG_2, OUTPUT );

  Serial.begin( 9600 );
  currentState = StateForward;
  
  delay(500);
} // end Setup()


void loop() 
{
  checkSensors();

  switch(currentState) 
  {
    case StateStop:
      stop();
      break;
    case StateForward:
      forward();
      break;
    case StateTurnLeft:
      turnLeft();
      break;
    case StateTurnRight:
      turnRight();
  }
} // end Loop()


void checkSensors()
{
  sensorCount++;
  leftSensorDistance = leftSensor.measureInches();
  rightSensorDistance = rightSensor.measureInches();
  if (sensorCount >= FrontSensorPollDivsor) { // Poll Front Every 3rd Reading
    sensorCount = 0;
    frontSensorDistance = frontSensor.measureInches();
  }
} // end checkSensors()


void stop()
{
  leftMotor.run(  Motor::MotorStop );
  rightMotor.run( Motor::MotorStop );
} // end stop()

int adjustmentTimeout = 0;

void forward()
{  
  // Start moving from stop
  if (!leftMotor.getSpeed() && !rightMotor.getSpeed()) 
  {
    leftMotor.setSpeed(  StartSpeed );
    rightMotor.setSpeed( StartSpeed );
    leftMotor.run(  Motor::MotorForward );
    rightMotor.run( Motor::MotorForward );
    return;
  }

  if ( frontSensorDistance < 4  && frontSensorDistance > 0.5 )
    currentState = StateStop;
  
  float difference = leftSensorDistance - rightSensorDistance;

  Serial.print("L Speed: ");
  Serial.print( leftMotor.getSpeed() );
  Serial.print(" | R Speed: ");
  Serial.println( rightMotor.getSpeed() );

  if ( difference > 1 )  // Robot deviating to the right
  {
    if (adjustmentTimeout)
    {
      adjustmentTimeout++;
      if (adjustmentTimeout > AdjustmentTimeoutLimit)
          adjustmentTimeout = 0;
    }
    else
    {
      digitalWrite( DEBUG_1, HIGH ); // Blue LED
      leftMotor.decrementSpeed();
      Serial.println("Diff > 1: Turning Left...");
      digitalWrite( DEBUG_1, LOW ); // Blue LED
    }
  }
  else if ( difference < -1 ) // Robot deviating to the left
  {
    if (adjustmentTimeout)
    {
      adjustmentTimeout++;
      if (adjustmentTimeout > AdjustmentTimeoutLimit)
          adjustmentTimeout = 0;
    }
    else
    {
      digitalWrite( DEBUG_2, HIGH ); // Green LED
      leftMotor.incrementSpeed();
      Serial.println("Diff < -1: Turning Right...");
      digitalWrite( DEBUG_2, LOW ); // Green LED
    }
  }

} // end forward()


void turnLeft()
{

} // end turnLeft()


void turnRight()
{

} // end turnRight()
