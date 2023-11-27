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
  StateStart,
  StateForward,
  StateCourseCorrectLeft,
  StateCourseCorrectRight,
  StateTurnLeft,
  StateTurnRight,
  StateTurnApproaching,
  StateTurnEnding,
  StateEndTurn
}; // enum State

State currentState;
int sensorCount;
float leftSensorDistance;
float rightSensorDistance;
float frontSensorDistance;
bool leftTurn;
float frontSensorPreTurn;



void setup() 
{
  leftMotor  = Motor( H1A, H2A, H12EN, LeftMotorSpeedCompensation ); 
  rightMotor = Motor( H3A, H4A, H34EN, RightMotorSpeedCompensation );

  frontSensor = UltrasonicSensor( FRONT_TRIG, FRONT_ECHO );
  leftSensor  = UltrasonicSensor( LEFT_TRIG,  LEFT_ECHO  );
  rightSensor = UltrasonicSensor( RIGHT_TRIG, RIGHT_ECHO );

  pinMode( DEBUG_RED, OUTPUT );
  pinMode( DEBUG_GREEN, OUTPUT );
  pinMode( DEBUG_BLUE, OUTPUT );

  #ifdef SerialDebugMode
  Serial.begin( 9600 );
  #endif
  currentState = StateStart;

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
    case StateStart:
      start();
      break;
    case StateForward:
      checkForStateChange();
      forward();
      break;
    case StateTurnLeft:
      turnLeft();
      break;
    case StateTurnRight:
      turnRight();
      break;
    case StateTurnApproaching:
      turnApproaching();
      break;
    case StateTurnEnding:
      turnEnding();
      break;
    case StateEndTurn:
      endTurn();
  }
  
  #ifdef SerialDebugMode
  Serial.println();
  #endif
} // end Loop()


void checkSensors()
{  
  sensorCount++;
  leftSensorDistance = leftSensor.measureInches();
  rightSensorDistance = rightSensor.measureInches();
  if (sensorCount >= FrontSensorPollDivsor)  // Poll Front Every 3rd Reading
  {
    sensorCount = 0;
    frontSensorDistance = frontSensor.measureInches();
  }

  #ifdef SerialDebugMode
  Serial.print( "Left: ");
  Serial.print( leftSensorDistance);
  Serial.print( " | Right: ");
  Serial.print( rightSensorDistance);
  Serial.print( " | Front: ");
  Serial.print( frontSensorDistance);
  #endif
} // end checkSensors()

void stop()
{
  #ifdef SerialDebugMode
    Serial.print( " Stop ");
  #endif 

  leftMotor.run(  Motor::MotorStop );
  rightMotor.run( Motor::MotorStop );
} // end stop()


void start()
{
  #ifdef SerialDebugMode
    Serial.print( " Start ");
  #endif 

  static int Speed = MinimumSpeed;
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  rightMotor.setSpeed( Speed );
  if (Speed == MinimumSpeed) // If first iteration
  {
    rightMotor.run( Motor::MotorForward );
    leftMotor.run(  Motor::MotorForward );
  }
  else if (Speed >= StartSpeed)
  {
    currentState = StateForward;
    Speed = MinimumSpeed;
    return;
  }
  Speed+=RampSpeedIncrement;
  rightMotor.setSpeed( Speed );
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  if (Speed >= StartSpeed)
  {
    currentState = StateForward;
    Speed = MinimumSpeed;
    return;
  }
  Speed+=RampSpeedIncrement;
} // end start()


void forward()
{    
  #ifdef SerialDebugMode
    Serial.print( " Forward ");
  #endif 

  // Update Future State as needed
  float currentDifference = leftSensorDistance - rightSensorDistance;


  #ifdef SerialDebugMode
  Serial.print(" - Current Difference: ");
  Serial.print(currentDifference);
  #endif

  float currentDeviation = abs(currentDifference) / 2;
  static float previousDeviation = 0;

  digitalWrite( DEBUG_RED,   LOW );
  digitalWrite( DEBUG_GREEN, LOW );

  // Course Correction Code
  if( leftSensorDistance > rightSensorDistance ) 
  {  // Deviating right, slow left motor speed 
    digitalWrite( DEBUG_RED, HIGH );
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall
      leftMotor.setSpeed(  ForwardSpeed - (correctionFactor * currentDeviation) - LeftMotorOffset );
      rightMotor.setSpeed( ForwardSpeed );
    }
    else
    { // Deviation decreasing, approaching midline
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset);
      rightMotor.setSpeed( ForwardSpeed ); 
    }
  }
  else
  {  // Deviating left, slow right motor speed
    digitalWrite( DEBUG_GREEN, HIGH );
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset );
      rightMotor.setSpeed( ForwardSpeed - (correctionFactor * currentDeviation) );
    }
    else
    { // Deviation decreasing, approaching midline
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset );
      rightMotor.setSpeed( ForwardSpeed );
    }
  }

  previousDeviation = currentDeviation;

} // end forward()


void checkForStateChange()
{
  if ( frontSensorDistance < FrontTurnApproachingDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateTurnApproaching;
  if ( frontSensorDistance < FrontStopDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateStop;
  if ( frontSensorDistance > UltrasonicMaxDistance - 5 && leftSensorDistance > UltrasonicMaxDistance - 5 && rightSensorDistance > UltrasonicMaxDistance - 5 )
    currentState = StateStop;
} // end checkForStateChange()


/*
 * Disables drift correction watches for turn condition.
 * Should be in this state while the incoming wall is between 15 and 30 inches.
 * At 15 inches, should see which side is open and change to turn state in that direction.
 */
void turnApproaching()
{ 
  #ifdef DisableTurn
  currentState = StateForward;
  return;
  #endif
  #ifdef SerialDebugMode
    Serial.print( " Identify Turn ");
  #endif 

  // Blue LED indicates we're in turnApproaching state
  digitalWrite( DEBUG_RED,   LOW  );
  digitalWrite( DEBUG_GREEN, LOW  );
  digitalWrite( DEBUG_BLUE,  HIGH );

  if( frontSensorDistance < FrontTurnStartDistance ) // Incoming wall less than FrontTurnStartDistance
  {
    frontSensorPreTurn = frontSensorDistance + FrontSensorOffsetFromSideSensors;
    
    leftMotor.run(  Motor::MotorStop );
    rightMotor.run( Motor::MotorStop );
    
    leftMotor.setSpeed(  TurnSpeed - LeftMotorOffset );
    rightMotor.setSpeed( TurnSpeed );
    
    if( leftSensorDistance > rightSensorDistance ) // Left wall further away than right
      currentState = StateTurnLeft;
    else  // Right wall further away than left
      currentState = StateTurnRight;
  }
} // end turnApproaching()


void turnLeft()
{
  digitalWrite( DEBUG_RED, HIGH );
  #ifdef SerialDebugMode
    Serial.print( " Turn Left ");
  #endif 

  leftTurn = true;

  static bool didATurn = false;

  if( !didATurn ){  
    leftMotor.run(  Motor::MotorReverse );
    rightMotor.run( Motor::MotorForward );
    didATurn = true;
  }
  else
  {
    leftMotor.run( Motor::MotorStop  );
    rightMotor.run( Motor::MotorStop );
    didATurn = false;
  }
  
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
    currentState = StateTurnEnding; // Ensure turn has started before looking to stop
} // end turnLeft()


void turnRight()
{
  digitalWrite( DEBUG_GREEN, HIGH );
  #ifdef SerialDebugMode
    Serial.print( " Turn Right ");
  #endif 

  leftTurn = false;
  
  static bool didATurn = false;

  if( !didATurn ){
    leftMotor.run(  Motor::MotorForward );
    rightMotor.run( Motor::MotorReverse );
    didATurn = true;
  }
  else
  {
    leftMotor.run( Motor::MotorStop  );
    rightMotor.run( Motor::MotorStop );
    didATurn = false;
  }
  
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
    currentState = StateTurnEnding; // Ensure turn has started before looking to stop
} // end turnRight()


void turnEnding()
{
  #ifdef SerialDebugMode
    Serial.print( " Finish Turn ");
  #endif 

  float currentSensorDistance;
  if ( leftTurn )
    currentSensorDistance = rightSensorDistance;
  else
    currentSensorDistance = leftSensorDistance;

  if ( currentSensorDistance <= (frontSensorPreTurn + TurnStopSensorDeviation) && currentSensorDistance >= (frontSensorPreTurn - TurnStopSensorDeviation) )
  {
    leftMotor.run(  Motor::MotorLock );
    rightMotor.run( Motor::MotorLock );
    currentState = StateEndTurn;
    delay( TurnEndingDelayMS );
  }
} // end turnEnding()


void endTurn()
{
  static int counter = 0;
  digitalWrite( DEBUG_RED,   LOW );
  digitalWrite( DEBUG_GREEN, LOW );
  digitalWrite( DEBUG_BLUE,  LOW );
  #ifdef SerialDebugMode
    Serial.print( " Exit Turn " );
  #endif 

  rightMotor.run( Motor::MotorForward );
  leftMotor.run(  Motor::MotorForward );
  
  if( counter >= TurnEndCountDelay && abs( leftSensorDistance - rightSensorDistance ) < TurnExitedDeviation )
  {
    currentState = StateStop; // StateForward
    counter = 0;
  }

  counter++;
} // end endTurn()
