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

  pinMode( DEBUG_RED,   OUTPUT );
  pinMode( DEBUG_GREEN, OUTPUT );
  pinMode( DEBUG_BLUE,  OUTPUT );

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
  Serial.print( "Left: " );
  Serial.print( leftSensorDistance );
  Serial.print( " | Right: " );
  Serial.print( rightSensorDistance );
  Serial.print( " | Front: " );
  Serial.print( frontSensorDistance );
  #endif
} // end checkSensors()

void stop()
{
  #ifdef SerialDebugMode
    Serial.print( " Stop ");
  #endif 

  digitalWrite( DEBUG_RED,   LOW );
  digitalWrite( DEBUG_BLUE,  LOW );
  digitalWrite( DEBUG_GREEN, LOW );

  leftMotor.run(  Motor::MotorLock );
  rightMotor.run( Motor::MotorLock );
} // end stop()


void start()
{
  #ifdef SerialDebugMode
    Serial.print( " Start ");
  #endif 

  static int Speed = MinAdjustSpeed;
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  rightMotor.setSpeed( Speed );
  if (Speed == MinAdjustSpeed) // If first iteration
  {
    rightMotor.run( Motor::MotorForward );
    leftMotor.run(  Motor::MotorForward );
  }
  else if (Speed >= StartSpeed)
  {
    currentState = StateForward;
    Speed = MinAdjustSpeed;
    return;
  }
  Speed+=RampSpeedIncrement;
  rightMotor.setSpeed( Speed );
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  if (Speed >= StartSpeed)
  {
    currentState = StateForward;
    Speed = MinAdjustSpeed;
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
  //  digitalWrite( DEBUG_RED, HIGH );
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset - (CourseCorrectCorrectionFactor * currentDeviation) );
      rightMotor.setSpeed( ForwardSpeed );
    }
    else
    { // Deviation decreasing, approaching midline
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset);
      rightMotor.setSpeed( ForwardSpeed  - (CourseCorrectCorrectionFactor * currentDeviation) ); 
    }
  }
  else
  {  // Deviating left, slow right motor speed
  //  digitalWrite( DEBUG_GREEN, HIGH );
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset );
      rightMotor.setSpeed( ForwardSpeed - (CourseCorrectCorrectionFactor * currentDeviation) );
    }
    else
    { // Deviation decreasing, approaching midline
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset - (CourseCorrectCorrectionFactor * currentDeviation) );
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
    currentState = StateStop; // No Wall Ramming?
  if ( frontSensorDistance > UltrasonicMaxDistance - 5 && leftSensorDistance > UltrasonicMaxDistance - 5 && rightSensorDistance > UltrasonicMaxDistance - 5 )
    currentState = StateStop;
} // end checkForStateChange()


/*
 * Disables drift correction watches for turn condition.
 * Should be in this state while the incoming wall is between 17 and 25 inches.
 * At 17 inches, should see which side is open and change to turn state in that direction.
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
  digitalWrite( DEBUG_BLUE,  HIGH );

  if( frontSensorDistance < FrontTurnStartDistance ) // Incoming wall less than FrontTurnStartDistance
  {
    frontSensorPreTurn = frontSensorDistance + FrontSensorOffsetFromSideSensors;
    
    leftMotor.run(  Motor::MotorStop );
    rightMotor.run( Motor::MotorStop );
    
    leftMotor.setSpeed(  TurnSpeed - LeftMotorOffset );
    rightMotor.setSpeed( TurnSpeed );
    
    if( leftSensorDistance > rightSensorDistance ) // Left wall further away than right
    { 
      currentState = StateTurnLeft;
      digitalWrite( DEBUG_BLUE, LOW );
    }
    else  // Right wall further away than left
    {
      currentState = StateTurnRight;
      digitalWrite( DEBUG_BLUE, LOW );
    }
  }
} // end turnApproaching()


void turnLeft()
{ 
  #ifdef SerialDebugMode
    Serial.print( " Turn Left ");
  #endif 

  // Visual indicator: in StateTurnLeft
  digitalWrite( DEBUG_GREEN, HIGH  );

  leftTurn = true;

  static int lastTurnCounter = 0;

  // Slow turn by pulsing motors
  if( lastTurnCounter == TurnSoftwarePWMMaxCount )
  {  
    leftMotor.run(  Motor::MotorReverse );
    rightMotor.run( Motor::MotorForward );
    lastTurnCounter = 0;
  }
  else
  {
    leftMotor.run( Motor::MotorStop  );
    rightMotor.run( Motor::MotorStop );
    lastTurnCounter++;
  }
  
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
  {
    leftMotor.run(  Motor::MotorReverse );
    rightMotor.run( Motor::MotorForward );
    currentState = StateTurnEnding; // Ensure turn has started before looking to stop

    // Clear LEDs
    digitalWrite( DEBUG_GREEN, LOW );
  }
} // end turnLeft()


void turnRight()
{  
  #ifdef SerialDebugMode
    Serial.print( " Turn Right ");
  #endif 

  // Visual indicator: in StateTurnRight
  digitalWrite( DEBUG_RED,   HIGH );

  leftTurn = false;
  
  static int lastTurnCounter = 0;

  // Slow turn by pulsing motors
  if( lastTurnCounter == TurnSoftwarePWMMaxCount )
  {  
    leftMotor.run(  Motor::MotorForward );
    rightMotor.run( Motor::MotorReverse );
    lastTurnCounter = 0;
  }
  else
  {
    leftMotor.run(  Motor::MotorStop );
    rightMotor.run( Motor::MotorStop );
    lastTurnCounter++;
  }
  
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
  {
    leftMotor.run(  Motor::MotorForward );
    rightMotor.run( Motor::MotorReverse );
    currentState = StateTurnEnding; // Ensure turn has started before looking to stop

    // Clear LEDs
    digitalWrite( DEBUG_RED,   LOW );
  }
} // end turnRight()


void turnEnding()
{
  #ifdef SerialDebugMode
    Serial.print( " Finish Turn ");
  #endif 

  digitalWrite( DEBUG_BLUE, HIGH );
  
  float currentSensorDistance;
  if ( leftTurn )
    currentSensorDistance = rightSensorDistance;
  else
    currentSensorDistance = leftSensorDistance;

  // Slow turn by pulsing motors
  static int lastTurnCounter = 0;
  if( lastTurnCounter == TurnSoftwarePWMMaxCount )
  {  
    if( leftTurn )
    {
      leftMotor.run(  Motor::MotorForward );
      rightMotor.run( Motor::MotorReverse );
    }
    else
    {
      leftMotor.run(  Motor::MotorReverse );
      rightMotor.run( Motor::MotorForward );
    }
    lastTurnCounter = 0;
  }
  else
  {
    leftMotor.run(  Motor::MotorStop );
    rightMotor.run( Motor::MotorStop );
    lastTurnCounter++;
  }
  
  if ( currentSensorDistance <= (frontSensorPreTurn + TurnStopSensorDeviation) && currentSensorDistance >= (frontSensorPreTurn - TurnStopSensorDeviation) )
  {
    leftMotor.run(  Motor::MotorLock );
    rightMotor.run( Motor::MotorLock );
    currentState = StateEndTurn;
    lastTurnCounter = 0;  // Reset for next turn
    delay( TurnEndingDelayMS );
  }
} // end turnEnding()


void endTurn()
{
  static int counter = 0;
  digitalWrite( DEBUG_RED,   HIGH );
  digitalWrite( DEBUG_GREEN, HIGH );
  digitalWrite( DEBUG_BLUE,  LOW  );
  #ifdef SerialDebugMode
    Serial.print( " Exit Turn " );
  #endif 

  // Ramp Up Motor speed
  static int Speed = MinAdjustSpeed;
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  rightMotor.setSpeed( Speed );
  if (Speed == MinAdjustSpeed) // If first iteration
  {
    rightMotor.run( Motor::MotorForward );
    leftMotor.run(  Motor::MotorForward );
  }
  Speed+=RampSpeedIncrement;
  rightMotor.setSpeed( Speed );
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  if (Speed < ForwardSpeed)
    Speed+=RampSpeedIncrement;
  
  if( counter >= TurnEndCountDelay && (rightSensorDistance + leftSensorDistance) < TurnExitedDeviation )
  { // Exited corner of course
    Speed = MinAdjustSpeed;
    currentState = StateForward;
    counter = 0;
    digitalWrite( DEBUG_RED,   LOW );
    digitalWrite( DEBUG_GREEN, LOW );
    digitalWrite( DEBUG_BLUE,  LOW );
  }

  counter++;
} // end endTurn()
