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
  StateTurnLeft,
  StateTurnRight,
  StateTurnApproaching,
  StateTurnEnding
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

  pinMode( DEBUG_RED, OUTPUT );
  pinMode( DEBUG_GREEN, OUTPUT );

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
  
  #ifdef SerialDebugMode
  Serial.print( "Left: ");
  Serial.print( leftSensorDistance);
  Serial.print( " | Right: ");
  Serial.print( rightSensorDistance);
  Serial.print( " | Front: ");
  Serial.println( frontSensorDistance);
  #endif
} // end checkSensors()


void stop()
{
  leftMotor.run(  Motor::MotorStop );
  rightMotor.run( Motor::MotorStop );
} // end stop()

void start()
{
  static int Speed = MinimumSpeed;
  leftMotor.setSpeed(  Speed );
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
  leftMotor.setSpeed(  Speed );
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
  static int adjustmentTimeout = 0;
  static float priorDifference = 0;
  
  // Update Future State as needed
  if ( frontSensorDistance < FrontTurnApproachingDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateTurnApproaching;
  if ( frontSensorDistance < FrontStopDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateStop;

  // Course Correcting Code
  float currentDifference = leftSensorDistance - rightSensorDistance;

  #ifdef SerialDebugMode
  Serial.print("Prior Difference: ");
  Serial.print(priorDifference);
  Serial.print(" Current Difference: ");
  Serial.println(currentDifference);
  #endif

  if ( currentDifference > SignificantSensorDeviation || rightSensorDistance == UltrasonicTimeoutReturnDistance )  // Robot deviating to the right
  {    
    // Course Correct every <adjustmentTimeoutLimit> loops
    if (adjustmentTimeout)
    { 
      adjustmentTimeout++;
      if (adjustmentTimeout > AdjustmentTimeoutLimit)
          adjustmentTimeout = 0;
    }
    else
    { // Compare prior difference to current difference - if difference is lessening, don't adjust
      if ( priorDifference <= currentDifference || rightSensorDistance == UltrasonicTimeoutReturnDistance )
      {
        digitalWrite( DEBUG_RED,  HIGH  );
        digitalWrite( DEBUG_GREEN, LOW );  // LEFT RED
        leftMotor.decrementSpeed();  // Turn left
        #ifdef SerialDebugMode
        Serial.println("Diff > 1: Adjusting to the left...");
        #endif
      }
      priorDifference = currentDifference;
    }
  }
  else if ( currentDifference < -SignificantSensorDeviation || leftSensorDistance == UltrasonicTimeoutReturnDistance ) // Robot deviating to the left
  { 
    // Course Correct every <AdjustmentTimeoutLimit> loops
    if (adjustmentTimeout)
    {
      adjustmentTimeout++;
      if (adjustmentTimeout > AdjustmentTimeoutLimit)
          adjustmentTimeout = 0;
    }
    else
    { // Compare prior difference to current difference - if difference is lessening, don't adjust
      if ( priorDifference >= currentDifference || leftSensorDistance == UltrasonicTimeoutReturnDistance )
      {
        digitalWrite( DEBUG_RED,  LOW  );
        digitalWrite( DEBUG_GREEN, HIGH );  // RIGHT GREEN
        leftMotor.incrementSpeed(); // Turn Right
        #ifdef SerialDebugMode
        Serial.println("Diff < -1: Adjusting to the right...");
        #endif
      }
      priorDifference = currentDifference;
    }
  }
  else // Within acceptable L/R deviation
  {
    digitalWrite( DEBUG_RED,   LOW );
    digitalWrite( DEBUG_GREEN, LOW );
  }

} // end forward()


void turnLeft()
{
  leftMotor.setSpeed( TurnSpeedSlow );
  rightMotor.setSpeed( TurnSpeedFast );
  currentState = StateTurnEnding;
} // end turnLeft()


void turnRight()
{
  leftMotor.setSpeed( TurnSpeedFast );
  rightMotor.setSpeed( TurnSpeedSlow );
  currentState = StateTurnEnding;
} // end turnRight()


/*
 * Disables drift correction watches for turn condition.
 * Should be in this state while the incoming wall is between 15 and 30 inches.
 * At 15 inches, should see which side is open and change to turn state in that direction.
 */
void turnApproaching()
{ 
  digitalWrite( DEBUG_RED,   LOW );
  digitalWrite( DEBUG_GREEN, LOW );

  if( frontSensorDistance < FrontTurnStartDistance ) // Incoming wall less than 15"
  {
    if( leftSensorDistance > rightSensorDistance ) // Left wall further away than right
    {
      currentState = StateTurnLeft;
    }
    else  // Right wall further away than left
    {
      currentState = StateTurnRight;
    }
  }
} // end turnApproaching()

void turnEnding()
{
  static int turnDelayCounter = 0;
  static int finishDelayCounter = 0;
  if( turnDelayCounter < TurnEndingDelay )
  {
      turnDelayCounter++;
  }
  else if ( finishDelayCounter < FinishTurnDelay )
  {
    finishDelayCounter++;
    leftMotor.setSpeed(  StartSpeed );
    rightMotor.setSpeed( StartSpeed );
  }
  else
  { // Check sensors to see if both walls are approximately equal
    if( abs( leftSensorDistance - rightSensorDistance ) < TurnEndingDeviation )
    {
      finishDelayCounter = 0;
      turnDelayCounter = 0;
      currentState = StateForward;
    }
  }
} // end turnEnding()

