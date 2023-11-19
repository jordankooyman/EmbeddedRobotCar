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
      forward();
      break;
    case StateCourseCorrectLeft:
      courseCorrectLeft();
      break;
    case StateCourseCorrectRight:
      courseCorrectRight();
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
  #ifdef SerialDebugMode
  Serial.println();
  #endif
} // end Loop()


void checkSensors()
{  
  digitalWrite( DEBUG_BLUE, LOW );
  
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
  
  digitalWrite( DEBUG_BLUE, HIGH );
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
  #ifdef SerialDebugMode
    Serial.print( " Forward ");
  #endif 

  // Update Future State as needed
  float currentDifference = leftSensorDistance - rightSensorDistance;
  checkForStateChange();
  
  #ifdef SerialDebugMode
  Serial.print(" - Current Difference: ");
  Serial.print(currentDifference);
  #endif

  if ( currentDifference > SignificantSensorDeviation  )  // Robot deviating to the right
  { 
    currentState = StateCourseCorrectLeft;
  }
  else if ( currentDifference < -SignificantSensorDeviation ) // Robot deviating to the left
  { 
    currentState = StateCourseCorrectRight;
  }

} // end forward()


void checkForStateChange()
{
  if ( frontSensorDistance < FrontTurnApproachingDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateTurnApproaching;
  if ( frontSensorDistance < FrontStopDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateStop;
} // end checkForStateChange()


// Too far right
void courseCorrectLeft()
{
  #ifdef SerialDebugMode
    Serial.print( " CC Left ");
  #endif 

  digitalWrite( DEBUG_RED, HIGH );
  
  static int adjustmentTimeout = 0;
  static float priorDifference = 0;
  static float filteredRightSensorDistance = UltrasonicMaxDistance;

  if (rightSensorDistance < (filteredRightSensorDistance * SensorChangeMultiplier))
    filteredRightSensorDistance = rightSensorDistance;
  float currentDifference = leftSensorDistance - filteredRightSensorDistance;

  checkForStateChange();
  // Course Correct every <adjustmentTimeoutLimit> loops
  if (adjustmentTimeout)
  { 
    adjustmentTimeout++;
    if (adjustmentTimeout > AdjustmentTimeoutLimit)
        adjustmentTimeout = 0;
  }
  else
  { 
    // Check if angle is too large, catching overcorrection
    float sensorSum = leftSensorDistance + rightSensorDistance;
    if( sensorSum > MaxAngledDistance ) 
    {
      leftMotor.incrementSpeed();  
     
      #ifdef SerialDebugMode
        Serial.print( " | Angle too large!");
      #endif
    }
    // Compare prior difference to current difference - if difference is lessening, don't adjust
    else if( priorDifference <= currentDifference )
    {
      leftMotor.decrementSpeed();  // Turn left
      #ifdef SerialDebugMode
      Serial.print(" | Diff > 1: Adjusting to the left...");
      #endif
    }
      
      // Angle OK, position improving
    else if ( priorDifference > currentDifference )
    {
      currentState = StateForward;
    }

    priorDifference = currentDifference;
  } // end adjustmentTimeout == 0;
  
  digitalWrite( DEBUG_RED, LOW );  // LEFT RED LED OFF
} // end courseCorrectLeft()


// Too far left
void courseCorrectRight()
{
  #ifdef SerialDebugMode
    Serial.print( " CC Right ");
  #endif 

  digitalWrite( DEBUG_GREEN, HIGH );  // RIGHT GREEN

  static int adjustmentTimeout = 0;
  static float priorDifference = 0;
  static float filteredLeftSensorDistance = UltrasonicMaxDistance;

  if (leftSensorDistance < (filteredLeftSensorDistance * SensorChangeMultiplier))
    filteredLeftSensorDistance = leftSensorDistance;

  float currentDifference = filteredLeftSensorDistance - rightSensorDistance;
  
  checkForStateChange();

  // Course Correct every <AdjustmentTimeoutLimit> loops
  if (adjustmentTimeout)
  {
    adjustmentTimeout++;
    if (adjustmentTimeout > AdjustmentTimeoutLimit)
        adjustmentTimeout = 0;
  }
  else
  { 
    float sensorSum = filteredLeftSensorDistance + rightSensorDistance;
    
    if( sensorSum > MaxAngledDistance ) 
    {
      leftMotor.decrementSpeed();  // Angle is too large, start angle reduction.

      #ifdef SerialDebugMode
        Serial.print( " | Angle too large!");
      #endif
    }
    
    // Compare prior difference to current difference - if difference is lessening, don't adjust
    else if( priorDifference >= currentDifference )
    {
      leftMotor.incrementSpeed(); // Turn Right
      #ifdef SerialDebugMode
      Serial.print(" | Diff < -1: Adjusting to the right...");
      #endif
    }

      // Angle OK, position improving
    else if ( priorDifference < currentDifference )
    {
      currentState = StateForward;
    }

    priorDifference = currentDifference;
  } // end adjustmentTimeout == 0

  digitalWrite( DEBUG_GREEN, LOW );
} // end courseCorrectRight()


/*
 * Disables drift correction watches for turn condition.
 * Should be in this state while the incoming wall is between 15 and 30 inches.
 * At 15 inches, should see which side is open and change to turn state in that direction.
 */
void turnApproaching()
{ 
  #ifdef SerialDebugMode
    Serial.print( " Pre-Turn ");
  #endif 

  digitalWrite( DEBUG_RED,   HIGH );
  digitalWrite( DEBUG_GREEN, HIGH );


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


void turnLeft()
{
  #ifdef SerialDebugMode
    Serial.print( " Turn Left ");
  #endif 

  leftMotor.setSpeed( TurnSpeedSlow );
  rightMotor.setSpeed( TurnSpeedFast );
  currentState = StateTurnEnding;
} // end turnLeft()


void turnRight()
{
  #ifdef SerialDebugMode
    Serial.print( " Turn Right ");
  #endif 

  leftMotor.setSpeed( TurnSpeedFast );
  rightMotor.setSpeed( TurnSpeedSlow );
  currentState = StateTurnEnding;
} // end turnRight()


void turnEnding()
{
  #ifdef SerialDebugMode
    Serial.print( " Post-Turn ");
  #endif 

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

