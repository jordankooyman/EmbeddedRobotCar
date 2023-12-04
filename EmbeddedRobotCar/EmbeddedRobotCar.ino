// File: EmbeddedRobotCar.ino
// Class: CEN 4930, Fall 2023, CRN 84929
// Contributors: Jordan Kyooyman, David West
// Description: Main project file for an Arduino-based robot car that
//              uses 3 ultrasonic sensors to run a maze.
// -----------------------------------------------------------

#include "config.cpp"
#include "motor.cpp"
#include "ultrasonic.cpp"

// Component declarations
Motor leftMotor;
Motor rightMotor;
UltrasonicSensor frontSensor;
UltrasonicSensor leftSensor;
UltrasonicSensor rightSensor;


// States
enum State {
  StateStop,                // End state for either exiting maze or an imminent crash is detected.
  StateStart,               // Initial state for ramping up motors prior to forward state.
  StateForward,             // Driving forward down a hall of unknown length.
  StateTurnApproaching,     // Incoming wall detected from front sensor, 
                            // prepares for turn and watches for direction
  StateTurnLeft,            // Begins left turn
  StateTurnRight,           // Begins right turn
  StateTurnEnding,          // Initial stage to exit turn
  StateEndTurn              // Final stage to exit turn, prepares to enter forward state
}; // end enum State

State currentState;         // Tracks current state
float leftSensorDistance;   // Distance from left ultrasonic sensor to left wall
float rightSensorDistance;  // Distance from right ultrasonic sensor to right wall
float frontSensorDistance;  // Distance from front ultrasonic sensor to front wall
bool leftTurn;              // Tracks whether current turn is to the left or right
float frontSensorPreTurn;   // Records the front sensor distance before transitioning into turn state


// ----------------- MAIN SETUP ----------------- //
/* One-time setup for components.
 */
void setup() 
{
  leftMotor  = Motor( H1A, H2A, H12EN ); 
  rightMotor = Motor( H3A, H4A, H34EN );

  frontSensor = UltrasonicSensor( FRONT_TRIG, FRONT_ECHO );
  leftSensor  = UltrasonicSensor( LEFT_TRIG,  LEFT_ECHO  );
  rightSensor = UltrasonicSensor( RIGHT_TRIG, RIGHT_ECHO );

  pinMode( DEBUG_RED,   OUTPUT );
  pinMode( DEBUG_GREEN, OUTPUT );
  pinMode( DEBUG_BLUE,  OUTPUT );

  #ifdef SerialDebugMode
  // If debugging, start serial console output.
  Serial.begin( 9600 );
  #endif
  
  currentState = StateStart;  // Initial state = Start

  delay(500);
} // end Setup()


// ----------------- MAIN LOOP ----------------- //
/* Checks sensors and enters appropriate state function 
 * for the robot's current state.
 */
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
  Serial.println();      // Each iteration of the loop is its own line.
  #endif
} // end Loop()


/* Gets left and right side sensor distances (in inches) with each loop. 
 * Gets front sensor distance once every FrontSensorPollDivisor loops.
 */
void checkSensors()
{  
  static int sensorCount = 0;    // Count variable for count controlled loop.
  
  sensorCount++;
  leftSensorDistance = leftSensor.measureInches();
  rightSensorDistance = rightSensor.measureInches();
  if (sensorCount >= FrontSensorPollDivsor)  // Poll front sensor every FrontSensorPollDivsor cycles.
  {
    sensorCount = 0;
    frontSensorDistance = frontSensor.measureInches();
  }
  
  #ifdef SerialDebugMode
  // If debugging, print sensor readings to serial console
  Serial.print( "Left: " );
  Serial.print( leftSensorDistance );
  Serial.print( " | Right: " );
  Serial.print( rightSensorDistance );
  Serial.print( " | Front: " );
  Serial.print( frontSensorDistance );
  #endif
} // end checkSensors()


/* Executes when robot in stop state.
 * Stops motors, remains in this state until reset.
 */
void stop()
{
  static byte red = HIGH;
  static byte green = LOW;
  static byte blue = LOW;
  
  #ifdef SerialDebugMode
    Serial.print( " Stop ");
  #endif 

  #ifndef LEDForwardDebugging
  #ifndef LEDTurnDebugging
  digitalWrite( DEBUG_RED,   red );
  digitalWrite( DEBUG_BLUE,  green );
  digitalWrite( DEBUG_GREEN, blue );
  #endif
  #endif

  leftMotor.run(  Motor::MotorLock );
  rightMotor.run( Motor::MotorLock );

  // Cycle LED Colors in Victory
  byte temp = red;
  red = blue;
  blue = green;
  green = temp;
} // end stop()


/* Initial state of the robot.
 * Ramps up speed of wheel motors to reduce left/right drift on start.
 * Transitions to forward state once motors are running at StartSpeed.
 */
void start()
{
  #ifdef SerialDebugMode
    Serial.print( " Start ");
  #endif 

  static int Speed = MinAdjustSpeed;  // Start motors at an initial speed set in config.cpp

  // Increase speed, starting with left motor
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  rightMotor.setSpeed( Speed );
  
  if (Speed == MinAdjustSpeed) // If first iteration
  {
    rightMotor.run( Motor::MotorForward );
    leftMotor.run(  Motor::MotorForward );
  }
  else if (Speed >= StartSpeed) // Check if up to speed
  {
    currentState = StateForward;
    Speed = MinAdjustSpeed;
    return;
  }

  // Increase speed, this time right motor first.
  Speed+=RampSpeedIncrement;
  rightMotor.setSpeed( Speed );
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  
  if (Speed >= StartSpeed) // Check if up to speed
  {
    currentState = StateForward;
    Speed = MinAdjustSpeed;
    return;
  }

  // Increase speed for next loop.
  Speed+=RampSpeedIncrement;
} // end start()


/* Forward state. Used during straightaways.
 * Robot drives forward until front sensor finds an incoming wall.
 * Transitions to approaching turn state if wall is found.
 * Corrects for deviation away from centerline.
 */
void forward()
{    
  #ifdef SerialDebugMode
    Serial.print( " Forward ");
  #endif 

  // Difference between left and right sensors. 
  // Negative = robot is drifting left. Positive = robot is drifting right.
  float currentDifference = leftSensorDistance - rightSensorDistance;


  #ifdef SerialDebugMode
  Serial.print(" - Current Difference: ");
  Serial.print(currentDifference);
  #endif

  float currentDeviation = abs(currentDifference) / 2;   // Deviation from midline.
  static float previousDeviation = 0;                    // Deviation from the last loop

  #ifdef LEDForwardDebugging
  digitalWrite( DEBUG_RED,   LOW );
  digitalWrite( DEBUG_GREEN, LOW );
  #endif

  // Course Correction Code
  if( leftSensorDistance > rightSensorDistance ) 
  {  // Right of center line
    #ifdef LEDForwardDebugging
    digitalWrite( DEBUG_RED,   LOW );
    digitalWrite( DEBUG_GREEN, HIGH );
    #endif
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall, slow left motor
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset - (CourseCorrectCorrectionFactor * currentDeviation) );
      rightMotor.setSpeed( ForwardSpeed );
    }
    else
    { // Deviation decreasing, approaching midline, slow right motor to decrease angle
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset);
      rightMotor.setSpeed( ForwardSpeed  - (CourseCorrectCorrectionFactor * currentDeviation) ); 
    }
  }
  else
  {  // Left of center line
    #ifdef LEDForwardDebugging
    digitalWrite( DEBUG_RED,   HIGH );
    digitalWrite( DEBUG_GREEN, LOW );
    #endif
    if( currentDeviation >= previousDeviation )
    { // Deviation increasing, approaching wall, slow right motor speed
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset );
      rightMotor.setSpeed( ForwardSpeed - (CourseCorrectCorrectionFactor * currentDeviation) );
    }
    else
    { // Deviation decreasing, approaching midline, slow left motor to decrease angle
      leftMotor.setSpeed(  ForwardSpeed - LeftMotorOffset - (CourseCorrectCorrectionFactor * currentDeviation) );
      rightMotor.setSpeed( ForwardSpeed );
    }
  }

  previousDeviation = currentDeviation;

} // end forward()


/* Runs in forward state prior to forward function.
 * Checks for whether an incoming wall is detected (turn approaching), 
 * a collision is about to occur (stop), or if the robot has left 
 * the maze (stop) and changes the state as appropriate for the next loop.
 */
void checkForStateChange()
{
  // Front sensor detects a wall approaching soon.
  if ( frontSensorDistance < FrontTurnApproachingDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateTurnApproaching;
  
    // Front sensor detects a collision is imminent.
  else if ( frontSensorDistance < FrontStopDistance && frontSensorDistance > MinimumSensorDistance )
    currentState = StateStop;
  
    // Front and side sensors max out, robot has left maze.
  else if ( frontSensorDistance > UltrasonicMaxDistance - 5 && leftSensorDistance > UltrasonicMaxDistance - 5 && rightSensorDistance > UltrasonicMaxDistance - 5 )
    currentState = StateStop;
} // end checkForStateChange()


/* Front sensor detects an incoming wall: a turn is approaching.
 * Disables drift correction and watches for turn condition.
 * Should be in this state while the incoming wall is between 17 and 25 inches.
 * At 17 inches, should see which side is open and change to turn state in that direction.
 */
void turnApproaching()
{ 
  #ifdef DisableTurn // If enabled, stay in forward state to test forward() functionality.
  currentState = StateForward;
  return;
  #endif
  
  #ifdef SerialDebugMode
    Serial.print( " Identify Turn ");
  #endif 

  #ifdef LEDTurnDebugging
  // Blue LED indicates we're in turnApproaching state
  digitalWrite( DEBUG_BLUE,  HIGH );
  #endif

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
      #ifdef LEDTurnDebugging
      digitalWrite( DEBUG_BLUE, LOW );
      #endif
    }
    else  // Right wall further away than left
    {
      currentState = StateTurnRight;
      #ifdef LEDTurnDebugging
      digitalWrite( DEBUG_BLUE, LOW );
      #endif
    }
  }
} // end turnApproaching()


/* Front wall within 17 inches, wall detected to right, no wall to the left.
 * Turn to the left in a pivot about the center.
 */
void turnLeft()
{ 
  #ifdef SerialDebugMode
    Serial.print( " Turn Left ");
  #endif 

  #ifdef LEDTurnDebugging
  // Visual indicator: in StateTurnLeft
  digitalWrite( DEBUG_GREEN, HIGH  );
  #endif

  leftTurn = true;

  static int lastTurnCounter = 0;

  // Turn slower than min motor speeds by pulsing motors
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

  // Front sensor gaining distance over its initial value: change state to turnEnding
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
  { 
    // Ensure motors are running, not locked, before changing states
    leftMotor.run(  Motor::MotorReverse );
    rightMotor.run( Motor::MotorForward );
    currentState = StateTurnEnding;

    #ifdef LEDTurnDebugging
    // Clear LEDs
    digitalWrite( DEBUG_GREEN, LOW );
    #endif
  }
} // end turnLeft()


/* Front wall within 17 inches, wall detected to left, no wall to the right.
 * Turn to the right in a pivot about the center.
 */
void turnRight()
{  
  #ifdef SerialDebugMode
    Serial.print( " Turn Right ");
  #endif 

  #ifdef LEDTurnDebugging
  // Visual indicator: in StateTurnRight
  digitalWrite( DEBUG_RED,   HIGH );
  #endif
    
  leftTurn = false;
  
  static int lastTurnCounter = 0;

  // Turn slower than min motor speeds by pulsing motors
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

  // Front sensor gaining distance over its initial value: change state to turnEnding
  if (frontSensorDistance >= frontSensorPreTurn + FrontSensorPreturnOffset)
  { 
    // Ensure motors are running, not locked, before changing states
    leftMotor.run(  Motor::MotorForward );
    rightMotor.run( Motor::MotorReverse );
    currentState = StateTurnEnding;

    #ifdef LEDTurnDebugging
    // Clear LEDs
    digitalWrite( DEBUG_RED,   LOW );
    #endif
  }
} // end turnRight()


/* The turn is in progress, start watching for the conditions to end the turn.
 */
void turnEnding()
{
  #ifdef SerialDebugMode
    Serial.print( " Finish Turn ");
  #endif 

  #ifdef LEDTurnDebugging
  digitalWrite( DEBUG_BLUE, HIGH );
  #endif

  // Get sensor distance for the sensor on the opposite side from the turn.
  float currentSensorDistance;
  if ( leftTurn )
    currentSensorDistance = rightSensorDistance;
  else
    currentSensorDistance = leftSensorDistance;

  // Turn slower than min motor speeds by pulsing motors
  static int lastTurnCounter = 0;
  if( lastTurnCounter == TurnSoftwarePWMMaxCount )
  {  
    if( leftTurn )
    {
      leftMotor.run(  Motor::MotorForward );
      rightMotor.run( Motor::MotorReverse );
    }
    else // right turn
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

  // Condition for ending the turn:
  // The side sensor now facing what was the front wall is at the distance the front sensor was,
  // with a margin for error.
  if ( currentSensorDistance <= (frontSensorPreTurn + TurnStopSensorDeviation) && currentSensorDistance >= (frontSensorPreTurn - TurnStopSensorDeviation) )
  {
    leftMotor.run(  Motor::MotorLock );  // Enter the next state with motors locked
    rightMotor.run( Motor::MotorLock );
    currentState = StateEndTurn;
    lastTurnCounter = 0;                 // Reset lastTurnCounter for the next turn
    delay( TurnEndingDelay );          // Pause with wheels locked to kill any momentum before endTurn state
  }
} // end turnEnding()


/* Condition met to exit the turn. 
 * Ramp up speed to minimize drift due to one motor starting before the other. Similar to start state.
 */
void endTurn()
{
  static int counter = 0;
  
  #ifdef LEDTurnDebugging
  digitalWrite( DEBUG_RED,   HIGH );
  digitalWrite( DEBUG_GREEN, HIGH );
  digitalWrite( DEBUG_BLUE,  LOW  );
  #endif
  
  #ifdef SerialDebugMode
    Serial.print( " Exit Turn " );
  #endif 

  static int Speed = MinAdjustSpeed;
  
  // increase motor speeds, left motor first
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  rightMotor.setSpeed( Speed );
  
  if (Speed == MinAdjustSpeed) // If first iteration
  {
    rightMotor.run( Motor::MotorForward );
    leftMotor.run(  Motor::MotorForward );
  }
  
  Speed+=RampSpeedIncrement;

  // Alternate increase in motor speeds, right motor first
  rightMotor.setSpeed( Speed );
  leftMotor.setSpeed(  Speed - LeftMotorOffset );
  
  if (Speed < ForwardSpeed)
    Speed+=RampSpeedIncrement;

  // Condition to enter forward state: TurnEndCountDelay loops have passed in this state
  // and the right and left sensors are sensing a wall.
  if( counter >= TurnEndCountDelay && (rightSensorDistance + leftSensorDistance) < TurnExitedDeviation )
  { 
    Speed = MinAdjustSpeed;
    currentState = StateForward;
    counter = 0;    // Reset counter for next turn
    
    #ifdef LEDTurnDebugging
    digitalWrite( DEBUG_RED,   LOW );
    digitalWrite( DEBUG_GREEN, LOW );
    digitalWrite( DEBUG_BLUE,  LOW );
    #endif
  }

  counter++;
} // end endTurn()
