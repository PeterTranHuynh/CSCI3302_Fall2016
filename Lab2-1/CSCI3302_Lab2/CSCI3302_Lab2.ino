// Libraries
#include <Sparki.h>

// Constants
#define MAX_LOOP_MS 100
#define THRESHOLD 900
#define WHEEL_VELOCITY 0.02785
#define X_SCREEN_BUFFER 40
#define Y_SCREEN_BUFFER 60

// Global Variables
int startTime, endTime, totalTime, lineLeft, lineCenter, lineRight;
float theta = 0.0, x = 0.0, y = 0.0, rightWheel = 0.0, leftWheel = 0.0, halfLeftRight = 0.0;

void setup()
{
  // Resets everything for program startup
  sparki.moveStop();
  sparki.servo(SERVO_CENTER);
  sparki.gripperStop();
  sparki.RGB(RGB_OFF);
  sparki.print("");
  sparki.updateLCD();
  sparki.clearLCD();
}

void loop() {
  // Records the time at starting of loop
  startTime = millis();

  // Taken from the LineFollowing program example, with minor additions to track both wheels
  lineLeft   = sparki.lineLeft();   // measure the left IR sensor
  lineCenter = sparki.lineCenter(); // measure the center IR sensor
  lineRight  = sparki.lineRight();  // measure the right IR sensor

  // Same as line following example, except we set the wheel values for later use in x and y
  if ( lineCenter < THRESHOLD ) // if line is below left line sensor
  {
    rightWheel = WHEEL_VELOCITY;
    leftWheel = WHEEL_VELOCITY;
    
    sparki.moveForward(); // move forward
  }
  else
  {
    if ( lineLeft < THRESHOLD ) // if line is below left line sensor
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      
      sparki.moveLeft(); // turn left
    }
  
    if ( lineRight < THRESHOLD ) // if line is below right line sensor
    {  
      rightWheel = -WHEEL_VELOCITY;
      leftWheel = WHEEL_VELOCITY;
      
      sparki.moveRight(); // turn right
    }
  }

  // Math for determining our position, theta, and such.
  halfLeftRight = (rightWheel + leftWheel) / 2;
  x += cos(theta) * halfLeftRight * MAX_LOOP_MS;
  y += sin(theta) * halfLeftRight * MAX_LOOP_MS;
  theta += (rightWheel - leftWheel) * MAX_LOOP_MS * (WHEEL_VELOCITY / 2);

  // Draws our x and position onto the screen, with a small buffer to ensure accuracy and centered screen
  sparki.drawPixel((x/10) + X_SCREEN_BUFFER, (y/10) + Y_SCREEN_BUFFER); 
  sparki.updateLCD();

  // Resets math variables and screen when at the start line
  if (lineCenter < THRESHOLD && lineRight < THRESHOLD && lineLeft < THRESHOLD)
  {
    sparki.print("");
    sparki.updateLCD();
    sparki.clearLCD();
    theta = 0.0, x = 0.0, y = 0.0, rightWheel = 0.0, leftWheel = 0.0, halfLeftRight = 0.0;
  }

  // Records time at end of loop, also sets total time out of 100ms
  endTime = millis();
  totalTime = MAX_LOOP_MS - (endTime - startTime);

  // Checks if the total time is under or over 100ms
  // If there is any remainder time, delay for that amount
  // Else (totalTime negative) flash red LED to show that it exceeded 100ms
  if (totalTime > 0)
  {
    delay(totalTime);
  }
  else if (totalTime < 0)
  {
    sparki.RGB(RGB_RED);
  }
}
