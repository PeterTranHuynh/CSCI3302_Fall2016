// Libraries
#include <Sparki.h>

// Constants in ms, cm, or values
#define MAX_LOOP_MS 100
#define THRESHOLD 0.5
#define WHEEL_VELOCITY 2.785
#define WHEEL_RADIUS 2.54
#define WHEEL_DISTANCES 8.41375
#define X_GOAL 50.00
#define Y_GOAL -18.00
#define THETA_GOAL 0.04
#define P1 0.01
#define P2 0.01
#define P3 0.001

// Global Variables
int startTime, endTime, totalTime, goal = 0;
float thetaSum = 0.0, x = 0.0, y = 0.0, halfLeftRight = 0.0, leftWheel = 0.0, rightWheel = 0.0, mu = 0.0;
float theta = 0.0, phiRight = 0.0, phiLeft = 0.0, rho = 0.0, alpha = 0.0, forwardSpeed = 0.0, rotationalSpeed = 0.0;

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

  // Math for determining our position, theta, and such.
  rho = sqrt(((x - X_GOAL)*(x - X_GOAL)) + ((y - Y_GOAL)*(y - Y_GOAL)));
  alpha = (thetaSum - atan2((y - Y_GOAL), (x - X_GOAL)));

  mu = THETA_GOAL - thetaSum;
  
  forwardSpeed = P1 * rho;
  rotationalSpeed = (P2 * alpha) + (P3 * mu);

  if (goal != 1)
  {
    phiRight = (((2.0 * forwardSpeed) / (WHEEL_RADIUS)) + (rotationalSpeed * WHEEL_DISTANCES)) / 2.0;
    phiLeft =  (((2.0 * forwardSpeed) / (WHEEL_RADIUS)) - (rotationalSpeed * WHEEL_DISTANCES)) / 2.0;
  }
  if (phiLeft > WHEEL_VELOCITY)
  {
    phiLeft = WHEEL_VELOCITY;
  }
  
  if (phiLeft > WHEEL_VELOCITY)
  {
    phiLeft = WHEEL_VELOCITY;
  }
    if (phiLeft < -WHEEL_VELOCITY)
  {
    phiLeft = -WHEEL_VELOCITY;
  }
  
  if (phiLeft < -WHEEL_VELOCITY)
  {
    phiLeft = -WHEEL_VELOCITY;
  }
  
  //halfLeftRight = (rightWheel + leftWheel) / 2;
  halfLeftRight = (phiRight + phiLeft) / 2;
  //x += cos(theta) * halfLeftRight;
  //y += sin(theta) * halfLeftRight;
  x += cos(thetaSum) * forwardSpeed;
  y += sin(thetaSum) * forwardSpeed;
  // Equation from Page 54, Number 3.39
  // Determines Theta when moving to goal
  if ( goal != 1 )
  {
    theta = (((phiRight * WHEEL_RADIUS) / WHEEL_DISTANCES) - ((phiLeft * WHEEL_RADIUS) / WHEEL_DISTANCES));
    thetaSum = (phiRight- phiLeft) * (WHEEL_VELOCITY / 2);

  }

  // Draws our x and position onto the screen, with a small buffer to ensure accuracy and centered screen
  //sparki.drawPixel((x/10) + X_SCREEN_BUFFER, (y/10) + Y_SCREEN_BUFFER); 
  // Displays all Math variables on the screen
  sparki.clearLCD();
  sparki.print(" X: ");
  sparki.print(x);
  sparki.print(" Y: ");
  sparki.println(y);
  sparki.print(" L: ");
  sparki.print(phiLeft*5);
  sparki.print(" R: ");
  sparki.println(phiRight*5);
  sparki.print(" Theta: ");
  sparki.println(thetaSum);
  sparki.print(" Rho: ");
  sparki.println(rho);
  sparki.print(" Alpha: ");
  sparki.println(alpha);
  sparki.print(" Forward: ");
  sparki.println(forwardSpeed);
  sparki.print(" Rotational: ");
  sparki.println(rotationalSpeed);
  sparki.updateLCD();

  if(rho <= THRESHOLD && rho >= -THRESHOLD)
  {
    sparki.RGB(RGB_GREEN);
    goal = 1;
  }
  // Moves Sparki to Goal if goal not achieved
  if ( goal != 1 )
  {
    if (phiLeft > 0)
      sparki.motorRotate(MOTOR_LEFT, DIR_CCW, abs((phiLeft / WHEEL_VELOCITY)*100)*5);
    else
      sparki.motorRotate(MOTOR_LEFT, DIR_CW, abs((phiLeft / WHEEL_VELOCITY)*100)*5);

    if (phiRight > 0)
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW, abs((phiRight / WHEEL_VELOCITY)*100)*5);
    else
      sparki.motorRotate(MOTOR_RIGHT, DIR_CW, abs((phiRight / WHEEL_VELOCITY)*100)*5);
  }

  if( goal == 1 && (thetaSum <= THETA_GOAL + THRESHOLD && thetaSum >= THETA_GOAL - THRESHOLD))
  {
    sparki.motorStop(MOTOR_LEFT);
    sparki.motorStop(MOTOR_RIGHT);
    phiLeft = 0, phiRight = 0;
  }
  if ( goal == 1 && (thetaSum >= THETA_GOAL + THRESHOLD && thetaSum <= THETA_GOAL - THRESHOLD))
  {
    sparki.motorRotate(MOTOR_LEFT, DIR_CCW, 100);
    sparki.motorRotate(MOTOR_RIGHT, DIR_CW, 100);
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
