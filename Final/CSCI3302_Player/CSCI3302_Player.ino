/**  Libraries  **/
#include <Sparki.h>

/**  Constants  **/
#define WHEEL_VELOCITY    2.785
#define WHEEL_RADIUS      2.54
#define WHEEL_DISTANCES   8.41375
#define SEARCH            1
#define RETRIEVE          2
#define PICKUP            3
#define REPOSITION        4
#define DROP              5
#define KICK              6

/**  Global Variables  **/
// Used for state machine switch statement
int stateId = SEARCH;
int threshold = 900;
int i, cm, lineLeft, lineRight, lineCenter, pingValues[10], distance = 0, servoAngle = -80, counter;
// Simple Odometry Variables
float x = 0.0, y = 0.0, theta = 0.0;  //, rightWheel = 0.0, leftWheel = 0.0, halfLeftRight = 0.0;
// Measures Time
int startT, endT;

/*  
 *   INITIAL FIELD FOR SPARKI SOCCER 
 *   #        = Goalie
 *   0        = Ball
 *   S        = Sparki Start
 *   < ^ v >  = Vector Position
 *   
 *    (0,81)            (61,81)
 *        ---|--------|---
 *         |            |
 *         |      #     |
 *         |      v     |
 *         | -  -  -  - |
 *         |            |
 *         |     0      |
 *         |S>          |
 *        ---|--------|---
 *     (0,0)            (61,0)
 */

/**  Functions  **/
/*
 * Setup Function for Sparki initial start-up
 */
void setup()
{
  sparki.servo(SERVO_CENTER);
  sparki.gripperOpen();
  delay(3000);
  sparki.gripperStop();
  sparki.moveStop();
  sparki.RGB(RGB_OFF);
  sparki.print("");
  sparki.updateLCD();
  sparki.clearLCD();
}

/*
 * Loop Function that repeats after setup of Sparki
 */
void loop()
{
  if(stateId > 7 || stateId < 1)
  {
    sparki.moveStop();
    sparki.RGB(RGB_RED);
  }
  
  sparki.clearLCD();
  sparki.print(" Theta:");
  sparki.print(theta);
  sparki.print("\n X:");
  sparki.print(x);
  sparki.print("\n Y:");
  sparki.print(y);
  sparki.updateLCD();
  switch(stateId)
  {
    case SEARCH:
      if(counter == 10)
      {
        for(i = 0; i < 10; i++)
        {
          distance += pingValues[i];
        }
        distance = distance / 10;
        theta = fmod(theta-(servoAngle), 360);
        sparki.moveLeft(theta);
        sparki.servo(SERVO_CENTER);
        stateId = RETRIEVE;
        break;
      }
      
      sparki.servo(servoAngle); 
      servoAngle += 1;

      if(servoAngle > 45 && counter != 10)
      { 
        servoAngle = -80;
        theta = fmod(theta+5, 360);
        sparki.moveLeft(5);
        counter = 0;
      }
      
      cm = sparki.ping();
      if(cm <= 40 && cm > 0 && counter < 10)
      {
        pingValues[counter] = cm;
        counter++;
      }
      break;
      
    case RETRIEVE:
      startT = millis();
      sparki.moveForward(distance-5);
      endT = millis();
      x += cos((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      y += sin((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      stateId = PICKUP;
      delay(100);
      break;
    
    case PICKUP:
      sparki.gripperClose();
      delay(2000);
      sparki.gripperStop();
      if(theta != 90.0)
      {
        sparki.moveLeft(90-theta);
        theta = 90.0;  
      }
      sparki.gripperOpen();
      delay(2000);
      sparki.gripperStop();
      stateId = REPOSITION;
      break;
    
    case REPOSITION:
      startT = millis();
      sparki.moveForward(3);
      endT = millis();
      x += cos((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      y += sin((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      if(y <= 32 && y >= 28) stateId = DROP;
      break;
    
    case DROP:
      if(x <= 16)
      {
        theta = 45;
        sparki.moveRight(45);
      }
      else if(x >= 45)
      {
        theta = 140;
        sparki.moveLeft(45);
      }
      stateId = KICK;
      break;
       
    case KICK:
      startT = millis();
      sparki.moveForward(10);
      endT = millis();
      x += cos((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      y += sin((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      startT = millis();
      sparki.moveBackward(32);
      endT = millis();
      x -= cos((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      y -= sin((theta * (PI/180))) * WHEEL_VELOCITY *((endT-startT)/1000);
      stateId = SEARCH;
      counter = 0;
      break;
  
    default:
      stateId = SEARCH;
      break;
  }
}
