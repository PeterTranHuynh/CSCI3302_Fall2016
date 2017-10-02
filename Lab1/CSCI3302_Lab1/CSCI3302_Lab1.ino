#include <Sparki.h>

#define SEARCH 1
#define DRIVE 2
#define PICKUP 3
#define ROTATE 4
#define TOLINE 5
#define FOLLOWLINE 6
#define DROP 7

// Yolo
int stateId = SEARCH;

void setup() {
  // put your setup code here, to run once:
  sparki.servo(SERVO_CENTER); // center the servo
  sparki.gripperOpen();
  delay(3000);
  sparki.gripperStop();
}

void loop() {
  // put your main code here, to run repeatedly:
  int threshold = 900;
  bool endLoop = false;
  int cm, lineLeft, lineRight, lineCenter;

  if(stateId > 7 || stateId < 1)
  {
    sparki.moveStop();
    sparki.RGB(RGB_RED);
  }
  
  while(!endLoop)
  {
    switch(stateId)
    {
      case SEARCH:
        sparki.moveLeft(5);
        cm = sparki.ping();
        if(cm != -1)
        {
          if(cm < 25)
          {
            sparki.beep();
            sparki.moveLeft(6);
            stateId = DRIVE;
          }
        }
        delay(100);
        break;
        
      case DRIVE:
        sparki.moveForward(5);
        cm = sparki.ping();
        if(cm != -1)
        {
          if(cm < 5)
          {
            sparki.beep();
            sparki.moveForward(2);
            stateId = PICKUP;
          }
        }
        delay(100);
        break;
      
      case PICKUP:
        sparki.beep();
        sparki.gripperClose();
        delay(3000);
        sparki.gripperStop();
        stateId = ROTATE;
        break;
      
      case ROTATE:
        sparki.beep();
        sparki.moveLeft(180);
        stateId = TOLINE;
        break;
      
      case TOLINE:
        lineLeft   = sparki.lineLeft();   // measure the left IR sensor
        lineCenter = sparki.lineCenter(); // measure the center IR sensor
        lineRight  = sparki.lineRight();  // measure the right IR sensor

        if ( lineCenter < threshold && lineLeft < threshold && lineRight < threshold  )
        {
          sparki.beep();
          sparki.moveForward(4); // move forward 
          sparki.moveRight(90);
          stateId = FOLLOWLINE;
        }
        else
        {
          if ( lineCenter < threshold )
          {
            sparki.moveForward(4); // move forward
            if ( lineCenter < threshold )
            {
              stateId = FOLLOWLINE;
            }
          }
          if (lineLeft < threshold)
          {
            sparki.moveLeft(45);
            if ( lineCenter < threshold )
            {
              stateId = FOLLOWLINE;
            }
          }
          if (lineLeft < threshold)
          {
            sparki.moveRight(45);
            if ( lineCenter < threshold )
            {
              stateId = FOLLOWLINE;
            }
          }
          else
          {
            sparki.moveForward(); // move forward 
          }
        }

        sparki.clearLCD(); // wipe the screen
  
        sparki.print("Line Left: "); // show left line sensor on screen
        sparki.println(lineLeft);
  
        sparki.print("Line Center: "); // show center line sensor on screen
        sparki.println(lineCenter);
  
        sparki.print("Line Right: "); // show right line sensor on screen
        sparki.println(lineRight);
  
        sparki.updateLCD(); // display all of the information written to the screen

        delay(100); // wait 0.1 seconds
        break;
        
      case FOLLOWLINE:
        lineLeft   = sparki.lineLeft();   // measure the left IR sensor
        lineCenter = sparki.lineCenter(); // measure the center IR sensor
        lineRight  = sparki.lineRight();  // measure the right IR sensor

        if ( lineCenter < threshold ) // if line is below left line sensor
        {
          if (lineLeft < threshold && lineRight < threshold)
          {
            stateId = DROP; 
          }
          sparki.moveForward(); // move forward
        }
        else
        {
          if ( lineLeft < threshold ) // if line is below left line sensor
          {  
            sparki.moveLeft(); // turn left
          }
  
          else if ( lineRight < threshold ) // if line is below right line sensor
          {  
            sparki.moveRight(); // turn right
          }
        }

        sparki.clearLCD(); // wipe the screen
  
        sparki.print("Line Left: "); // show left line sensor on screen
        sparki.println(lineLeft);
  
        sparki.print("Line Center: "); // show center line sensor on screen
        sparki.println(lineCenter);
  
        sparki.print("Line Right: "); // show right line sensor on screen
        sparki.println(lineRight);
  
        sparki.updateLCD(); // display all of the information written to the screen

        delay(100); // wait 0.1 seconds
        break;
      
      case DROP:
        sparki.beep();
        sparki.moveStop();
        sparki.gripperOpen();
        delay(3000);
        sparki.gripperStop();
        endLoop = true;
        stateId = -1;
        break;
    
      default:
        endLoop = true;
        break;
  }
}
}
