// Libraries
#include  <Sparki.h>

// Constants
#define NO_LCD 
#define MAX_LOOP_MS 100
#define WHEEL_VELOCITY 2.78551532
#define WHEEL_RADIUS 2.54
#define WHEEL_DISTANCES 8.41375

// Global Variables
int i, counter = 0, servoAngle = -30, cmDistance, varTotal = 0, currentVar, sumVar, variancePartial, variance, startTime, endTime, totalTime;
int pingValues[10], varArray[10];

void setup()
{
  sparki.moveStop();
  sparki.servo(-30);
  delay(100);
  sparki.gripperStop();
  sparki.RGB(RGB_OFF);
  delay(100);
  sparki.clearLCD();
  sparki.print("");
  sparki.updateLCD();
} 

void loop()
{
  startTime = millis();

  // Moves Sparki Head / Ultrasound Sensor
  sparki.servo(servoAngle); 
  servoAngle = servoAngle + 1; 
  if(servoAngle > 30)
  { 
    servoAngle = -30;
    Serial.println();
    if(counter == 10)
    {
      varTotal = 0;
      for(i = 0; i < 10; i++)
      {
        varTotal += pingValues[i];
      }
      variancePartial = varTotal / 10;

      for(i = 0; i < 10; i++)
      {
        varArray[i] = (pingValues[i] - variancePartial)^2;
        sumVar += varArray[i];
      }
      variance = sumVar / (10 - 1);
      
      Serial.print("Variance: ");
      Serial.println(variance); 
      
      counter = 0;
    }
  } 

  // Measures the distance with Sparki's eyes 
  cmDistance = sparki.ping();
  Serial.print(" "); 
  Serial.print(cmDistance);  

  // Adds current cm from Beacons to an array at every 10cm
  if(cmDistance <= 45 && cmDistance > 0 && counter < 10)
  {
    pingValues[counter] = cmDistance;
    counter++;
  }
  
  endTime = millis();
  totalTime = MAX_LOOP_MS - (endTime - startTime);
  if(totalTime > 0) delay(totalTime);
}


