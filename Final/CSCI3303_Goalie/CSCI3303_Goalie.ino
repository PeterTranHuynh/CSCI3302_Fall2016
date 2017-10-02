#include <Sparki.h>
#include <math.h>



int _startAngle, _endAngle, _goalLength, _iterator, state, data[3][2], interceptLoc;
unsigned long _start;

//data:   angle // length // time
//        angle2 // length2 // time2

void setup() {
  #define LEFT (true)
  sparki.clearLCD();
  _goalLength = 30;    //hardcoded length of goal posts
  if (LEFT) {
    _startAngle = -66;
    _endAngle = -30;
    _iterator = 2;
  }

  if (not LEFT) {
    _startAngle = 66;
    _endAngle = 30;
    _iterator = -2;
  }
  sparki.servo(_startAngle + _iterator);
  state = 1;
  delay(500);
}

void loop() {
  #define SCANNING 1
  #define SCANNING_2 2
  #define CALCULATING 3
  #define DEFENDING 4
  #define THRESHOLD 40 //distance away
  switch (state)
  {
    case SCANNING:
    {
      //Scan for a ping that is within the threshold distance
      int ping = sparki.ping();
      int iterator = _iterator;
      int currAngle = _startAngle;
      sparki.println("Scanning for first object");
      sparki.updateLCD();
      while (ping > THRESHOLD) {
        currAngle += iterator;
        sparki.servo(currAngle);
        delay(14);  //Delay to ensure servo head turns.   SOMETHING WE SHOULD TIME WHEN WE HAVE SPARKI
                            //Also test to see if delay affects millis()
        //When it turns closest to Goal line 
        if (abs(currAngle) <= abs(_endAngle))  {
          iterator = -iterator;
        }
        //When it turns all the way towards the field 
        else if (abs(currAngle) >= abs(_startAngle)) {
          iterator = -iterator;
        }
        ping = sparki.ping();
      }
      sparki.println("Found object 1");
      sparki.print("Ping:  ");
      sparki.println(ping);
      sparki.updateLCD();
      _start = millis();
      data[0][0] = currAngle;
      data[1][0] = ping;
      data[2][0] = _start;
      state = SCANNING_2;
      //state = CALCULATING;
      //data[2][1] = _start + 5000;
      break;
    }

     case SCANNING_2:
     {
      //Find a second reading of the object - requires seeing the ball a second time
      sparki.servo(_endAngle);
      sparki.println("Scanning for second object");
      sparki.updateLCD();
      int ping = sparki.ping() + THRESHOLD;    
      int iterator = - _iterator;
      int currAngle = _endAngle;
      delay(25);
      while (ping > THRESHOLD and millis() < 4000) {
        currAngle += iterator;
        sparki.servo(currAngle);
        delay(10);  //Delay to ensure servo head turns.   SOMETHING WE SHOULD TIME WHEN WE HAVE SPARKI
        
        //When it turns closest to Goal line 
        if (abs(currAngle) <= abs(_endAngle))  {
          iterator = -iterator;
        }
        //When it turns all the way towards the field 
        else if (abs(currAngle) >= abs(_startAngle)) {
          iterator = -iterator;
        }
        ping = sparki.ping();
      }
      data[2][1] = millis();
      data[0][1] = currAngle;
      data[1][1] = ping;
      state = CALCULATING;

      sparki.println("Found 2nd object");
      sparki.updateLCD();
     }

    case CALCULATING:
    {
      //Calculate trajectory if ping difference is less than 2.5 seconds
      unsigned long timer = data[1][1] - data[1][0];
      
      if (timer < 4000) {
        //Convert to Cartesian 
        // x = r * cos ( theta )        y = r * sin ( theta )
        float x1 = data[1][0] * cos( (data[0][0] * 3.14159 / 180));
        float y1 = data[1][0] * sin( (data[0][0] * 3.14159 / 180)); 
        float x2 = data[1][1] * cos( (data[0][1] * 3.14159 / 180));
        float y2 = data[1][1] * sin( (data[0][1] * 3.14159 / 180)); 
        float distance = sqrt( pow((x2 - x1),2) + pow((y2 - y1),2) );
  
        float velocity = distance / timer;

        float scaler = y2 / abs(y2 - y1);
         
        interceptLoc = abs(x2 + abs(x2 - x1) * scaler);
      }


      //Else assume object is coming perpendicular into the goal, 2nd reading took too long.  Assuming 2nd object
      else {
        float x1 = data[1][0] * cos( (data[0][0] * 3.14159 / 180));

        interceptLoc = abs(x1);  
      }

      if (interceptLoc > _goalLength) {
        interceptLoc = _goalLength;
      }

      if (interceptLoc < 0) {
        interceptLoc = 5;
      }
      

      state = DEFENDING;

    }
      
       
      

    case DEFENDING:
    {
      //Move towards intercept location
      sparki.print("Moving to ");
      sparki.println(interceptLoc);
      sparki.updateLCD(); 
      sparki.moveForward(interceptLoc);
      delay(1000);
      state = 0;
      break;
    }

    default:
      sparki.RGB(RGB_RED);
      break;
      
      
  }

}
