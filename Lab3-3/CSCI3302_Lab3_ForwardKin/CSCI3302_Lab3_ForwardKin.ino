#include <Sparki.h> // include the sparki library

#define MAX_LOOP_MS 100
#define WHEEL_VELOCITY 2.785
#define WHEEL_RADIUS 2.54
#define WHEEL_DISTANCES 8.41375

int i, j, cmDetect, north = 300, west = 300, east = 300, south = 300, currentColumn = 0, currentRow = 0, theta = 0, startTime, endTime, totalTime;
float x = 0.0, y = 0.0, rightWheel = 0.0, leftWheel = 0.0, halfLeftRight = 0.0;
int worldMap[4][4] = {{1,1,1,0}, {0,0,1,0}, {1,1,1,1}, {0,1,0,1}};
int bestPathMap[4][4] = {{1,0,0,0},{0,0,0,0},{0,0,0,0},{0,1,0,0}}; 

/*  
 *   worldMap - Hard coded map 3D array of the world 
 *   1 = traverse-able, 0 = blocks
 *  -----------
 *  | 1 1 1 0 |
 *  | 0 0 1 0 |
 *  | 1 1 1 1 |
 *  | 0 1 0 1 |
 *  -----------
 *  
 *   bestPathMap - Path map, initially only showing start and goal, will be filled with optimal path
 *  -----------
 *  | 1 0 0 0 |
 *  | 0 0 0 0 |
 *  | 0 0 0 0 |
 *  | 0 1 0 0 |
 *  -----------
 */

void dijkstra()
{
  // Local Variables
  int incompletedMapNodes[16] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}, mapNodes[16] = {0,50,50,50,50,50,50,50,50,50,50,50,50,50,50,50}, remainingNodes = 10,  minimumNode, minimumCost, newCost;
  
  /* ********************
   * 
   * Dijkstra Part 1
   * Calculates the proper traversal cost between each node 
   * 
   ******************** */
  while(remainingNodes > 0)
  {
    // Resets minimum values to dummy values
    minimumNode = 0;
    minimumCost = 300;

    // Searches for minimums every loop for any incompleted node
    for(i = 0; i < 16; i++)
    {
      if(incompletedMapNodes[i] > 0 && (mapNodes[i] < minimumCost || minimumNode < 0))
      {
        minimumNode = i;
        minimumCost = mapNodes[i];
      }
    }

    // Set row and column to the minimum values divided by 4 and remainder of 4, respectively
    currentRow = minimumNode / 4;
    currentColumn = minimumNode % 4;
    // New adjusted cost to node it min + 1, incompleted node set to completed
    newCost = minimumCost + 1;
    incompletedMapNodes[minimumNode] = 0;

    // Checks for column and row within boundaries and if the current node cost less than the new calc'd cost
    // Also checks if on an incompleted node and if position is not a blocked point.
    // If so, adjust node to new cost.
    // Big if statements :(
    if( currentColumn + 1 <= 3 && 
        mapNodes[(currentRow * 4 + currentColumn + 1)] > newCost && 
        incompletedMapNodes[(currentRow * 4 + currentColumn + 1)] > 0 && 
        worldMap[currentRow][currentColumn + 1] != 0)
      mapNodes[(currentRow * 4 + currentColumn + 1)] = newCost;
    if( currentColumn - 1 >= 0 && 
        mapNodes[(currentRow * 4 + currentColumn - 1)] > newCost && 
        incompletedMapNodes[(currentRow * 4 + currentColumn - 1)] > 0 && 
        worldMap[currentRow][currentColumn - 1] != 0)
        mapNodes[(currentRow * 4 + currentColumn - 1)] = newCost;
    if( currentRow + 1 <= 3 && 
        mapNodes[((currentRow + 1) * 4 + currentColumn)] > newCost && 
        incompletedMapNodes[((currentRow + 1) * 4 + currentColumn)] > 0 && 
        worldMap[currentRow + 1][currentColumn] != 0)
      mapNodes[((currentRow + 1) * 4 + currentColumn)] = newCost;
    if( currentRow - 1 >= 0 && 
        mapNodes[((currentRow - 1) * 4 + currentColumn)] > newCost && 
        incompletedMapNodes[((currentRow - 1) * 4 + currentColumn)] > 0 && 
        worldMap[currentRow - 1][currentColumn] != 0)
      mapNodes[((currentRow - 1) * 4 + currentColumn)] = newCost;

    // One less node left to do
    remainingNodes--;
  }

  /* ********************
   *  
   * Dijkstras Part 2
   * Finds ideal graph this loop, after finding all node distances. Starts from goal to start. 
   * 
   ******************** */
  currentRow = 3, currentColumn = 1;
  while(currentRow > 0 || currentColumn > 0)
  { 
    // Checks for map boundaries, then does compass value settings
    // In order of north, east, south and then west (CLOCKWISE) in priority
    if(currentColumn + 1 <= 3) north = mapNodes[(currentRow * 4 + currentColumn + 1)];
    if(currentColumn - 1 >= 0) south = mapNodes[(currentRow * 4 + currentColumn - 1)];
    if(currentRow + 1 <= 3) east = mapNodes[((currentRow + 1) * 4 + currentColumn)];
    if(currentRow - 1 >= 0) west = mapNodes[((currentRow - 1) * 4 + currentColumn)];

    // Finds shortest path out of compass directions, sets path & updates position
    if(north < east && north < south && north < west)
    {
      bestPathMap[currentRow][currentColumn+1] = 1;
      currentColumn += 1;
    }
    else if(east < north && east < south && east < west)
    {
      bestPathMap[currentRow+1][currentColumn] = 1;
      currentRow += 1;
    }
    else if(south < north && south < east && south < west)
    {
      bestPathMap[currentRow][currentColumn-1] = 1;
      currentColumn -= 1;
    }
    else if(west < north && west < east && west < south)
    {
      bestPathMap[currentRow - 1][currentColumn] = 1;
      currentRow -= 1;
    }
  }
}

void setup()
{
  sparki.moveStop();
  sparki.servo(SERVO_CENTER);
  sparki.gripperStop();
  sparki.RGB(RGB_OFF);
  sparki.print("");
  sparki.updateLCD();
  sparki.clearLCD();
  dijkstra();
}

void loop()
{
  // Checks if goal reached, if so, stop
  if(currentRow == 3 && currentColumn == 1)
  {
    sparki.RGB(RGB_OFF);
    sparki.moveStop();
    sparki.RGB(RGB_GREEN);
    while(true);
  }
  
  // Records start time
  startTime = millis();
  // Reset values
  cmDetect = 0, north = 0, west = 0, east = 0, south = 0;

  // Determines X and Y using half of summed wheel speeds
  halfLeftRight = (rightWheel + leftWheel) / 2;
  x += cos((theta * (PI/180))) * halfLeftRight;
  y += sin((theta * (PI/180))) * halfLeftRight;
  // Theta, now set when turning. Only turns 90 degrees
  //theta += (rightWheel - leftWheel) * (WHEEL_VELOCITY / 2);
  // Makes sure theta stays in boundaries
  if(theta >= 360)
    theta = fmod(theta, 360);
  if(theta <= -360)
    theta = -fmod(abs(theta), 360);

  // Set current Column and Row values from X and Y, done for scaling
  currentColumn = x / 150;
  currentRow = y / 105;

  // Object Detection, changes world map if sees object within 20 cm
  cmDetect = sparki.ping();
  if(cmDetect != -1)
  {
    if(cmDetect < 10)
    {
      // Northern Object
      if(currentColumn+1 <= 3 && (theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
        bestPathMap[currentRow][currentColumn+1] = worldMap[currentRow][currentColumn+1] = 0;
      // East
      else if(currentRow + 1 <= 3 && (theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
        bestPathMap[currentRow+1][currentColumn] = worldMap[currentRow+1][currentColumn] = 0;
      // West
      else if(currentRow-1 >= 0 && (theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85))
        bestPathMap[currentRow-1][currentColumn] = worldMap[currentRow-1][currentColumn] = 0;
      // South
      else if(currentColumn-1 >= 0 && (theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
        bestPathMap[currentRow][currentColumn-1] = worldMap[currentRow][currentColumn-1] = 0;
    }
  }

  // Prints onto Screen Everything needed.
  sparki.clearLCD(); // wipe the screen
  sparki.print("  Row: ");
  sparki.print(currentRow+1);
  sparki.print("  Column: ");
  sparki.print(currentColumn+1);
  sparki.print("\n D:");
  sparki.print(theta);
  sparki.print(" X:");
  sparki.print(x/100);
  sparki.print(" Y:");
  sparki.print(y/100);
  sparki.print("\n      World Map\n");
  // Prints Map
  for(i = 0; i < 4; i++)
  {
    sparki.print("       ");
    for(j = 0; j < 4; j++)
    {
      if(bestPathMap[i][j] >= 1 && i == currentRow && j == currentColumn)
        sparki.print("R");
      else
        sparki.print(worldMap[i][j]);
      sparki.print(" ");
    }
    sparki.print("\n");
  }
  // Prints
  if(currentRow == 3 && currentColumn == 1)
    sparki.print("    GOAL  REACHED");
  sparki.updateLCD();

  // Checks boundary limits, if in bounds, adjust compass values, if not, set to 0 for blockage
  if(currentColumn+1 <= 3) north = bestPathMap[currentRow][currentColumn+1];
  else north = 0;
  if(currentRow + 1 <= 3) east = bestPathMap[currentRow+1][currentColumn];
  else east = 0;
  if(currentRow-1 >= 0) west = bestPathMap[currentRow-1][currentColumn];
  else west = 0;
  if(currentColumn-1 >= 0) south = bestPathMap[currentRow][currentColumn-1];
  else south = 0;

  // Now Does Traversal, based on compass values and theta for rotation
  // Proritizes Clockwise Motion
  if(north == 1)
  {
    if((theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = WHEEL_VELOCITY;
      sparki.moveForward();
    }
    else if((theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta -= 90;
      sparki.moveLeft(90);
    }
    else if((theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 90;
      sparki.moveRight(90);
    }
    else if((theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 180;
      sparki.moveRight(180);
    }
  } 
  else if(east == 1)
  {  
    if((theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = WHEEL_VELOCITY;
      sparki.moveForward();
    }
    else if((theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta -= 90;
      sparki.moveLeft(90);
    }
    else if((theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 90;
      sparki.moveRight(90);
    }
    else if(theta >= ((theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85)))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 180;
      sparki.moveRight(180);
    }
  }
  else if(south == 1)
  {
    if((theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = WHEEL_VELOCITY;
      sparki.moveForward();
    }
    else if((theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta -= 90;
      sparki.moveLeft(90);
    }
    else if((theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 90;
      sparki.moveRight(90);
    }
    else if((theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 180;
      sparki.moveRight(180);
    }
  }
  else if(west == 1)
  {
    if((theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = WHEEL_VELOCITY;
      sparki.moveForward();
    }
    else if((theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta -= 90;
      sparki.moveLeft(90);
    }
    else if((theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 90;
      sparki.moveRight(90);
    }
    else if((theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
    {
      rightWheel = WHEEL_VELOCITY;
      leftWheel = -WHEEL_VELOCITY;
      theta += 180;
      sparki.moveRight(180);
    }
  } 
  else
  {
    sparki.moveStop();
    sparki.RGB(RGB_RED);
    while(true);
  }

  // Set current location to 2 to signify that this path has been traversed
  bestPathMap[currentRow][currentColumn] = 2;

  // Records time at end of loop, also sets total time out of 100ms
  endTime = millis();
  totalTime = MAX_LOOP_MS - (endTime - startTime);

  // Checks ifthe total time is under or over 100ms
  // If there is any remainder time, delay for that amount
  if(totalTime > 0)
  {
    delay(totalTime);
  }
}
