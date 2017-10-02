// Libraries
#include <Sparki.h>

// Constants
#define MAX_LOOP_MS 100
#define WHEEL_VELOCITY 2.78551532
#define WHEEL_RADIUS 2.54
#define WHEEL_DISTANCES 8.41375
#define P1 0.1
#define P2 0.005
#define P3 0.01

// Global Variables
int i, j, cmDetect, north = 10, west = 10, east = 10, south = 10, currentColumn = 0, currentRow = 0, startTime, endTime, totalTime;
float x = 0.0, y = 0.0, theta = 0.0, rightWheel = 0.0, leftWheel = 0.0, halfLeftRight = 0.0;
int worldMap[4][4] = {{1,1,1,0}, {0,0,1,0}, {1,1,1,1}, {0,1,0,1}};
int bestPathMap[4][4] = {{1,0,0,0},{0,0,0,0},{0,0,0,0},{0,1,0,0}};
// Feedback Controller Globals
float mu = 0.0, rho = 0.0, alpha = 0.0, forwardSpeed = 0.0, rotationalSpeed = 0.0;
// Goals
float columnGoal = 0, rowGoal = 0, thetaGoal = 0, leftWheelPercent = 0, rightWheelPercent = 0;
// Step Two, see two steps ahead
int north2 = 0, west2 = 0, east2 = 0, south2 = 0, columnGoal2 = 0, rowGoal2 = 0, thetaGoal2 = 0;

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
    minimumCost = 250;

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
  // Records start time
   startTime = millis();
  // Reset values
   cmDetect = 0, north = 0, west = 0, east = 0, south = 0, north2 = 0, west2 = 0, east2 = 0, south2 = 0;

  // Checks if goal reached, if so, stop
  if(currentRow == 3 && currentColumn == 1)
  {
    sparki.RGB(RGB_OFF);
    sparki.moveStop();
    sparki.RGB(RGB_GREEN);
    while(true);
  }

   // Odometry - Keepin track of distance traveled
   if(fmod(x, 60) >= 30 || fmod(y, 60) >= 30)
   {
     rho   = sqrt(sq(x - (columnGoal2 * 60)) + sq(y - (rowGoal2 * 60)));
     alpha = theta - atan2((y - (rowGoal2 * 60)), (x - (columnGoal2 * 60)));
     mu    = thetaGoal2 - theta;
   }
   else
   {
     rho = sqrt(sq(x - (columnGoal * 60)) + sq(y - (rowGoal * 60)));
     alpha = theta - atan2((y - (rowGoal * 60)), (x - (columnGoal * 60)));
     mu    = thetaGoal - theta;
   }
   forwardSpeed    = P1 * rho;
   rotationalSpeed = P2 * alpha + P3 * mu;

   rightWheel = ((2 * forwardSpeed) + (rotationalSpeed * WHEEL_DISTANCES)) / (2 * WHEEL_RADIUS);
   leftWheel  = ((2 * forwardSpeed) - (rotationalSpeed * WHEEL_DISTANCES)) / (2 * WHEEL_RADIUS);


   if(rightWheel > WHEEL_VELOCITY/2) rightWheel = WHEEL_VELOCITY;
   if(leftWheel > WHEEL_VELOCITY/2) leftWheel = WHEEL_VELOCITY;

   leftWheelPercent = (100 * (leftWheel/WHEEL_VELOCITY));
   rightWheelPercent = (100 * (rightWheel/WHEEL_VELOCITY));

   sparki.motorRotate(MOTOR_LEFT, DIR_CCW, leftWheelPercent);
   sparki.motorRotate(MOTOR_RIGHT, DIR_CW, rightWheelPercent);


   // Determines X and Y using half of summed wheel speeds
   halfLeftRight = (rightWheel + leftWheel) / 2;
   x += cos(theta * (PI/180)) * halfLeftRight;
   y += sin(theta * (PI/180)) * halfLeftRight;
   theta = (((rightWheel * WHEEL_RADIUS) / WHEEL_DISTANCES) - ((leftWheel * WHEEL_RADIUS) / WHEEL_DISTANCES)) * (180/PI);
   //theta += rotationalSpeed * (180/PI);
   if(theta >= 360) theta = fmod(theta, 360);
   if(theta <= -360) theta = -fmod(abs(theta), 360);

  // Set current Column and Row values from X and Y, done for scaling
  currentColumn = x / 60;
  currentRow = y / 60;

  // Object Detection, changes world map if sees object within 20 cm COMMENTED OUT
  cmDetect = sparki.ping();
  // if(cmDetect != -1)
  // {
  //   if(cmDetect < 10)
  //   {
  //     // Northern Object
  //     if(currentColumn+1 <= 3 && (theta <= 5 && theta >= -5) || (theta <= 365 && theta >= 355) || (theta <= -355 && theta >= -365))
  //       bestPathMap[currentRow][currentColumn+1] = worldMap[currentRow][currentColumn+1] = 0;
  //     // East
  //     else if(currentRow + 1 <= 3 && (theta >= 85 && theta <= 95) || (theta >= -275 && theta <= -265))
  //       bestPathMap[currentRow+1][currentColumn] = worldMap[currentRow+1][currentColumn] = 0;
  //     // West
  //     else if(currentRow-1 >= 0 && (theta >= 265 && theta <= 275) || (theta >= -95 && theta <= -85))
  //       bestPathMap[currentRow-1][currentColumn] = worldMap[currentRow-1][currentColumn] = 0;
  //     // South
  //     else if(currentColumn-1 >= 0 && (theta >= 175 && theta <= 185) || (theta >= -185 && theta <= -175))
  //       bestPathMap[currentRow][currentColumn-1] = worldMap[currentRow][currentColumn-1] = 0;
  //   }
  // }

  // Prints onto Screen Everything needed.
  sparki.clearLCD(); // wipe the screen
  sparki.print("  Row: ");
  sparki.print(currentRow+1);
  sparki.print("  Column: ");
  sparki.print(currentColumn+1);
  sparki.print("\n  D:");
  sparki.print(theta);
  sparki.print("  X:");
  sparki.print(x/10);
  sparki.print("\n  Y:");
  sparki.print(y/10);
  sparki.print("  World Map\n");
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
  {
    sparki.print("    GOAL  REACHED");
    sparki.updateLCD();
    sparki.RGB(RGB_OFF);
    sparki.moveStop();
    sparki.RGB(RGB_GREEN);
    while(true);

  }
  sparki.updateLCD();

  // Checks boundary limits, if in bounds, adjust compass values, if not, set to 0 for blockage
  if(currentColumn+1 <= 3) north = bestPathMap[currentRow][currentColumn+1];
  else north = 0;
  if(currentRow + 1 <= 3) east = bestPathMap[currentRow+1][currentColumn];
  else east = 0;
  if(currentRow - 1 >= 0) west = bestPathMap[currentRow-1][currentColumn];
  else west = 0;
  if(currentColumn - 1 >= 0) south = bestPathMap[currentRow][currentColumn-1];
  else south = 0;

  // Now Does Traversal, based on compass values and theta for rotation
  // Proritizes Clockwise Motion
  if(north == 1)
  {
    thetaGoal = 0;
    columnGoal = currentColumn + 1;
    rowGoal = currentRow;

    if(currentColumn + 2 <= 3) north2 = bestPathMap[currentRow][currentColumn+2];
    else north2 = 0;
    if(currentRow + 1 <= 3) east2 = bestPathMap[currentRow+1][currentColumn+1];
    else east2 = 0;
    if(currentRow - 1 >= 0) west2 = bestPathMap[currentRow-1][currentColumn+1];
    else west2 = 0;
    if(currentColumn >= 0) south2 = bestPathMap[currentRow][currentColumn];
    else south2 = 0;

    // Now Does Traversal, based on compass values and theta for rotation
    // Proritizes Clockwise Motion
    if(north2 == 1)
    {
      thetaGoal2 = 0;
      columnGoal2 = currentColumn + 2;
      rowGoal2 = currentRow;
    }
    else if(east2 == 1)
    {
      thetaGoal2 = -90;
      columnGoal2 = currentColumn + 1;
      rowGoal2 = currentRow + 1;
    }
    else if(south2 == 1)
    {
      thetaGoal2 = 180;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow;
    }
    else if(west2 == 1)
    {
      thetaGoal2 = 90;
      columnGoal2 = currentColumn + 1;
      rowGoal2 = currentRow - 1;
    }
    else
    {
      thetaGoal2 = thetaGoal;
      columnGoal2 = columnGoal;
      rowGoal2 = rowGoal;
    }
  }
  else if(east == 1)
  {
    thetaGoal = -90;
    columnGoal = currentColumn;
    rowGoal = currentRow + 1;

    if(currentColumn + 1 <= 3) north2 = bestPathMap[currentRow+1][currentColumn+1];
    else north2 = 0;
    if(currentRow + 2 <= 3) east2 = bestPathMap[currentRow+2][currentColumn];
    else east2 = 0;
    if(currentRow >= 0) west2 = bestPathMap[currentRow][currentColumn];
    else west2 = 0;
    if(currentColumn - 1 >= 0) south2 = bestPathMap[currentRow+1][currentColumn-1];
    else south2 = 0;

    // Now Does Traversal, based on compass values and theta for rotation
    // Proritizes Clockwise Motion
    if(north2 == 1)
    {
      thetaGoal2 = 0;
      columnGoal2 = currentColumn + 1;
      rowGoal2 = currentRow +1;
    }
    else if(east2 == 1)
    {
      thetaGoal2 = -90;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow + 2;
    }
    else if(south2 == 1)
    {
      thetaGoal2 = 180;
      columnGoal2 = currentColumn - 1;
      rowGoal2 = currentRow + 1;
    }
    else if(west2 == 1)
    {
      thetaGoal2 = 90;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow;
    }
    else
    {
      thetaGoal2 = thetaGoal;
      columnGoal2 = columnGoal;
      rowGoal2 = rowGoal;
    }
  }
  else if(south == 1)
  {
    thetaGoal = 180;
    columnGoal = currentColumn - 1;
    rowGoal = currentRow;

    if(currentColumn <= 3) north2 = bestPathMap[currentRow][currentColumn];
    else north2 = 0;
    if(currentRow + 1 <= 3) east2 = bestPathMap[currentRow+1][currentColumn-1];
    else east2 = 0;
    if(currentRow - 1 >= 0) west2 = bestPathMap[currentRow-1][currentColumn-1];
    else west2 = 0;
    if(currentColumn - 2 >= 0) south2 = bestPathMap[currentRow][currentColumn-2];
    else south2 = 0;

    // Now Does Traversal, based on compass values and theta for rotation
    // Proritizes Clockwise Motion
    if(north2 == 1)
    {
      thetaGoal2 = 0;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow;
    }
    else if(east2 == 1)
    {
      thetaGoal2 = -90;
      columnGoal2 = currentColumn - 1;
      rowGoal2 = currentRow + 1;
    }
    else if(south2 == 1)
    {
      thetaGoal2 = 180;
      columnGoal2 = currentColumn - 2;
      rowGoal2 = currentRow;
    }
    else if(west2 == 1)
    {
      thetaGoal2 = 90;
      columnGoal2 = currentColumn - 1;
      rowGoal2 = currentRow - 1;
    }
    else
    {
      thetaGoal2 = thetaGoal;
      columnGoal2 = columnGoal;
      rowGoal2 = rowGoal;
    }
  }
  else if(west == 1)
  {
    thetaGoal = 90;
    columnGoal = currentColumn;
    rowGoal = currentRow - 1;

    if(currentColumn + 1 <= 3) north2 = bestPathMap[currentRow-1][currentColumn+1];
    else north2 = 0;
    if(currentRow <= 3) east2 = bestPathMap[currentRow][currentColumn];
    else east2 = 0;
    if(currentRow - 2 >= 0) west2 = bestPathMap[currentRow-2][currentColumn];
    else west2 = 0;
    if(currentColumn - 1 >= 0) south2 = bestPathMap[currentRow-1][currentColumn-1];
    else south2 = 0;

    // Now Does Traversal, based on compass values and theta for rotation
    // Proritizes Clockwise Motion
    if(north2 == 1)
    {
      thetaGoal2 = 0;
      columnGoal2 = currentColumn + 1;
      rowGoal2 = currentRow - 1;
    }
    else if(east2 == 1)
    {
      thetaGoal2 = -90;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow;
    }
    else if(south2 == 1)
    {
      thetaGoal2 = 180;
      columnGoal2 = currentColumn;
      rowGoal2 = currentRow - 1;
    }
    else if(west2 == 1)
    {
      thetaGoal2 = 90;
      columnGoal2 = currentColumn - 1;
      rowGoal2 = currentRow - 2;
    }
    else
    {
      thetaGoal2 = thetaGoal;
      columnGoal2 = columnGoal;
      rowGoal2 = rowGoal;
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
  if(totalTime > 0) delay(totalTime);
}

