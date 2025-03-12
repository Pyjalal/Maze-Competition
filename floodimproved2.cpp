#include <Arduino.h>
#include <EEPROM.h>

/***** Maze & Competition Parameters *****/
const int MAZE_SIZE = 10;              // 10 x 10 grid
const int START_X = 0;
const int START_Y = 0;
const int GOAL_X  = MAZE_SIZE - 1;
const int GOAL_Y  = MAZE_SIZE - 1;

// Wall flag bit masks for each cell
#define NORTH_WALL 0x1
#define EAST_WALL  0x2
#define SOUTH_WALL 0x4
#define WEST_WALL  0x8

// Direction definitions (no enum used)
#define NORTH 0
#define EAST  1
#define SOUTH 2
#define WEST  3

// Movement parameters
const int BASE_SPEED   = 150;         // PWM speed for normal moves
const int TURN_SPEED   = 120;         // PWM speed for turning
const int FAST_SPEED   = 200;         // PWM speed for fast-run mode
const int CELL_DISTANCE = 180;         // Distance of one cell (mm) [adjust as needed]
const unsigned long BASE_MOVE_TIME = 600; // ms to traverse one cell at BASE_SPEED

// Ultrasonic sensor pin assignments (example)
const int TRIG_FRONT = 2;
const int ECHO_FRONT = 3;
const int TRIG_LEFT  = 4;
const int ECHO_LEFT  = 5;
const int TRIG_RIGHT = 6;
const int ECHO_RIGHT = 7;

// Motor driver pin assignments (example)
const int LEFT_MOTOR_FWD  = 8;
const int LEFT_MOTOR_REV  = 9;
const int RIGHT_MOTOR_FWD = 10;
const int RIGHT_MOTOR_REV = 11;
const int LEFT_MOTOR_PWM  = 5;
const int RIGHT_MOTOR_PWM = 6;

// Sensor thresholds and timeouts
const int WALL_DISTANCE_CM = 15;       // if distance <= 15cm, consider a wall detected
const unsigned long PING_TIMEOUT = 30000UL; // us timeout for ultrasonic sensor

/***** Maze Mapping Data Structures *****/
uint8_t mazeWalls[MAZE_SIZE][MAZE_SIZE];   // For each cell, record walls (using bitwise OR of wall flags)
uint8_t distanceGrid[MAZE_SIZE][MAZE_SIZE];  // Flood-fill distance values

/***** Robot State Variables *****/
int curX = START_X;
int curY = START_Y;
int curDir = NORTH;              // starting orientation (facing NORTH)
bool exploring = true;           // true: exploration phase; false: fast-run phase

// Variables for non-blocking movement control
bool moving = false;
unsigned long moveStartTime = 0;
unsigned long moveDuration = 0;

/***** Fast-run Path Storage *****/
#define MAX_PATH_LEN 256
int shortestPath[MAX_PATH_LEN];  // Array to hold the sequence of moves (each as NORTH, EAST, SOUTH, or WEST)
int pathLength = 0;
int pathIndex = 0;

/***** Function Prototypes *****/
void initializeMaze();
long readUltrasonic(int trigPin, int echoPin);
void scanWalls(bool &wallLeft, bool &wallFront, bool &wallRight);
void updateWallMap(bool wallLeft, bool wallFront, bool wallRight);
void floodFillCompute();
int chooseNextMove();
void setMotorSpeedDirection(int moveDir, int speed);
void turnToDirection(int newDir);
void stopMotors();
void computeShortestPath();
void saveFastRunPathToEEPROM();
void startFastRun();

/***** Setup Function *****/
void setup() {
  Serial.begin(115200);

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize motor driver pins
  pinMode(LEFT_MOTOR_FWD, OUTPUT);
  pinMode(LEFT_MOTOR_REV, OUTPUT);
  pinMode(RIGHT_MOTOR_FWD, OUTPUT);
  pinMode(RIGHT_MOTOR_REV, OUTPUT);
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  
  // Initialize maze mapping structures
  initializeMaze();
  
  Serial.println("== Maze Solver Initialized ==");
  Serial.print("Start at ("); Serial.print(curX); Serial.print(","); Serial.print(curY); 
  Serial.println("), facing NORTH.");
}

/***** Main Loop Function *****/
void loop() {
  // ---- Exploration Phase ----
  if (exploring) {
    if (!moving) {
      // If reached the goal, finish exploration and prepare fast-run
      if (curX == GOAL_X && curY == GOAL_Y) {
        Serial.println("Goal reached! Exploration complete.");
        exploring = false;
        computeShortestPath();
        saveFastRunPathToEEPROM(); // Save the computed fast-run path to EEPROM
        startFastRun();
        return;
      }
      
      // At a cell: scan walls and update maze mapping
      bool wallL, wallF, wallR;
      scanWalls(wallL, wallF, wallR);
      updateWallMap(wallL, wallF, wallR);
      
      Serial.print("At cell ("); Serial.print(curX); Serial.print(","); Serial.print(curY); Serial.print(") ");
      Serial.print("Walls -> Left: "); Serial.print(wallL);
      Serial.print(", Front: "); Serial.print(wallF);
      Serial.print(", Right: "); Serial.println(wallR);
      
      // Recompute flood-fill distances based on new wall information
      floodFillCompute();
      
      // Decide next move based on the flood-fill grid
      int nextDir = chooseNextMove();
      Serial.print("Next move direction: ");
      if (nextDir == NORTH) Serial.println("NORTH");
      else if (nextDir == EAST) Serial.println("EAST");
      else if (nextDir == SOUTH) Serial.println("SOUTH");
      else if (nextDir == WEST) Serial.println("WEST");
      
      // Rotate to face the desired direction if necessary
      if (nextDir != curDir) {
        turnToDirection(nextDir);
        curDir = nextDir;
        delay(100);  // short pause to ensure turn completes
      }
      
      // Begin moving forward one cell at BASE_SPEED
      setMotorSpeedDirection(curDir, BASE_SPEED);
      moving = true;
      moveStartTime = millis();
      moveDuration = BASE_MOVE_TIME;
    } 
    else {
      // Check if movement for one cell is complete
      unsigned long elapsed = millis() - moveStartTime;
      if (elapsed >= moveDuration) {
        stopMotors();
        moving = false;
        // Update current cell coordinates based on current direction
        if (curDir == NORTH) curY += 1;
        else if (curDir == EAST) curX += 1;
        else if (curDir == SOUTH) curY -= 1;
        else if (curDir == WEST) curX -= 1;
        Serial.print("Arrived at cell ("); Serial.print(curX); Serial.print(","); Serial.print(curY); Serial.println(").");
      }
    }
  }
  // ---- Fast-Run Phase ----
  else {
    if (!moving) {
      if (pathIndex >= pathLength) {
        stopMotors();
        Serial.println("Fast-run complete! Robot has reached the goal via the shortest path.");
        while (true) {
          // End of run - halt.
        }
      }
      int nextDir = shortestPath[pathIndex++];
      Serial.print("Fast-run step "); Serial.print(pathIndex); Serial.print("/"); Serial.print(pathLength);
      Serial.print(" -> ");
      if (nextDir == NORTH) Serial.println("NORTH");
      else if (nextDir == EAST) Serial.println("EAST");
      else if (nextDir == SOUTH) Serial.println("SOUTH");
      else if (nextDir == WEST) Serial.println("WEST");
      
      // Adjust speed: if the next move is a turn, reduce speed
      int speed = FAST_SPEED;
      if (pathIndex < pathLength) {
        int upcomingDir = shortestPath[pathIndex];
        if (upcomingDir != nextDir) {
          speed = BASE_SPEED;
          Serial.println("Approaching turn, reducing speed.");
        } else {
          speed = FAST_SPEED;
        }
      }
      
      // If not already facing the correct direction, turn
      if (nextDir != curDir) {
        turnToDirection(nextDir);
        curDir = nextDir;
        delay(50);
      }
      // Move one cell forward at the chosen speed
      setMotorSpeedDirection(curDir, speed);
      moving = true;
      moveStartTime = millis();
      unsigned long fastTime = BASE_MOVE_TIME;
      if (speed > BASE_SPEED) {
        fastTime = (unsigned long)((float)BASE_MOVE_TIME * BASE_SPEED / speed);
      }
      moveDuration = fastTime;
    } 
    else {
      // Check if the current fast-run movement is complete
      unsigned long elapsed = millis() - moveStartTime;
      if (elapsed >= moveDuration) {
        stopMotors();
        moving = false;
        if (curDir == NORTH) curY += 1;
        else if (curDir == EAST) curX += 1;
        else if (curDir == SOUTH) curY -= 1;
        else if (curDir == WEST) curX -= 1;
        Serial.print("Fast-run: Arrived at cell ("); Serial.print(curX); Serial.print(","); Serial.print(curY); Serial.println(").");
      }
    }
  }
}

/***** Helper Functions *****/

// Initialize the maze mapping: reset walls and flood-fill distances.
void initializeMaze() {
  for (int i = 0; i < MAZE_SIZE; ++i) {
    for (int j = 0; j < MAZE_SIZE; ++j) {
      mazeWalls[i][j] = 0;       
      distanceGrid[i][j] = 0;
    }
  }
  // Mark the outer boundaries as walls to keep the robot within the maze.
  for (int i = 0; i < MAZE_SIZE; ++i) {
    mazeWalls[i][0] |= SOUTH_WALL;             // bottom row
    mazeWalls[i][MAZE_SIZE - 1] |= NORTH_WALL;   // top row
    mazeWalls[0][i] |= WEST_WALL;                // left column
    mazeWalls[MAZE_SIZE - 1][i] |= EAST_WALL;     // right column
  }
  floodFillCompute();
}

// Trigger an ultrasonic sensor and return the measured distance (cm).
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  unsigned long duration = pulseIn(echoPin, HIGH, PING_TIMEOUT);
  if (duration == 0) return 999;  // No echo received
  long distanceCm = duration / 58;  // Convert microseconds to centimeters
  return distanceCm;
}

// Read all three ultrasonic sensors to detect walls on left, front, and right.
void scanWalls(bool &wallLeft, bool &wallFront, bool &wallRight) {
  long distFront = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long distLeft  = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long distRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
  wallFront = (distFront <= WALL_DISTANCE_CM);
  wallLeft  = (distLeft  <= WALL_DISTANCE_CM);
  wallRight = (distRight <= WALL_DISTANCE_CM);
  
  Serial.print("Sensor distances (cm) -> Front: "); Serial.print(distFront);
  Serial.print(", Left: "); Serial.print(distLeft);
  Serial.print(", Right: "); Serial.println(distRight);
}

// Update the maze wall map based on sensor readings and current orientation.
void updateWallMap(bool wallLeft, bool wallFront, bool wallRight) {
  if (curDir == NORTH) {
    if (wallFront) {
      mazeWalls[curX][curY] |= NORTH_WALL;
      if (curY < MAZE_SIZE - 1) mazeWalls[curX][curY + 1] |= SOUTH_WALL;
    }
    if (wallLeft) {
      mazeWalls[curX][curY] |= WEST_WALL;
      if (curX > 0) mazeWalls[curX - 1][curY] |= EAST_WALL;
    }
    if (wallRight) {
      mazeWalls[curX][curY] |= EAST_WALL;
      if (curX < MAZE_SIZE - 1) mazeWalls[curX + 1][curY] |= WEST_WALL;
    }
  } else if (curDir == EAST) {
    if (wallFront) {
      mazeWalls[curX][curY] |= EAST_WALL;
      if (curX < MAZE_SIZE - 1) mazeWalls[curX + 1][curY] |= WEST_WALL;
    }
    if (wallLeft) {
      mazeWalls[curX][curY] |= NORTH_WALL;
      if (curY < MAZE_SIZE - 1) mazeWalls[curX][curY + 1] |= SOUTH_WALL;
    }
    if (wallRight) {
      mazeWalls[curX][curY] |= SOUTH_WALL;
      if (curY > 0) mazeWalls[curX][curY - 1] |= NORTH_WALL;
    }
  } else if (curDir == SOUTH) {
    if (wallFront) {
      mazeWalls[curX][curY] |= SOUTH_WALL;
      if (curY > 0) mazeWalls[curX][curY - 1] |= NORTH_WALL;
    }
    if (wallLeft) {
      mazeWalls[curX][curY] |= EAST_WALL;
      if (curX < MAZE_SIZE - 1) mazeWalls[curX + 1][curY] |= WEST_WALL;
    }
    if (wallRight) {
      mazeWalls[curX][curY] |= WEST_WALL;
      if (curX > 0) mazeWalls[curX - 1][curY] |= EAST_WALL;
    }
  } else if (curDir == WEST) {
    if (wallFront) {
      mazeWalls[curX][curY] |= WEST_WALL;
      if (curX > 0) mazeWalls[curX - 1][curY] |= EAST_WALL;
    }
    if (wallLeft) {
      mazeWalls[curX][curY] |= SOUTH_WALL;
      if (curY > 0) mazeWalls[curX][curY - 1] |= NORTH_WALL;
    }
    if (wallRight) {
      mazeWalls[curX][curY] |= NORTH_WALL;
      if (curY < MAZE_SIZE - 1) mazeWalls[curX][curY + 1] |= SOUTH_WALL;
    }
  }
}

// Perform a flood-fill from the goal cell to compute the distanceGrid.
void floodFillCompute() {
  const uint8_t INF = 255;
  for (int x = 0; x < MAZE_SIZE; ++x) {
    for (int y = 0; y < MAZE_SIZE; ++y) {
      distanceGrid[x][y] = INF;
    }
  }
  
  int queueX[MAZE_SIZE * MAZE_SIZE];
  int queueY[MAZE_SIZE * MAZE_SIZE];
  int qHead = 0, qTail = 0;
  
  distanceGrid[GOAL_X][GOAL_Y] = 0;
  queueX[qTail] = GOAL_X;
  queueY[qTail] = GOAL_Y;
  qTail++;
  
  while (qHead < qTail) {
    int cx = queueX[qHead];
    int cy = queueY[qHead];
    qHead++;
    uint8_t baseDist = distanceGrid[cx][cy];
    
    // Check North neighbor
    if (cy < MAZE_SIZE - 1) {
      bool wallHere = (mazeWalls[cx][cy] & NORTH_WALL);
      if (!wallHere && distanceGrid[cx][cy + 1] == INF) {
        distanceGrid[cx][cy + 1] = baseDist + 1;
        queueX[qTail] = cx;
        queueY[qTail] = cy + 1;
        qTail++;
      }
    }
    // Check East neighbor
    if (cx < MAZE_SIZE - 1) {
      bool wallHere = (mazeWalls[cx][cy] & EAST_WALL);
      if (!wallHere && distanceGrid[cx + 1][cy] == INF) {
        distanceGrid[cx + 1][cy] = baseDist + 1;
        queueX[qTail] = cx + 1;
        queueY[qTail] = cy;
        qTail++;
      }
    }
    // Check South neighbor
    if (cy > 0) {
      bool wallHere = (mazeWalls[cx][cy] & SOUTH_WALL);
      if (!wallHere && distanceGrid[cx][cy - 1] == INF) {
        distanceGrid[cx][cy - 1] = baseDist + 1;
        queueX[qTail] = cx;
        queueY[qTail] = cy - 1;
        qTail++;
      }
    }
    // Check West neighbor
    if (cx > 0) {
      bool wallHere = (mazeWalls[cx][cy] & WEST_WALL);
      if (!wallHere && distanceGrid[cx - 1][cy] == INF) {
        distanceGrid[cx - 1][cy] = baseDist + 1;
        queueX[qTail] = cx - 1;
        queueY[qTail] = cy;
        qTail++;
      }
    }
  }
  
  Serial.println("Distance grid (partial):");
  for (int yy = MAZE_SIZE - 1; yy >= MAZE_SIZE - 4; --yy) {
    for (int xx = 0; xx < 5; ++xx) {
      Serial.print(distanceGrid[xx][yy]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

// Decide the next move direction from the current cell based on the flood-fill grid.
int chooseNextMove() {
  uint8_t currentDist = distanceGrid[curX][curY];
  uint8_t minDist = 255;
  int bestDir = NORTH;
  
  // Check North neighbor
  if (!(mazeWalls[curX][curY] & NORTH_WALL) && curY < MAZE_SIZE - 1) {
    uint8_t d = distanceGrid[curX][curY + 1];
    if (d < minDist) { minDist = d; bestDir = NORTH; }
  }
  // Check East neighbor
  if (!(mazeWalls[curX][curY] & EAST_WALL) && curX < MAZE_SIZE - 1) {
    uint8_t d = distanceGrid[curX + 1][curY];
    if (d < minDist) { minDist = d; bestDir = EAST; }
  }
  // Check South neighbor
  if (!(mazeWalls[curX][curY] & SOUTH_WALL) && curY > 0) {
    uint8_t d = distanceGrid[curX][curY - 1];
    if (d < minDist) { minDist = d; bestDir = SOUTH; }
  }
  // Check West neighbor
  if (!(mazeWalls[curX][curY] & WEST_WALL) && curX > 0) {
    uint8_t d = distanceGrid[curX - 1][curY];
    if (d < minDist) { minDist = d; bestDir = WEST; }
  }
  return bestDir;
}

// Set motor speeds to drive the robot forward.
void setMotorSpeedDirection(int moveDir, int speed) {
  if (speed < 0) speed = 0;
  if (speed > 255) speed = 255;
  stopMotors();
  // For forward movement, both motors run forward.
  digitalWrite(LEFT_MOTOR_FWD, HIGH);
  digitalWrite(LEFT_MOTOR_REV, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, HIGH);
  digitalWrite(RIGHT_MOTOR_REV, LOW);
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}

// Turn the robot in place to face a new direction.
void turnToDirection(int newDir) {
  int turnSteps = newDir - curDir;
  if (turnSteps < 0) turnSteps += 4;
  if (turnSteps == 0) return;
  if (turnSteps == 2) {
    Serial.println("Turning 180 degrees.");
    digitalWrite(LEFT_MOTOR_FWD, HIGH);
    digitalWrite(LEFT_MOTOR_REV, LOW);
    digitalWrite(RIGHT_MOTOR_FWD, LOW);
    digitalWrite(RIGHT_MOTOR_REV, HIGH);
    analogWrite(LEFT_MOTOR_PWM, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_PWM, TURN_SPEED);
    delay(300);
    stopMotors();
    delay(100);
    digitalWrite(LEFT_MOTOR_FWD, HIGH);
    digitalWrite(LEFT_MOTOR_REV, LOW);
    digitalWrite(RIGHT_MOTOR_FWD, LOW);
    digitalWrite(RIGHT_MOTOR_REV, HIGH);
    analogWrite(LEFT_MOTOR_PWM, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_PWM, TURN_SPEED);
    delay(300);
    stopMotors();
    curDir = newDir;
    return;
  }
  if (turnSteps == 1) {
    Serial.println("Turning right 90 degrees.");
    digitalWrite(LEFT_MOTOR_FWD, HIGH);
    digitalWrite(LEFT_MOTOR_REV, LOW);
    digitalWrite(RIGHT_MOTOR_FWD, LOW);
    digitalWrite(RIGHT_MOTOR_REV, HIGH);
    analogWrite(LEFT_MOTOR_PWM, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_PWM, TURN_SPEED);
    delay(300);
    stopMotors();
    curDir = newDir;
    return;
  }
  if (turnSteps == 3) {
    Serial.println("Turning left 90 degrees.");
    digitalWrite(LEFT_MOTOR_FWD, LOW);
    digitalWrite(LEFT_MOTOR_REV, HIGH);
    digitalWrite(RIGHT_MOTOR_FWD, HIGH);
    digitalWrite(RIGHT_MOTOR_REV, LOW);
    analogWrite(LEFT_MOTOR_PWM, TURN_SPEED);
    analogWrite(RIGHT_MOTOR_PWM, TURN_SPEED);
    delay(300);
    stopMotors();
    curDir = newDir;
    return;
  }
}

// Stop both motors.
void stopMotors() {
  analogWrite(LEFT_MOTOR_PWM, 0);
  analogWrite(RIGHT_MOTOR_PWM, 0);
  digitalWrite(LEFT_MOTOR_FWD, LOW);
  digitalWrite(LEFT_MOTOR_REV, LOW);
  digitalWrite(RIGHT_MOTOR_FWD, LOW);
  digitalWrite(RIGHT_MOTOR_REV, LOW);
}

// Compute the shortest path from start to goal using the flood-fill distance grid.
void computeShortestPath() {
  int x = START_X;
  int y = START_Y;
  pathLength = 0;
  
  if (distanceGrid[x][y] == 255) {
    Serial.println("No path found to goal!");
    return;
  }
  Serial.println("Computing shortest path from start to goal...");
  while (!(x == GOAL_X && y == GOAL_Y) && pathLength < MAX_PATH_LEN) {
    uint8_t currentDist = distanceGrid[x][y];
    if (y < MAZE_SIZE - 1 && !(mazeWalls[x][y] & NORTH_WALL) && distanceGrid[x][y + 1] == currentDist - 1) {
      shortestPath[pathLength++] = NORTH;
      y += 1;
      continue;
    }
    if (x < MAZE_SIZE - 1 && !(mazeWalls[x][y] & EAST_WALL) && distanceGrid[x + 1][y] == currentDist - 1) {
      shortestPath[pathLength++] = EAST;
      x += 1;
      continue;
    }
    if (y > 0 && !(mazeWalls[x][y] & SOUTH_WALL) && distanceGrid[x][y - 1] == currentDist - 1) {
      shortestPath[pathLength++] = SOUTH;
      y -= 1;
      continue;
    }
    if (x > 0 && !(mazeWalls[x][y] & WEST_WALL) && distanceGrid[x - 1][y] == currentDist - 1) {
      shortestPath[pathLength++] = WEST;
      x -= 1;
      continue;
    }
    Serial.println("Warning: shortest path reconstruction issue.");
    break;
  }
  Serial.print("Shortest path moves (");
  Serial.print(pathLength);
  Serial.println(" steps):");
  for (int i = 0; i < pathLength; i++) {
    if (shortestPath[i] == NORTH) Serial.print("N ");
    else if (shortestPath[i] == EAST) Serial.print("E ");
    else if (shortestPath[i] == SOUTH) Serial.print("S ");
    else if (shortestPath[i] == WEST) Serial.print("W ");
  }
  Serial.println();
}

// Save the computed fast-run path to EEPROM.
void saveFastRunPathToEEPROM() {
  int address = 0;
  EEPROM.put(address, pathLength);
  address += sizeof(pathLength);
  for (int i = 0; i < pathLength; i++) {
    EEPROM.put(address, shortestPath[i]);
    address += sizeof(shortestPath[i]);
  }
  Serial.println("Fast-run path saved to EEPROM.");
}

// Prepare for the fast-run phase.
void startFastRun() {
  Serial.println("Starting fast-run mode: executing shortest path.");
  curX = START_X;
  curY = START_Y;
  curDir = NORTH;
  moving = false;
  pathIndex = 0;
}
