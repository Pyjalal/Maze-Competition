#include <Wire.h>          // I2C library (for MPU6050)
#include <MPU6050.h>       // MPU6050 library
#include <EEPROM.h>        // EEPROM library
#include <Arduino.h>
#include <Ultrasonic.h>
#include <Encoder.h>

// ======================
// Hardware Pin Assignments
// ======================
// Rotary Encoder pins
const int ENC_PIN_A = A1;
const int ENC_PIN_B = A0;

// Motor driver pins (assuming dual H-bridge for two motors)
const int ENA = 10;   // Left motor PWM
const int IN1 = 5;    // Left motor direction
const int IN2 = 4;
const int ENB = 9;    // Right motor PWM
const int IN3 = 3;    // Right motor direction
const int IN4 = 2;

// Ultrasonic sensor pins (3 sensors: front, left, right)
const int TRIG_FRONT = 6;
const int ECHO_FRONT = 7;
const int TRIG_LEFT  = 8;
const int ECHO_LEFT  = 11;
const int TRIG_RIGHT = 12;
const int ECHO_RIGHT = 13;

// ======================
// Maze & Competition Parameters (from Rule Book [])
// ======================
const int MAZE_SIZE = 10;   // 10 x 10 grid
const int CELL_SIZE_CM = 25;  // Each cell is 25cm x 25cm

// Define starting and goal positions.
const int START_X = 0;
const int START_Y = 0;
const int GOAL_X  = 9;
const int GOAL_Y  = 9;

// Maximum distance value for flood-fill propagation
const int MAX_DISTANCE = 255;
// Threshold (in cm) to consider a wall detected by an ultrasonic sensor
const int WALL_DISTANCE_CM = 15;  // Adjust as needed

// Encoder threshold for one cell travel (calibrate for 25cm per cell)
int cellDistanceTicks = 200; // Example value; adjust based on your setup

// ======================
// Global Variables for Maze Navigation
// ======================
int currentDirection = 0;  // Orientation: 0 = North, 1 = East, 2 = South, 3 = West
int posX = START_X, posY = START_Y;  // Robot's current cell position

// Maze mapping arrays
uint8_t distanceGrid[MAZE_SIZE][MAZE_SIZE];   // Flood-fill distances
uint8_t wallsGrid[MAZE_SIZE][MAZE_SIZE];       // Wall information bitfield per cell
// Wall bits: bit0 = North, bit1 = East, bit2 = South, bit3 = West

// ======================
// MPU-6050 Sensor
// ======================
MPU6050 mpu;
int16_t gz;
float gyroZoffset = 0; // gyroZoffset = GyroErrorZ
float yaw = 0;

void mpuSetup(){
  Wire.begin();
  mpu.initialize();
  if(mpu.testConnection()){
    Serial.println("MPU6050 connected");
  } else {
    Serial.println("MPU6050 connection failed");
    while(1);
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println("Setting full scale range");
}

// Get Z-axis rotation (yaw) from MPU6050
void getOrientation() {
  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ);
  gz = gyroZ;
  gz -= gyroZoffset;
}

// calibration() = calculateError()
void calibration(){
  long sum = 0;
  for (int i = 0; i < 200; i++) {
    getOrientation();
    sum += gz;
    delay(5);
  }
  gyroZoffset = sum / 200.0;
  Serial.print("Gyro Z offset: "); Serial.println(gyroZoffset);
}

void update() {
  static unsigned long previousTime = millis();
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - previousTime) / 1000.0; // seconds
  previousTime = currentTime;

  getOrientation();

  // Ignore small noise
  if (abs(gz) < 0.05) {
    gz = 0;
  }
  
  yaw += gz * elapsedTime; // integrate gyro data
  Serial.print("gz: ");
  Serial.println(gz);
  Serial.print("Yaw: ");
  Serial.println(yaw);
}

// ======================
// Ultrasonic Sensor Setup
// ======================
Ultrasonic frontUS(TRIG_FRONT, ECHO_FRONT);
Ultrasonic leftUS(TRIG_LEFT, ECHO_LEFT);
Ultrasonic rightUS(TRIG_RIGHT, ECHO_RIGHT);

// Return distance in cm
unsigned int dFront(){
  digitalWrite(TRIG_FRONT, LOW);
  delay(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delay(10);
  digitalWrite(TRIG_FRONT, LOW);
  return (pulseIn(ECHO_FRONT, HIGH) * 0.034613 / 2.00);
}

float dLeft(){
  digitalWrite(TRIG_LEFT, LOW);
  delay(2);
  digitalWrite(TRIG_LEFT, HIGH);
  delay(10);
  digitalWrite(TRIG_LEFT, LOW);
  return (pulseIn(ECHO_LEFT, HIGH) * 0.034613 / 2.00);
}

float dRight(){
  digitalWrite(TRIG_RIGHT, LOW);
  delay(2);
  digitalWrite(TRIG_RIGHT, HIGH);
  delay(10);
  digitalWrite(TRIG_RIGHT, LOW);
  return (pulseIn(ECHO_RIGHT, HIGH) * 0.034613 / 2.00);
}

// ======================
// EEPROM Addresses
// ======================
const int EEPROM_FLAG_ADDR = 0;              // Flag address to indicate stored maze data
const int EEPROM_MAZE_START_ADDR = 1;          // Maze data start address

// ======================
// Motor Speed Variables
// ======================
const int baseSpeed = 100;
const int fastSpeed = 200;

// ======================
// Rotary Encoder
// ======================
Encoder enc(ENC_PIN_A, ENC_PIN_B);

// ======================
// Debug Function: debugPrintMaze()
// This function prints the maze mapping stored in the wallsGrid array.
// Each cell is printed as a 4-bit binary number (order: N, E, S, W)
void debugPrintMaze() {
  Serial.println("Maze bit array (N, E, S, W):");
  // Loop from top row to bottom row for a more natural view.
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      char buf[5];
      // Bits: bit0 = North, bit1 = East, bit2 = South, bit3 = West.
      for (int b = 0; b < 4; b++) {
        buf[b] = (wallsGrid[y][x] & (1 << b)) ? '1' : '0';
      }
      buf[4] = '\0';
      Serial.print(buf);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// ======================
// Function: scanWalls()
// Reads the ultrasonic sensors and updates wall information for the current cell.
void scanWalls() {
  unsigned int dF = frontUS.read(CM);
  unsigned int dL = leftUS.read(CM);
  unsigned int dR = rightUS.read(CM);
  // For the rear, assume no wall (simulate with a large distance)
  unsigned int dBack = 100;
  
  uint8_t wallBits = 0;
  // Map sensor readings to walls based on current orientation:
  switch(currentDirection) {
    case 0: // Facing North
      if(dF < WALL_DISTANCE_CM) wallBits |= 0x01;  // North wall
      if(dR < WALL_DISTANCE_CM) wallBits |= 0x02;  // East wall
      if(dBack < WALL_DISTANCE_CM) wallBits |= 0x04; // South wall (assumed)
      if(dL < WALL_DISTANCE_CM) wallBits |= 0x08;  // West wall
      Serial.println("Scanning: Facing North");
      break;
    case 1: // Facing East
      if(dF < WALL_DISTANCE_CM) wallBits |= 0x02;  // East wall
      if(dR < WALL_DISTANCE_CM) wallBits |= 0x04;  // South wall
      if(dBack < WALL_DISTANCE_CM) wallBits |= 0x08; // West wall
      if(dL < WALL_DISTANCE_CM) wallBits |= 0x01;  // North wall
      Serial.println("Scanning: Facing East");
      break;
    case 2: // Facing South
      if(dF < WALL_DISTANCE_CM) wallBits |= 0x04;  // South wall
      if(dR < WALL_DISTANCE_CM) wallBits |= 0x08;  // West wall
      if(dBack < WALL_DISTANCE_CM) wallBits |= 0x01; // North wall
      if(dL < WALL_DISTANCE_CM) wallBits |= 0x02;  // East wall
      Serial.println("Scanning: Facing South");
      break;
    case 3: // Facing West
      if(dF < WALL_DISTANCE_CM) wallBits |= 0x08;  // West wall
      if(dR < WALL_DISTANCE_CM) wallBits |= 0x01;  // North wall
      if(dBack < WALL_DISTANCE_CM) wallBits |= 0x02; // East wall
      if(dL < WALL_DISTANCE_CM) wallBits |= 0x04;  // South wall
      Serial.println("Scanning: Facing West");
      break;
  }
  // Merge new wall data with any previously recorded info
  wallsGrid[posY][posX] |= wallBits;
  // Update neighboring cells reciprocally:
  if((wallBits & 0x01) && posY < MAZE_SIZE-1) wallsGrid[posY+1][posX] |= 0x04;
  if((wallBits & 0x02) && posX < MAZE_SIZE-1) wallsGrid[posY][posX+1] |= 0x08;
  if((wallBits & 0x04) && posY > 0)          wallsGrid[posY-1][posX] |= 0x01;
  if((wallBits & 0x08) && posX > 0)          wallsGrid[posY][posX-1] |= 0x02;
}

// ======================
// Function: computeDistances()
// Implements the flood-fill algorithm to update the distance grid from the goal.
void computeDistances(int targetX, int targetY) {
  // Initialize all cells to MAX_DISTANCE
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      distanceGrid[y][x] = MAX_DISTANCE;
    }
  }
  distanceGrid[targetY][targetX] = 0;
  bool updated = true;
  while (updated) {
    updated = false;
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int x = 0; x < MAZE_SIZE; x++) {
        if (x == targetX && y == targetY) continue;
        uint8_t minNeighbor = MAX_DISTANCE;
        // Check neighbors (if no wall between current cell and neighbor)
        // North neighbor:
        if (!(wallsGrid[y][x] & 0x01) && y < MAZE_SIZE - 1) {
          if (distanceGrid[y+1][x] < minNeighbor) minNeighbor = distanceGrid[y+1][x];
        }
        // East neighbor:
        if (!(wallsGrid[y][x] & 0x02) && x < MAZE_SIZE - 1) {
          if (distanceGrid[y][x+1] < minNeighbor) minNeighbor = distanceGrid[y][x+1];
        }
        // South neighbor:
        if (!(wallsGrid[y][x] & 0x04) && y > 0) {
          if (distanceGrid[y-1][x] < minNeighbor) minNeighbor = distanceGrid[y-1][x];
        }
        // West neighbor:
        if (!(wallsGrid[y][x] & 0x08) && x > 0) {
          if (distanceGrid[y][x-1] < minNeighbor) minNeighbor = distanceGrid[y][x-1];
        }
        if (minNeighbor != MAX_DISTANCE && distanceGrid[y][x] != minNeighbor + 1) {
          distanceGrid[y][x] = minNeighbor + 1;
          updated = true;
        }
      }
    }
  }
}

// ======================
// Motor Control Functions
// ======================
void driveStop() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void driveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void driveTurnLeft(int speed) {
  // In-place left turn: left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

void driveTurnRight(int speed) {
  // In-place right turn: left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

// ======================
// Function: turnByDegree(float degree)
// Uses the MPU6050 to perform a precise turn. Positive degree for right, negative for left.
void turnByDegree(float degree) {
  float target = fabs(degree);
  if (degree > 0) {
    driveTurnRight(baseSpeed);
  } else if (degree < 0) {
    driveTurnLeft(baseSpeed);
  } else return;

  unsigned long startTime = millis();
  float accumulated = 0.0;
  while (accumulated < target) {
    update();
    accumulated = fabs(yaw);
    if (millis() - startTime > 5000) {
      Serial.println("Turn timed out!");
      break;
    }
  }
  driveStop();
  yaw = 0; // reset yaw after the turn
  // Update current orientation based on turn direction
  if (degree > 0)
    currentDirection = (currentDirection + 1) % 4;
  else if (degree < 0)
    currentDirection = (currentDirection + 3) % 4;
}

// ======================
// Function: moveForwardOneCell()
// Drives forward exactly one cell (25 cm) using encoder feedback.
void moveForwardOneCell(int speed) {
  enc.write(0);
  driveForward(speed);
  while (abs(enc.read()) < cellDistanceTicks) {
    // Optionally add gyro corrections here if needed.
  }
  driveStop();
  // Update cell position based on current orientation:
  if (currentDirection == 0) posY += 1;       // North
  else if (currentDirection == 1) posX += 1;    // East
  else if (currentDirection == 2) posY -= 1;    // South
  else if (currentDirection == 3) posX -= 1;    // West
}

// ======================
// Function: decideAndMove()
// Chooses the next move using the flood-fill distances,
// preferring forward motion over turns when distances tie.
void decideAndMove(bool fastRun = false) {
  uint8_t currDist = distanceGrid[posY][posX];
  uint8_t distForward = MAX_DISTANCE, distLeft = MAX_DISTANCE;
  uint8_t distRight = MAX_DISTANCE, distBack = MAX_DISTANCE;
  
  // Check each neighbor based on currentDirection
  if (!(wallsGrid[posY][posX] & 0x01) && posY < MAZE_SIZE - 1) { // North neighbor
    uint8_t d = distanceGrid[posY+1][posX];
    switch(currentDirection) {
      case 0: distForward = d; break;
      case 1: distLeft    = d; break;
      case 2: distBack    = d; break;
      case 3: distRight   = d; break;
    }
  }
  if (!(wallsGrid[posY][posX] & 0x02) && posX < MAZE_SIZE - 1) { // East neighbor
    uint8_t d = distanceGrid[posY][posX+1];
    switch(currentDirection) {
      case 0: distRight   = d; break;
      case 1: distForward = d; break;
      case 2: distLeft    = d; break;
      case 3: distBack    = d; break;
    }
  }
  if (!(wallsGrid[posY][posX] & 0x04) && posY > 0) { // South neighbor
    uint8_t d = distanceGrid[posY-1][posX];
    switch(currentDirection) {
      case 0: distBack    = d; break;
      case 1: distRight   = d; break;
      case 2: distForward = d; break;
      case 3: distLeft    = d; break;
    }
  }
  if (!(wallsGrid[posY][posX] & 0x08) && posX > 0) { // West neighbor
    uint8_t d = distanceGrid[posY][posX-1];
    switch(currentDirection) {
      case 0: distLeft    = d; break;
      case 1: distBack    = d; break;
      case 2: distRight   = d; break;
      case 3: distForward = d; break;
    }
  }
  
  // Determine the minimum distance among available moves
  uint8_t minDist = currDist;
  if (distForward < minDist) minDist = distForward;
  if (distLeft < minDist)    minDist = distLeft;
  if (distRight < minDist)   minDist = distRight;
  if (distBack < minDist)    minDist = distBack;
  
  // Prefer forward > left > right > back
  if (distForward == minDist) {
    moveForwardOneCell(fastRun ? fastSpeed : baseSpeed);
  } else if (distLeft == minDist) {
    turnByDegree(-90);
    moveForwardOneCell(fastRun ? fastSpeed : baseSpeed);
  } else if (distRight == minDist) {
    turnByDegree(90);
    moveForwardOneCell(fastRun ? fastSpeed : baseSpeed);
  } else if (distBack == minDist) {
    turnByDegree(180);
    moveForwardOneCell(fastRun ? fastSpeed : baseSpeed);
  }
}

// ======================
// Setup and Main Loop
// ======================
void setup() {
  Serial.begin(115200);
  
  // Initialize motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  driveStop();
  
  // Setup ultrasonic sensor pins
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  // Initialize MPU6050
  mpuSetup();
  calibration();
  
  // Initialize maze mapping arrays
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      wallsGrid[y][x] = 0;
      distanceGrid[y][x] = MAX_DISTANCE;
    }
  }
  
  // Set maze outer boundaries:
  for (int i = 0; i < MAZE_SIZE; i++) {
    wallsGrid[0][i]         |= 0x04; // South wall for bottom row
    wallsGrid[MAZE_SIZE-1][i] |= 0x01; // North wall for top row
    wallsGrid[i][0]         |= 0x08; // West wall for left column
    wallsGrid[i][MAZE_SIZE-1] |= 0x02; // East wall for right column
  }
  // Mark starting cell boundaries if needed (e.g. start at (0,0))
  wallsGrid[START_Y][START_X] |= 0x04; // Example: south wall at start
  wallsGrid[START_Y][START_X] |= 0x08; // Example: west wall at start
  
  // Set initial position and orientation
  posX = START_X;
  posY = START_Y;
  currentDirection = 0;  // Assume starting facing North
  
  // Load maze data from EEPROM if available
  if(EEPROM.read(EEPROM_FLAG_ADDR) == 0xA5) {
    for (int idx = 0; idx < MAZE_SIZE * MAZE_SIZE; idx++) {
      int cx = idx % MAZE_SIZE;
      int cy = idx / MAZE_SIZE;
      wallsGrid[cy][cx] = EEPROM.read(EEPROM_MAZE_START_ADDR + idx);
    }
    Serial.println("Loaded maze from EEPROM.");
    computeDistances(GOAL_X, GOAL_Y);
  } else {
    // Initialize distance grid with a simple Manhattan distance heuristic
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int x = 0; x < MAZE_SIZE; x++) {
        distanceGrid[y][x] = abs(GOAL_X - x) + abs(GOAL_Y - y);
      }
    }
  }
}

void loop() {
  static bool mazeSolved = false;
  static bool fastRun = false;
  
  if (!mazeSolved) {
    // Exploration phase: scan walls, update distances, and decide next move.
    scanWalls();
    computeDistances(GOAL_X, GOAL_Y);
    decideAndMove(false);
    if (posX == GOAL_X && posY == GOAL_Y) {
      mazeSolved = true;
      Serial.println("Goal reached! Exploration complete.");
      EEPROM.update(EEPROM_FLAG_ADDR, 0xA5);
      int idx = 0;
      for (int y = 0; y < MAZE_SIZE; y++) {
        for (int x = 0; x < MAZE_SIZE; x++) {
          EEPROM.update(EEPROM_MAZE_START_ADDR + idx, wallsGrid[y][x]);
          idx++;
        }
      }
      Serial.println("Maze data saved to EEPROM.");
      // Debug print of the maze bit array
      debugPrintMaze();
      // Prepare for a fast run using the known maze.
      computeDistances(GOAL_X, GOAL_Y);
      posX = START_X; posY = START_Y;
      currentDirection = 0;
      fastRun = true;
    }
  } 
  else if (fastRun) {
    if (posX == GOAL_X && posY == GOAL_Y) {
      driveStop();
      Serial.println("Fast run complete!");
      fastRun = false;
      while (true) { delay(1000); }
    } else {
      decideAndMove(true);
    }
  }
  
  delay(5);
}
