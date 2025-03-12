#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ==================== Hardware Definitions ====================
// ---------- Ultrasonic Sensors ----------
#define leftTrig 6
#define leftEcho 7
#define frontTrig 8
#define frontEcho 9
#define rightTrig 12
#define rightEcho 13

// ---------- Motor Control Pins ----------
#define motor1Speed 10
#define motor2Speed 11
#define motor1A A0
#define motor1B A1
#define motor2A 5
#define motor2B 4

// ---------- Encoder Pins ----------
#define encoderPinA 2
#define encoderPinB 3

// ==================== Maze & Competition Parameters ======================
const int MAZE_SIZE = 8;          // 8x8 grid
const int CELL_SIZE_CM = 25;      // Each cell is 25cm square

// Start and goal positions (using 0-indexed grid: top-left is (0,0))
const int START_X = 0;
const int START_Y = 0;
const int GOAL_X  = 7;
const int GOAL_Y  = 7;

const int WALL_DISTANCE_CM = 15;  // Distance threshold for wall detection

// Movement calibration: we use distance in cm for a cell move.
const float cellDistanceCm = CELL_SIZE_CM;

// ==================== Global Variables for Sensors & Motion ====================
// ---------- Ultrasonic Sensor ----------
float getDistance(int direction);

// ---------- MPU6050 (Gyroscope) ----------
const int MPU = 0x68;
float elapsedTime, currentTime, previousTime;
float GyroErrorZ, gyroOutputBuffer = 0, yaw = 0, angle = 0;
const float GYRO_SCALE = 1.0 / 131.0;

// ---------- Motor Speed (PWM Values) ----------
const byte BASE_SPEED  = 105;   // Slow speed during exploration
const byte TURN_SPEED  = 150;   // Speed during turning
const byte FAST_SPEED  = 150;   // Higher speed during fast-run
int leftSpeedVal, rightSpeedVal;

// ---------- Encoder Pulse Counts ----------
volatile byte pulsesLeft = 0;
volatile byte pulsesRight = 0;

// Interrupt service routines for encoders
void counterLeftUpdate() { pulsesLeft++; }
void counterRightUpdate() { pulsesRight++; }

// ==================== Maze Data Structures ====================
// Each cell stores wall information as a bitmask (bit 0: North, 1: East, 2: South, 3: West)
int wallMap[MAZE_SIZE][MAZE_SIZE];
// Flood-fill distances (number of steps to reach the goal)
int floodFill[MAZE_SIZE][MAZE_SIZE];

// Robot’s current position and heading (0 = North, 1 = East, 2 = South, 3 = West)
int currX = START_X, currY = START_Y;
int heading = 0;

// ==================== Robot State Machine ====================
// Define states as constants (no enums)
#define STATE_IDLE         0
#define STATE_TURNING      1
#define STATE_MOVING       2
#define STATE_EXPLORATION  3
#define STATE_FAST_RUN     4

int robotState = STATE_EXPLORATION;

// For non-blocking timing:
unsigned long stateStartTime = 0;
unsigned long moveDuration = 0;     // Duration for a cell move (ms)
unsigned long turnDuration = 500;   // 90° turn duration (ms); adjust as needed

// Fast-run path: sequence of directions (0=N, 1=E, 2=S, 3=W)
const int MAX_PATH_LENGTH = 100;
int fastPath[MAX_PATH_LENGTH];
int fastPathLength = 0;
int fastPathIndex = 0;          // Index into fastPath during fast-run

// ==================== Function Prototypes ====================
void ultrasonicSetup();
float getDistance(int direction);
void mpuSetup();
bool getOrientation();
void calculateError();
void updateMPU();
void motorSetup();
void resetMotor1();
void resetMotor2();
void goForwardMotor1();
void goForwardMotor2();
void stopMotors();
void encoderSetup();
float getMovingDistance();
void resetDistance();

void updateCurrentCellWalls(); // Read sensors & update wallMap for current cell
void floodFillUpdate();        // Recalculate flood-fill distances from the goal
int decideNextMove();          // Decide next move based on floodFill values
void startMove();
void updateMove();             // Check non-blocking movement
void startTurn(int desiredDirection);
void updateTurn();             // Check non-blocking turning
void computeFastPath();        // Compute shortest path from start to goal
void executeFastRun();         // Follow computed path in fast-run phase

// ==================== Setup Functions ====================
void setup() {
  Serial.begin(9600);
  
  // Initialize Ultrasonic Sensors
  ultrasonicSetup();
  
  // Initialize MPU6050
  mpuSetup();
  previousTime = millis();
  currentTime = millis();

  // Initialize Motor Control
  motorSetup();

  // Initialize Encoder and attach interrupts
  encoderSetup();

  // Initialize maze structures: assume no walls (0) until discovered.
  for (int i = 0; i < MAZE_SIZE; i++){
    for (int j = 0; j < MAZE_SIZE; j++){
      wallMap[i][j] = 0;
      floodFill[i][j] = 999;  // High value means unknown/unreached
    }
  }
  // For flood-fill, set goal cell distance to 0.
  floodFill[GOAL_Y][GOAL_X] = 0;

  // Set initial robot state.
  robotState = STATE_EXPLORATION;
  currX = START_X; currY = START_Y;
  heading = 0;  // Starting facing North
  stateStartTime = millis();
  Serial.println("Maze exploration started...");
}

// ==================== Loop Function ====================
void loop() {
  // Update MPU6050 for heading control
  updateMPU();
  
  // State machine (non-blocking control)
  if(robotState == STATE_EXPLORATION) {
    // Every ~100 ms, update cell data and decide next move.
    if(millis() - stateStartTime > 100) {
      updateCurrentCellWalls();
      floodFillUpdate();
      int nextDir = decideNextMove();
      Serial.print("Current cell: (");
      Serial.print(currX);
      Serial.print(", ");
      Serial.print(currY);
      Serial.print(") | Flood-fill value: ");
      Serial.println(floodFill[currY][currX]);

      // If at goal, finish exploration.
      if (currX == GOAL_X && currY == GOAL_Y) {
        Serial.println("Goal reached during exploration!");
        computeFastPath();
        robotState = STATE_FAST_RUN;
        fastPathIndex = 0;
        stateStartTime = millis();
      }
      else if(nextDir != heading) {
        // Need to turn toward nextDir.
        startTurn(nextDir);
        robotState = STATE_TURNING;
      }
      else {
        // Already facing desired direction; start moving one cell.
        resetDistance();
        startMove();
        robotState = STATE_MOVING;
      }
    }
  }
  else if(robotState == STATE_TURNING) {
    updateTurn();
  }
  else if(robotState == STATE_MOVING) {
    updateMove();
  }
  else if(robotState == STATE_FAST_RUN) {
    executeFastRun();
  }
  else if(robotState == STATE_IDLE) {
    // Idle state: wait a bit before next action.
    if(millis() - stateStartTime > 100) {
      robotState = STATE_EXPLORATION;
      stateStartTime = millis();
    }
  }
}

// ==================== Sensor and Module Functions ====================

// ---------- Ultrasonic Sensor Functions ----------
void ultrasonicSetup(){
  pinMode(leftEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(frontTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
}

float getDistance(int direction) {
  byte trigPinNo, echoPinNo;
  if(direction == 0) {         // FRONT
    trigPinNo = frontTrig; echoPinNo = frontEcho;
  } else if(direction == 1) {  // LEFT
    trigPinNo = leftTrig; echoPinNo = leftEcho;
  } else if(direction == 2) {  // RIGHT
    trigPinNo = rightTrig; echoPinNo = rightEcho;
  } else {
    return 999; // invalid direction
  }
  digitalWrite(trigPinNo, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPinNo, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinNo, LOW);
  long duration = pulseIn(echoPinNo, HIGH, 30000); // timeout to avoid blocking
  float distance = (duration * 0.034613) / 2.0;
  return distance;
}

// ---------- MPU6050 Functions ----------
bool getOrientation(){
  gyroOutputBuffer = 0;
  Wire.beginTransmission(MPU);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 2, true);
  gyroOutputBuffer = (Wire.read() << 8 | Wire.read()) * GYRO_SCALE;
  return (gyroOutputBuffer != 0);
}

void calculateError() {
  byte c = 0;
  GyroErrorZ = 0;
  while (c < 200) {
    if(getOrientation()){
      GyroErrorZ += gyroOutputBuffer;
      c++;
    }
  }
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("MPU6050 gyroscope calibrated");
}

void mpuSetup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calculateError();
}

void updateMPU(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) * 0.001;
  getOrientation();
  gyroOutputBuffer -= GyroErrorZ;
  yaw += gyroOutputBuffer * elapsedTime;
  angle = yaw;
}

// ---------- Motor Functions ----------
void motorSetup(){
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  leftSpeedVal = BASE_SPEED;
  rightSpeedVal = BASE_SPEED;
  analogWrite(motor1Speed, rightSpeedVal);
  analogWrite(motor2Speed, leftSpeedVal);
  stopMotors();
}

void resetMotor1(){
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
}

void resetMotor2(){
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}

void goForwardMotor1(){
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
}

void goForwardMotor2(){
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}

void stopMotors(){
  resetMotor1();
  resetMotor2();
  analogWrite(motor1Speed, 0);
  analogWrite(motor2Speed, 0);
}

// ---------- Encoder Functions ----------
void encoderSetup(){
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), counterLeftUpdate, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), counterRightUpdate, RISING);
}

float getMovingDistance(){
  noInterrupts();
  // Here we assume a linear scale (you may replace with encoder tick calibration)
  float distance = (((pulsesLeft + pulsesRight) / 2.0) * (CELL_SIZE_CM / 25.0));
  interrupts();
  return distance;
}

void resetDistance(){
  noInterrupts();
  pulsesLeft = 0;
  pulsesRight = 0;
  interrupts();
}

// ==================== Maze Functions ====================

// Reads sensor values and updates wallMap for the current cell.
// Logs raw sensor readings and the interpreted wall booleans.
void updateCurrentCellWalls(){
  float frontD = getDistance(0);
  float leftD  = getDistance(1);
  float rightD = getDistance(2);
  
  // Log raw sensor readings.
  Serial.print("Sensor Readings: Front: ");
  Serial.print(frontD);
  Serial.print(" cm, Left: ");
  Serial.print(leftD);
  Serial.print(" cm, Right: ");
  Serial.print(rightD);
  Serial.println(" cm");
  
  int cellWalls = 0;
  // Map sensor readings to absolute walls based on current heading.
  if(heading == 0) { // Facing North: front -> North, left -> West, right -> East.
    if(frontD < WALL_DISTANCE_CM) cellWalls |= 1;      // North
    if(rightD < WALL_DISTANCE_CM) cellWalls |= 2;        // East
    if(leftD  < WALL_DISTANCE_CM) cellWalls |= 8;        // West
  }
  else if(heading == 1) { // Facing East: front -> East, left -> North, right -> South.
    if(frontD < WALL_DISTANCE_CM) cellWalls |= 2;        // East
    if(rightD < WALL_DISTANCE_CM) cellWalls |= 4;        // South
    if(leftD  < WALL_DISTANCE_CM) cellWalls |= 1;        // North
  }
  else if(heading == 2) { // Facing South: front -> South, left -> East, right -> West.
    if(frontD < WALL_DISTANCE_CM) cellWalls |= 4;        // South
    if(rightD < WALL_DISTANCE_CM) cellWalls |= 8;        // West
    if(leftD  < WALL_DISTANCE_CM) cellWalls |= 2;        // East
  }
  else if(heading == 3) { // Facing West: front -> West, left -> South, right -> North.
    if(frontD < WALL_DISTANCE_CM) cellWalls |= 8;        // West
    if(rightD < WALL_DISTANCE_CM) cellWalls |= 1;        // North
    if(leftD  < WALL_DISTANCE_CM) cellWalls |= 4;        // South
  }
  
  wallMap[currY][currX] = cellWalls;
  Serial.print("Updated cell (");
  Serial.print(currX);
  Serial.print(", ");
  Serial.print(currY);
  Serial.print(") wall bitmask: ");
  Serial.println(cellWalls);
}

// Flood-fill: recalc floodFill distances from the goal using BFS.
// Logs the resulting grid for debugging.
void floodFillUpdate(){
  for (int i = 0; i < MAZE_SIZE; i++){
    for (int j = 0; j < MAZE_SIZE; j++){
      floodFill[i][j] = 999;
    }
  }
  floodFill[GOAL_Y][GOAL_X] = 0;
  
  struct Cell { int x, y; };
  Cell queue[MAZE_SIZE*MAZE_SIZE];
  int qStart = 0, qEnd = 0;
  queue[qEnd++] = {GOAL_X, GOAL_Y};
  
  while(qStart < qEnd) {
    Cell c = queue[qStart++];
    int x = c.x, y = c.y;
    int currentVal = floodFill[y][x];
    
    // Check neighbors: North, East, South, West.
    if (y > 0 && !(wallMap[y][x] & 1)) {
      if (floodFill[y-1][x] > currentVal + 1) {
        floodFill[y-1][x] = currentVal + 1;
        queue[qEnd++] = {x, y-1};
      }
    }
    if (x < MAZE_SIZE-1 && !(wallMap[y][x] & 2)) {
      if (floodFill[y][x+1] > currentVal + 1) {
        floodFill[y][x+1] = currentVal + 1;
        queue[qEnd++] = {x+1, y};
      }
    }
    if (y < MAZE_SIZE-1 && !(wallMap[y][x] & 4)) {
      if (floodFill[y+1][x] > currentVal + 1) {
        floodFill[y+1][x] = currentVal + 1;
        queue[qEnd++] = {x, y+1};
      }
    }
    if (x > 0 && !(wallMap[y][x] & 8)) {
      if (floodFill[y][x-1] > currentVal + 1) {
        floodFill[y][x-1] = currentVal + 1;
        queue[qEnd++] = {x-1, y};
      }
    }
  }
  
  // Log the flood-fill grid.
  Serial.println("Flood-fill grid:");
  for (int i = 0; i < MAZE_SIZE; i++){
    for (int j = 0; j < MAZE_SIZE; j++){
      Serial.print(floodFill[i][j]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

// Decide next move based on the floodFill grid.
// Returns the desired absolute direction (0 = North, 1 = East, 2 = South, 3 = West).
int decideNextMove(){
  int bestDir = -1;
  int bestVal = 999;
  
  // NORTH
  if (!(wallMap[currY][currX] & 1) && (currY > 0)) {
    if(floodFill[currY-1][currX] < bestVal) { bestVal = floodFill[currY-1][currX]; bestDir = 0; }
  }
  // EAST
  if (!(wallMap[currY][currX] & 2) && (currX < MAZE_SIZE-1)) {
    if(floodFill[currY][currX+1] < bestVal) { bestVal = floodFill[currY][currX+1]; bestDir = 1; }
  }
  // SOUTH
  if (!(wallMap[currY][currX] & 4) && (currY < MAZE_SIZE-1)) {
    if(floodFill[currY+1][currX] < bestVal) { bestVal = floodFill[currY+1][currX]; bestDir = 2; }
  }
  // WEST
  if (!(wallMap[currY][currX] & 8) && (currX > 0)) {
    if(floodFill[currY][currX-1] < bestVal) { bestVal = floodFill[currY][currX-1]; bestDir = 3; }
  }
  Serial.print("Decided next move direction: ");
  Serial.println(bestDir);
  return bestDir;
}

// ==================== Movement Functions (Non-Blocking) ====================

// Start moving one cell forward.
void startMove(){
  Serial.println("Starting move forward one cell.");
  stateStartTime = millis();
  resetDistance();
  // Set motor speeds (use BASE_SPEED during exploration; FAST_SPEED during fast-run).
  analogWrite(motor1Speed, leftSpeedVal);
  analogWrite(motor2Speed, rightSpeedVal);
  goForwardMotor1();
  goForwardMotor2();
}

// Continuously check encoder feedback to determine if a cell has been traversed.
void updateMove(){
  float distance = getMovingDistance();
  if(distance >= cellDistanceCm) {
    stopMotors();
    // Update current cell coordinates based on heading.
    if(heading == 0) currY--;       // Moving North (assuming y decreases upward)
    else if(heading == 1) currX++;   // East
    else if(heading == 2) currY++;   // South
    else if(heading == 3) currX--;   // West
    
    Serial.print("Arrived at cell: (");
    Serial.print(currX);
    Serial.print(", ");
    Serial.print(currY);
    Serial.println(")");
    
    robotState = STATE_IDLE;
    stateStartTime = millis();
  }
}

// Initiate a turn toward the desired absolute direction.
void startTurn(int desiredDirection){
  Serial.print("Initiating turn. Current heading: ");
  Serial.print(heading);
  Serial.print(" -> Desired heading: ");
  Serial.println(desiredDirection);
  stateStartTime = millis();
  heading = desiredDirection;  // For simulation purposes; in practice, sensor feedback would update heading.
}

// Check if the turning action is complete.
void updateTurn(){
  if(millis() - stateStartTime >= turnDuration){
    Serial.println("Turn complete.");
    robotState = STATE_IDLE;
    stateStartTime = millis();
  }
}

// ==================== Fast Run Functions ====================
// Compute the shortest path from start to goal using the floodFill grid.
// The path is stored as a sequence of directions.
void computeFastPath(){
  int x = START_X, y = START_Y;
  fastPathLength = 0;
  while(!(x == GOAL_X && y == GOAL_Y) && fastPathLength < MAX_PATH_LENGTH) {
    int currentVal = floodFill[y][x];
    int nextDir = -1;
    if (y > 0 && !(wallMap[y][x] & 1) && floodFill[y-1][x] == currentVal - 1) {
      nextDir = 0; y = y - 1;
    }
    else if (x < MAZE_SIZE - 1 && !(wallMap[y][x] & 2) && floodFill[y][x+1] == currentVal - 1) {
      nextDir = 1; x = x + 1;
    }
    else if (y < MAZE_SIZE - 1 && !(wallMap[y][x] & 4) && floodFill[y+1][x] == currentVal - 1) {
      nextDir = 2; y = y + 1;
    }
    else if (x > 0 && !(wallMap[y][x] & 8) && floodFill[y][x-1] == currentVal - 1) {
      nextDir = 3; x = x - 1;
    }
    if(nextDir == -1) break;
    fastPath[fastPathLength++] = nextDir;
  }
  Serial.println("Fast run path computed:");
  for (int i = 0; i < fastPathLength; i++){
    Serial.print(fastPath[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Reset robot position and heading for fast run.
  currX = START_X; currY = START_Y;
  heading = 0;
  fastPathIndex = 0;
  leftSpeedVal = FAST_SPEED;
  rightSpeedVal = FAST_SPEED;
  robotState = STATE_FAST_RUN;
  stateStartTime = millis();
}

// Follow the computed fast-run path at higher speeds.
void executeFastRun(){
  if(fastPathIndex >= fastPathLength){
    stopMotors();
    Serial.println("Fast run complete!");
    while(true); // End execution.
  } else {
    int desiredDir = fastPath[fastPathIndex];
    Serial.print("Fast run step ");
    Serial.print(fastPathIndex);
    Serial.print(" - Desired direction: ");
    Serial.println(desiredDir);
    
    if(heading != desiredDir) {
      startTurn(desiredDir);
      robotState = STATE_TURNING;
    } else {
      resetDistance();
      startMove();
      robotState = STATE_MOVING;
      fastPathIndex++;  // Advance to next step after movement completes.
    }
  }
}

// ==================== End of Code ====================
