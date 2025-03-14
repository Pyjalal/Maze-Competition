#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050.h>
#include <Encoder.h>

/***** Maze Parameters *****/
const int MAZE_SIZE = 8;
const int START_X = 0, START_Y = 0;
const int GOAL_X = MAZE_SIZE - 1, GOAL_Y = MAZE_SIZE - 1;
int posX = START_X, posY = START_Y;
int curDir = 0; // 0 = NORTH, 1 = EAST, 2 = SOUTH, 3 = WEST
bool reachedGoal = false;
bool isStuck = false;
bool visited[MAZE_SIZE][MAZE_SIZE] = {false};  // Prevent infinite loops

// Global flag for movement (added)
bool isMoving = false;

/***** Flood Fill Storage *****/
int distanceGrid[MAZE_SIZE][MAZE_SIZE];  // Flood Fill grid
int backtrackX[100], backtrackY[100];      // For backtracking
int backtrackIndex = 0;
int previousBestDistance = 9999;           // For saving Q-table when improvement occurs

/***** Q-Learning Variables (Memory-Optimized) *****/
// Q_table stored as int16_t (fixed-point style)
int16_t Q_table[MAZE_SIZE][MAZE_SIZE][4] = {{{0}}};  
float LEARNING_RATE = 0.5;       // Decays over time
const float MIN_LEARNING_RATE = 0.1;
const float DISCOUNT_FACTOR = 0.9;
float EXPLORATION_RATE = 0.5;    // 50% exploration initially

/***** Movement & Sensor Parameters *****/
const int BASE_SPEED = 100;
const int TURN_SPEED = 120;
const int FAST_SPEED = 150;      // For fast run mode (if used)
const int CELL_DISTANCE = 125;
const int WALL_DISTANCE_CM = 15;
const int ALIGN_THRESHOLD = 3;   // in cm

/***** Motor & Encoder Setup *****/
// Motor control pins and directions (from your provided code):
#define motor1Speed 10
#define motor2Speed 11
#define motor1A A0
#define motor1B A1
#define motor2A 5
#define motor2B 4
const byte equilibriumSpeed = 105;
const byte turningSpeed = 150;
int leftSpeedVal;
int rightSpeedVal;
bool isReachPoint = false;

void resetMotor1(){
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, LOW);
}
void goForwardMotor1(){
  digitalWrite(motor1A, HIGH);
  digitalWrite(motor1B, LOW);
}
void goBackwardMotor1(){
  digitalWrite(motor1A, LOW);
  digitalWrite(motor1B, HIGH);
}
void resetMotor2(){
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, LOW);
}
void goForwardMotor2(){
  digitalWrite(motor2A, HIGH);
  digitalWrite(motor2B, LOW);
}
void goBackwardMotor2(){
  digitalWrite(motor2A, LOW);
  digitalWrite(motor2B, HIGH);
}
void motorSetup(){
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);
  leftSpeedVal = equilibriumSpeed;
  rightSpeedVal = equilibriumSpeed;
  analogWrite(motor1Speed, rightSpeedVal);
  analogWrite(motor2Speed, leftSpeedVal);
  resetMotor1();
  resetMotor2();
}

/***** Encoder Setup *****/
#define encoderPinA 2
#define encoderPinB 3
volatile byte pulsesLeft = 0;
volatile byte pulsesRight = 0;
const float encoderConstant = 0.05 * PI * 0.0325 * 100;
void counterLeftUpdate() { pulsesLeft++; }
void counterRightUpdate() { pulsesRight++; }
void encoderSetup(){
  pinMode(encoderPinA, INPUT); 
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), counterLeftUpdate, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), counterRightUpdate, RISING);
}
float getMovingDistance(){
  noInterrupts();
  float distance = ((pulsesLeft + pulsesRight) / 2) * encoderConstant;
  interrupts();
  return distance;
}
void resetDistance(){
  pulsesLeft = 0;
  pulsesRight = 0;
}

/***** Ultrasonic Sensors *****/
const int TRIG_FRONT = 6, ECHO_FRONT = 7;
const int TRIG_LEFT = 8, ECHO_LEFT = 11;
const int TRIG_RIGHT = 12, ECHO_RIGHT = 13;

/***** IMU Setup & Functions *****/
// MPU6050 using provided code
const int MPU = 0x68; // MPU6050 I2C address
float elapsedTime, currentTime, previousTime;
float GyroErrorZ;
float gyroOutputBuffer = 0;
float yaw;
float angle = 0;
float targetAngle = 0;
const float GYRO_SCALE = 1.0 / 131.0;
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
  Serial.println("Gyroscope calibrated");
}
void mpuSetup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  calculateError();
}
void update(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) * 0.001;
  getOrientation();
  gyroOutputBuffer -= GyroErrorZ;
  yaw += gyroOutputBuffer * elapsedTime;
  angle = yaw;
  Serial.print("Angle: ");
  Serial.println(angle);
}
// alignWithIMU() removed as requested

/***** Time Tracking for Stuck Detection *****/
unsigned long lastMoveTime = millis();

/***** EEPROM Storage *****/
#define EEPROM_START_ADDR 0
void saveQTable() {
  int addr = EEPROM_START_ADDR;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int a = 0; a < 4; a++) {
        EEPROM.put(addr, Q_table[x][y][a]);
        addr += sizeof(int16_t);
      }
    }
  }
}
void loadQTable() {
  int addr = EEPROM_START_ADDR;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int a = 0; a < 4; a++) {
        EEPROM.get(addr, Q_table[x][y][a]);
        addr += sizeof(int16_t);
      }
    }
  }
}

/***** Flood Fill for Path Calculation *****/
void floodFillCompute() {
  const int INF = 9999;
  for (int x = 0; x < MAZE_SIZE; x++) {
    for (int y = 0; y < MAZE_SIZE; y++) {
      distanceGrid[x][y] = INF;
    }
  }
  distanceGrid[GOAL_X][GOAL_Y] = 0;
  int queueX[200], queueY[200];
  int front = 0, rear = 0;
  queueX[rear] = GOAL_X;
  queueY[rear] = GOAL_Y;
  rear++;
  while (front < rear) {
    int x = queueX[front];
    int y = queueY[front];
    front++;
    int currentDist = distanceGrid[x][y];
    if (y < MAZE_SIZE - 1 && distanceGrid[x][y + 1] > currentDist + 1) {
      distanceGrid[x][y + 1] = currentDist + 1;
      queueX[rear] = x;
      queueY[rear] = y + 1;
      rear++;
    }
    if (x < MAZE_SIZE - 1 && distanceGrid[x + 1][y] > currentDist + 1) {
      distanceGrid[x + 1][y] = currentDist + 1;
      queueX[rear] = x + 1;
      queueY[rear] = y;
      rear++;
    }
    if (y > 0 && distanceGrid[x][y - 1] > currentDist + 1) {
      distanceGrid[x][y - 1] = currentDist + 1;
      queueX[rear] = x;
      queueY[rear] = y - 1;
      rear++;
    }
    if (x > 0 && distanceGrid[x - 1][y] > currentDist + 1) {
      distanceGrid[x - 1][y] = currentDist + 1;
      queueX[rear] = x - 1;
      queueY[rear] = y;
      rear++;
    }
  }
  previousBestDistance = distanceGrid[START_X][START_Y];
}

/***** Q-Learning Update *****/
void updateQTable(int prevX, int prevY, int action, float reward) {
  int bestNextAction = chooseBestAction(posX, posY);
  int16_t maxNextQ = Q_table[posX][posY][bestNextAction];
  float updateValue = LEARNING_RATE * (reward + DISCOUNT_FACTOR * maxNextQ - Q_table[prevX][prevY][action]);
  Q_table[prevX][prevY][action] += (int16_t)updateValue;
  if (distanceGrid[START_X][START_Y] < previousBestDistance) {
    saveQTable();
    previousBestDistance = distanceGrid[START_X][START_Y];
  }
}

/***** Choose Best Action (ε-Greedy) *****/
int chooseBestAction(int x, int y) {
  if (random(0, 100) < EXPLORATION_RATE * 100)
    return random(0, 4);
  int bestAction = 0;
  int16_t maxQ = -1000;
  for (int i = 0; i < 4; i++) {
    if (Q_table[x][y][i] > maxQ) {
      maxQ = Q_table[x][y][i];
      bestAction = i;
    }
  }
  return bestAction;
}

/***** Filtered Ultrasonic Reading *****/
long readUltrasonic(int trigPin, int echoPin) {
  long sum = 0;
  const int samples = 3;
  for (int i = 0; i < samples; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long reading = pulseIn(echoPin, HIGH) / 58;
    if (reading == 0 || reading > 300)
      reading = 300;
    sum += reading;
    delay(5);
  }
  return sum / samples;
}

/***** Movement Functions *****/
void moveForward() {
  analogWrite(motor1Speed, equilibriumSpeed);
  analogWrite(motor2Speed, equilibriumSpeed);
  goForwardMotor1();
  goForwardMotor2();
  resetDistance();
}
void stopMotors() {
  analogWrite(motor1Speed, 0);
  analogWrite(motor2Speed, 0);
  resetMotor1();
  resetMotor2();
}
void turnLeft() {
  analogWrite(motor1Speed, turningSpeed);
  analogWrite(motor2Speed, turningSpeed);
  goForwardMotor1();
  goBackwardMotor2();
  delay(500);
  stopMotors();
}
void turnRight() {
  analogWrite(motor1Speed, turningSpeed);
  analogWrite(motor2Speed, turningSpeed);
  goBackwardMotor1();
  goForwardMotor2();
  delay(500);
  stopMotors();
}
void uTurn() {
  analogWrite(motor1Speed, turningSpeed);
  analogWrite(motor2Speed, turningSpeed);
  goForwardMotor1();
  goBackwardMotor2();
  delay(1000);
  stopMotors();
}
void moveBackward() {
  digitalWrite(motor1Speed, 0);
  digitalWrite(motor2Speed, 0);
  goBackwardMotor1();
  goBackwardMotor2();
  analogWrite(motor1Speed, equilibriumSpeed);
  analogWrite(motor2Speed, equilibriumSpeed);
  delay(500);
  stopMotors();
}

/***** moveForwardAfterTurn: Centering After a Turn *****/
unsigned long current = millis();
bool isTurnRight = false, isTurnLeft = false, isUTurn = false;
const byte mazeWidth = 20;
float distanceArr[3] = {0, 0, 0}; // For left/right sensor readings
void moveForwardAfterTurn(){
    current = millis();
    while(true){
        update();  // Update IMU readings
        float requiredAngle;
        if(isTurnRight){
            requiredAngle = targetAngle - angle;
        } else if(isTurnLeft || isUTurn){
            requiredAngle = angle - targetAngle;
        }
        if(requiredAngle <= 0){
            continue;
        } else {
            Serial.println(targetAngle);
            stopMotors();
            resetDistance();
            while(getMovingDistance() < 20){
                if(millis() - current > 30){
                    if(getMovingDistance() < 20){
                        update();
                        Serial.print(String(angle) + " ");
                        if(targetAngle < 0){
                            if(angle - targetAngle < -5){
                                Serial.println("Align Left");
                                update();
                            } else if(angle - targetAngle > 5){
                                Serial.println("Align Right");
                                update();
                            } else {
                                Serial.println("Move Forward");
                                moveForward();
                            }
                        } else {
                            if(angle - targetAngle > 5){
                                Serial.println("Align Right");
                                update();
                            } else if(angle - targetAngle < -5){
                                Serial.println("Align Left");
                                update();
                            } else {
                                Serial.println("Move Forward");
                                moveForward();
                            }
                        }
                        current = millis();
                    } else {
                        break;
                    }
                }
            }
            current = millis();
            while(true){
                float ultrasonicResult = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
                if((getMovingDistance() > 30 && int(ultrasonicResult) % 27 <= 4) || ultrasonicResult < 6){
                  Serial.print(getMovingDistance());
                  Serial.print("  " + String(int(ultrasonicResult) % 27));
                  Serial.print("  " + String(ultrasonicResult));
                  Serial.println();
                  stopMotors();
                  isMoving = false;
                  isReachPoint = true;
                  break;
                } else {
                  if(millis() - current > 30){
                    distanceArr[0] = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
                    distanceArr[1] = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
                    Serial.print("FRONT: " + String(ultrasonicResult) + " LEFT: " + String(distanceArr[0]) + " RIGHT: " + String(distanceArr[1]) + " ");
                    if(distanceArr[0] < mazeWidth && distanceArr[1] < mazeWidth){ 
                        if(distanceArr[0] - distanceArr[1] < -3){
                          Serial.println("Align Right");
                          update();
                        } else if(distanceArr[0] - distanceArr[1] > 3){
                          Serial.println("Align Left");
                          update();
                        } else {
                          Serial.println("Move Forward");
                          moveForward();
                        }
                    } else if(distanceArr[0] < mazeWidth && distanceArr[1] > mazeWidth){ 
                        if(distanceArr[0] < 6){
                          Serial.println("Align Right");
                          update();
                        } else if(distanceArr[0] > 8){
                          Serial.println("Align Left");
                          update();
                        } else {
                          Serial.println("Move Forward");
                          moveForward();
                        }
                    } else if(distanceArr[0] > mazeWidth && distanceArr[1] < mazeWidth){ 
                        if(distanceArr[1] < 6){
                          Serial.println("Align Left");
                          update();
                        } else if(distanceArr[1] > 8){
                          Serial.println("Align Right");
                          update();
                        } else {
                          Serial.println("Move Forward");
                          moveForward();
                        }
                    } else {
                        update();
                        Serial.print(String(angle) + " ");
                        if(angle < 0){
                            if(angle - targetAngle < -10){
                                Serial.println("Align Left");
                                update();
                            } else if(angle - targetAngle > 10){
                                Serial.println("Align Right");
                                update();
                            } else {
                                Serial.println("Move Forward");
                                moveForward();
                            }
                        } else {
                            if(angle - targetAngle > 5){
                                Serial.println("Align Right");
                                update();
                            } else if(angle - targetAngle < -5){
                                Serial.println("Align Left");
                                update();
                            } else {
                                Serial.println("Move Forward");
                                moveForward();
                            }
                        }
                    }
                    current = millis();
                  }
                }
            }
            isReachPoint = true;
            break;
        }
    }
}

/***** detectAndRecoverFromStuck *****/
void detectAndRecoverFromStuck() {
  if (millis() - lastMoveTime > 3000) {
    Serial.println("⚠️ Robot is stuck! Attempting recovery...");
    isStuck = true;
    stopMotors();
    long leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
    long rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
    Serial.println("🔄 Shaking backward...");
    for (int i = 0; i < 3; i++) {
      moveBackward();
      delay(300);
      moveForward();
      delay(300);
    }
    if (leftDist > rightDist) {
      Serial.println("🔄 Trying left turn for recovery...");
      turnLeft();
    } else {
      Serial.println("🔄 Trying right turn for recovery...");
      turnRight();
    }
    lastMoveTime = millis();
    isStuck = false;
  }
}

/***** Backtracking for Dead-Ends *****/
void backtrack() {
  if (backtrackIndex > 0) {
    backtrackIndex--;
    posX = backtrackX[backtrackIndex];
    posY = backtrackY[backtrackIndex];
    moveForward();
  } else {
    Serial.println("No backtracking path available!");
  }
}

/***** Update Maze Memory (Detect New Obstacles) *****/
void updateMazeMemory() {
  if (!knownWalls[posX][posY]) {
    knownWalls[posX][posY] = true;
    floodFillCompute();
  }
}

/***** Main Robot Movement Function *****/
void moveRobot() {
  if (reachedGoal) return;
  
  // Prevent loops by checking if cell was visited:
  if (visited[posX][posY]) {
    Serial.println("Already visited; backtracking...");
    backtrack();
    return;
  }
  visited[posX][posY] = true;
  
  long frontDist = readUltrasonic(TRIG_FRONT, ECHO_FRONT);
  long leftDist = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  long rightDist = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
  // Save current position for backtracking:
  backtrackX[backtrackIndex] = posX;
  backtrackY[backtrackIndex] = posY;
  backtrackIndex++;
  
  int prevX = posX, prevY = posY;
  int action = chooseBestAction(posX, posY);
  float reward = -0.1;
  
  if (posX == GOAL_X && posY == GOAL_Y) {
    Serial.println("🎉 Goal Reached!");
    reachedGoal = true;
    return;
  }
  
  detectAndRecoverFromStuck();
  avoidCollisions();
  
  if (frontDist > WALL_DISTANCE_CM) {
    moveForward();
    reward = 1.0;
    lastMoveTime = millis();
  } else {
    updateMazeMemory();
    if (leftDist > rightDist) {
      turnLeft();
      moveForwardAfterTurn();  // Center after turning
      action = 3;
    } else if (rightDist > leftDist) {
      turnRight();
      moveForwardAfterTurn();  // Center after turning
      action = 1;
    } else {
      uTurn();
      moveForwardAfterTurn();  // Center after turning
      reward = -1.0;
    }
  }
  
  updateQTable(prevX, prevY, action, reward);
  saveQTable();
  updateLearningRates();
  // alignWithIMU() removed per request
}

/***** Debug Visual Map *****/
void debugVisualMap() {
  Serial.println("Maze State:");
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (x == posX && y == posY)
        Serial.print("R ");
      else if (knownWalls[x][y])
        Serial.print("X ");
      else
        Serial.print(". ");
    }
    Serial.println();
  }
}

/***** Main Loop *****/
void loop() {
  moveRobot();
  debugVisualMap();
  delay(500);
}
