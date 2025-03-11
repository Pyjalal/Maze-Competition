// ======================
// Global Definitions and Includes
// ======================
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h> 
#include <Wire.h>
#include <EEPROM.h>
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
// Maze & Competition Parameters
// ======================
const int MAZE_SIZE = 8;          // 8 x 8 grid
const int CELL_SIZE_CM = 25;      // Each cell is 25 cm x 25 cm

// Starting and goal positions (exit cell)
const int START_X = 0;
const int START_Y = 0;
const int GOAL_X  = 7;
const int GOAL_Y  = 7;

const int MAX_DISTANCE = 255;
const int WALL_DISTANCE_CM = 10;  // Threshold to detect a wall

// For encoder-based travel (if used)
int cellDistanceTicks = 410; // Example value; adjust as needed

// Global maze arrays (for mapping if needed)
int posX = START_X, posY = START_Y;  
uint8_t wallsGrid[MAZE_SIZE][MAZE_SIZE] = {0}; 
uint8_t distanceGrid[MAZE_SIZE][MAZE_SIZE];

// ======================
// Other Global Variables
// ======================
int currentDirection = 0;  // 0 = North, 1 = East, 2 = South, 3 = West
int baseSpeed = 100;       // Speed for motion commands

// ======================
// Sensors, Encoders, and Other Objects
// ======================
Adafruit_MPU6050 mpu;
Encoder enc(ENC_PIN_A, ENC_PIN_B);

// ======================
// Function Prototypes
// ======================
long measureDistance(int trigPin, int echoPin);
void driveStop();
void driveForward(int speed);
void driveTurnLeft(int speed);
void driveTurnRight(int speed);
void updatePosition(int dx, int dy);
void turnByDegrees(int degrees, int speed);
void moveForwardOneCell(int speed);
void updateDirection(int degrees);
void updatePositionAccordingToDirection();
void computeFloodFill();
void printDistanceGrid();
void navigateUsingFloodFill();

// ======================
// Function: measureDistance()
// Measures distance (in cm) using the given ultrasonic sensor pins.
long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration / 2.0) * 0.0343;
  return distance;
}

// ======================
// Motor Drive Functions
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
// Function: updatePosition()
// Updates the robot's grid position by the given offsets.
void updatePosition(int dx, int dy) {
  posX += dx;
  posY += dy;
  Serial.print("New Grid Position: (");
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(posY);
  Serial.println(")");
}

// ======================
// Helper Function: updateDirection()
// Updates currentDirection based on the turn degrees (assumes multiples of 90).
void updateDirection(int degrees) {
  int turns = (degrees / 90); // Positive for right, negative for left
  currentDirection = (currentDirection + turns + 4) % 4;
}

// ======================
// Function: turnByDegrees()
// Turns the robot by the specified degrees (must be a multiple of 90)
// and updates currentDirection.
void turnByDegrees(int degrees, int speed) {
  if (degrees < 0) {
    driveTurnLeft(speed);
  } else {
    driveTurnRight(speed);
  }
  delay(500);  // Adjust delay for a 90° turn (tweak as needed)
  updateDirection(degrees);
  driveStop();
}

// ======================
// Function: moveForwardOneCell()
// Moves the robot forward one cell and then stops.
void moveForwardOneCell(int speed) {
  driveForward(speed);
  delay(450);  // Adjust delay to travel one cell (e.g., 25 cm)
  driveStop();
}

// ======================
// Helper Function: updatePositionAccordingToDirection()
// Updates grid position based on currentDirection.
void updatePositionAccordingToDirection() {
  // currentDirection: 0 = North, 1 = East, 2 = South, 3 = West
  if (currentDirection == 0) {
    updatePosition(0, 1);
  } else if (currentDirection == 1) {
    updatePosition(1, 0);
  } else if (currentDirection == 2) {
    updatePosition(0, -1);
  } else if (currentDirection == 3) {
    updatePosition(-1, 0);
  }
}

// ======================
// Flood-Fill: computeFloodFill()
// Computes the distance from each cell to the goal (GOAL_X, GOAL_Y)
// using the current wall information in wallsGrid.
void computeFloodFill() {
  // Initialize distanceGrid to MAX_DISTANCE for all cells
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      distanceGrid[y][x] = MAX_DISTANCE;
    }
  }
  // Set the goal cell distance to 0
  distanceGrid[GOAL_Y][GOAL_X] = 0;
  
  bool updated = true;
  while (updated) {
    updated = false;
    for (int y = 0; y < MAZE_SIZE; y++) {
      for (int x = 0; x < MAZE_SIZE; x++) {
        // Skip the goal cell itself
        if (x == GOAL_X && y == GOAL_Y) continue;
        uint8_t minNeighbor = MAX_DISTANCE;
        // Check North neighbor: if no wall in current cell to the north
        if (!(wallsGrid[y][x] & 0x01) && (y + 1 < MAZE_SIZE)) {
          if (distanceGrid[y + 1][x] < minNeighbor)
            minNeighbor = distanceGrid[y + 1][x];
        }
        // Check East neighbor:
        if (!(wallsGrid[y][x] & 0x02) && (x + 1 < MAZE_SIZE)) {
          if (distanceGrid[y][x + 1] < minNeighbor)
            minNeighbor = distanceGrid[y][x + 1];
        }
        // Check South neighbor:
        if (!(wallsGrid[y][x] & 0x04) && (y - 1 >= 0)) {
          if (distanceGrid[y - 1][x] < minNeighbor)
            minNeighbor = distanceGrid[y - 1][x];
        }
        // Check West neighbor:
        if (!(wallsGrid[y][x] & 0x08) && (x - 1 >= 0)) {
          if (distanceGrid[y][x - 1] < minNeighbor)
            minNeighbor = distanceGrid[y][x - 1];
        }
        if (minNeighbor < MAX_DISTANCE && distanceGrid[y][x] > minNeighbor + 1) {
          distanceGrid[y][x] = minNeighbor + 1;
          updated = true;
        }
      }
    }
  }
}

// ======================
// Function: printDistanceGrid()
// Prints the current flood–fill distance grid to Serial.
void printDistanceGrid() {
  Serial.println("Flood–Fill Distance Grid:");
  for (int y = MAZE_SIZE - 1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      if (distanceGrid[y][x] == MAX_DISTANCE)
        Serial.print(" X ");
      else {
        if(distanceGrid[y][x] < 10) Serial.print(" ");
        Serial.print(distanceGrid[y][x]);
        Serial.print(" ");
      }
    }
    Serial.println();
  }
  Serial.println("--------------------");
}

// ======================
// Flood-Fill Navigation: navigateUsingFloodFill()
// Uses the computed flood–fill grid to decide the next move.
// The robot turns toward the neighbor with the lowest flood–fill value.
void navigateUsingFloodFill() {
  // If the robot has reached the goal cell, stop.
  if (posX == GOAL_X && posY == GOAL_Y) {
    Serial.println("Goal reached! Stopping.");
    driveStop();
    while (true) {
      delay(1000);
    }
  }
  
  // Update the flood–fill grid with current maze data.
  computeFloodFill();
  printDistanceGrid();
  
  uint8_t currentVal = distanceGrid[posY][posX];
  uint8_t bestVal = currentVal;
  int bestDirection = -1; // 0 = North, 1 = East, 2 = South, 3 = West

  // Check accessible neighbors:
  // North neighbor: if no wall north and within bounds
  if (!(wallsGrid[posY][posX] & 0x01) && (posY + 1 < MAZE_SIZE)) {
    if (distanceGrid[posY + 1][posX] < bestVal) {
      bestVal = distanceGrid[posY + 1][posX];
      bestDirection = 0;
    }
  }
  // East neighbor:
  if (!(wallsGrid[posY][posX] & 0x02) && (posX + 1 < MAZE_SIZE)) {
    if (distanceGrid[posY][posX + 1] < bestVal) {
      bestVal = distanceGrid[posY][posX + 1];
      bestDirection = 1;
    }
  }
  // South neighbor:
  if (!(wallsGrid[posY][posX] & 0x04) && (posY - 1 >= 0)) {
    if (distanceGrid[posY - 1][posX] < bestVal) {
      bestVal = distanceGrid[posY - 1][posX];
      bestDirection = 2;
    }
  }
  // West neighbor:
  if (!(wallsGrid[posY][posX] & 0x08) && (posX - 1 >= 0)) {
    if (distanceGrid[posY][posX - 1] < bestVal) {
      bestVal = distanceGrid[posY][posX - 1];
      bestDirection = 3;
    }
  }
  
  // If a neighbor with a lower distance exists, turn toward that cell.
  if (bestDirection != -1) {
    int diff = bestDirection - currentDirection;
    // Adjust for wrap-around:
    if (diff > 2) diff -= 4;
    if (diff < -2) diff += 4;
    int turnAngle = diff * 90;
    turnByDegrees(turnAngle, baseSpeed);
    moveForwardOneCell(baseSpeed);
    updatePositionAccordingToDirection();
  }
  else {
    // No accessible neighbor found with a lower value. 
    Serial.println("No lower neighbor found. Stopping.");
    driveStop();
    while(true) { delay(1000); }
  }
}

// ======================
// Setup Function
// ======================
void setup() {
  Serial.begin(115200);
  delay(100);

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
  
  // (Optional) Initialize maze mapping arrays
  for (int y = 0; y < MAZE_SIZE; y++) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      wallsGrid[y][x] = 0;
      distanceGrid[y][x] = MAX_DISTANCE;
    }
  }
  
  Serial.println("Setup complete. Starting flood-fill navigation mode.");
}

// ======================
// Main Loop: Use Flood-Fill Navigation
// ======================
void loop() {
  navigateUsingFloodFill();
  delay(100); // Small delay between moves
}
