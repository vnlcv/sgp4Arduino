/*
Homing Process Test: Check if the motor correctly finds the zero-degree reference point.
Calibration Process Test: Confirm that the motor accurately counts the steps required for a full rotation.
Target Angle Movement Test: Test if the motor can move to specified angles and return to zero.
Motor Direction Test: Ensure the motor chooses the shortest path when rotating to a new angle.
*/
#include "Arduino_APDS9960.h"
#include "Arduino_HS300x.h"

// Motor pin definitions
const int stepPinBottom = 5;  //PUL -Pulse
const int dirPinBottom = 6; //DIR -Direction
const int enPinBottom = 7;  //ENA -Enable
const int microstep = 4;
const int pulse_rev = 200 * microstep;  // Steps for one full revolution
const int magstep = (700 / microstep); // Delay between pulses

int totalSteps = 0;  
bool homingComplete = false; 
bool countingSteps = false; 
int proximity = 0;
int currentPosition = 0;        // Current angle position in steps
int targetSteps = 0;            // Target position in steps

// Input angle in degrees
float psi_d1 = 0; 

void setup() {
  Serial.begin(9600);
  initializeSensors();
  initializeMotorPins();
  testMotorControl();
}

void loop() {
  // float temperature = HS300x.readTemperature();
  // float humidity    = HS300x.readHumidity();
  // Serial.print("Temperature: ");
  // Serial.print(temperature);
  // Serial.print(" °C");
  // Serial.print(" | ");
  // Serial.print("Humidity: ");
  // Serial.print(humidity);
  // Serial.println(" %");

  // Perform homing and calibration if not complete
  if (!homingComplete) {
    homing();
  } else if (homingComplete && countingSteps) {
    calibration();
  } else {
    // Control loop to move to target angle every 50 seconds
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 10000) {  // Update every 10 seconds
      lastUpdate = millis();
      
      // Example to update target angle (psi_d1) randomly 
      psi_d1 = int(psi_d1 + 85) % 360;  // Change target angle
      Serial.print("New psi_d1: ");
      Serial.println(psi_d1);
      
      // Calculate target steps from current position
      targetSteps = angleToSteps(psi_d1);
      rotateToAngle(targetSteps);
    }
  }
}

// Initializes motor control pins
void initializeMotorPins() {
  pinMode(stepPinBottom, OUTPUT);
  pinMode(dirPinBottom, OUTPUT);
  pinMode(enPinBottom, OUTPUT);
  digitalWrite(enPinBottom, LOW);  // Enable motor
}

// Initializes sensors and filter
void initializeSensors() {
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor!");
  }

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1); // Halt if sensor fails
  }
}

// Converts angle to steps based on 0-degree reference
int angleToSteps(float angle) {
  return int((angle / 360.0) * totalSteps);
}

// Rotates motor to the desired angle
void rotateToAngle(int targetSteps) {
  int stepDifference = targetSteps - currentPosition;
  int direction = (stepDifference >= 0) ? 1 : -1;   // If stepdifference >= 0, direction = 1, clockwise. If stepdifference < 0, direction = -1, anticlockwise. 

  // Calculate the shortest path
  int stepsToMove = abs(stepDifference);
  if (stepsToMove > totalSteps / 2) { // If longer path
    stepsToMove = totalSteps - stepsToMove; // Steps in opposite direction
    direction *= -1; // Reverse direction eg 1 to -1
  }

  // Set direction pin
  digitalWrite(dirPinBottom, direction == 1 ? HIGH : LOW); // If direction == 1, set to HIGH; otherwise set to LOW
  Serial.print("Moving to ");
  Serial.print(psi_d1);
  Serial.println(" degrees.");

  // Rotate the required steps
  rotateMotor(stepsToMove);

  // Update current position within totalSteps for one disc revolution 
  currentPosition = targetSteps % totalSteps; 
}

// Rotates the motor by a number of steps
void rotateMotor(int motorSteps) {  // Specify the number of steps to rotate
  // digitalWrite(enPinBottom, LOW);  // Enable motor
  digitalWrite(dirPinBottom, HIGH);  // Set direction
  
  for (int x = 0; x < motorSteps; x++) {
    digitalWrite(stepPinBottom, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPinBottom, LOW);
    delayMicroseconds(magstep);
  }
}

// Homing process to find 0 degree notch
void homing(){
  rotateMotor(1);  // Rotate one step at a time 

  // Read proximity sensor 
  if (APDS.proximityAvailable()) {
    proximity = APDS.readProximity(); 
    Serial.print("Proximity: ");
    Serial.println(proximity); 

    // If proximity between 0 and 10 is detected, 0 degree notch detected
    if (proximity == 0 ) {
        homingComplete = true;  // Homing complete
        Serial.println("Homing complete.");
        countingSteps = true;  // Start counting steps
        totalSteps = 0;  // Reset step count
        currentPosition = 0; // Set current position as 0 degree reference
        
    }
  }
}

// Calibration to count steps for a full rotation
void calibration(){
  rotateMotor(1);  // Rotate one step at a time
  // Count the steps 
        if (countingSteps) {
          totalSteps++;
        } 

  // Read proximity sensor 
  if (APDS.proximityAvailable()) {
    proximity = APDS.readProximity(); 
    Serial.print("Proximity: ");
    Serial.println(proximity); 

    // If proximity between 0 and 10 is detected, full revolution completed
    if (totalSteps >= 100 && proximity == 0 ) {
      countingSteps = false;  // Stop counting steps
      // digitalWrite(enPinBottom, HIGH);  // Disable motor
      Serial.println("Full revolution complete");
      Serial.print("Total steps for 360° rotation: ");
      Serial.println(totalSteps);
    }
  }
}

// Test Code
void testMotorControl() {
  Serial.println("Starting Motor Control Tests...");

  // Test Homing Process
  Serial.println("Testing Homing Process...");
  homingComplete = false;
  countingSteps = false;
  while (!homingComplete) { 
    homing();
  }
  Serial.print("Homing Current Position: ");
  Serial.println(currentPosition);
  // Pause for 10 seconds at the homing position
  Serial.println("Pausing at homing position for 10 seconds...");
  delay(10000);  // 10-second delay
  Serial.println("Resuming after pause.");

  // Test Calibration Process
  Serial.println("Testing Calibration Process...");
  homingComplete = true;
  countingSteps = true;
  totalSteps = 0;
  while (countingSteps) {
    calibration();
  }
  // Pause for 10 seconds at the homing position
  Serial.println("Pausing at homing position for 10 seconds...");
  delay(10000);  // 10-second delay
  Serial.print("Calibration Current Position: ");
  Serial.println(currentPosition);
  Serial.println("Resuming after pause.");

  // Test 5 Full Rotations 
  Serial.println("Testing 5 Full Rotations...");
  int rotations = 5;  // Number of full rotations to perform
  unsigned long startTime, endTime;  // For tracking elapsed time per rotation
  for (int i = 1; i <= rotations; i++) {
      Serial.print("Starting rotation ");
      Serial.print(i);
      Serial.println("...");

      // Record the start time for each rotation
      startTime = millis();

      // Reset homing position and move one full 360-degree rotation
      int stepsForFullRotation = totalSteps;  // Use total steps from calibration for a full circle
      rotateToAngle(stepsForFullRotation);    // Rotate motor to 360 degrees

      // Record the end time after rotation is complete
      endTime = millis();
      unsigned long elapsedTime = endTime - startTime;

      Serial.print("Rotation ");
      Serial.print(i);
      Serial.println(" complete.");
      Serial.print("360 Rotation Current Position: ");
      Serial.println(currentPosition);
      Serial.print("Elapsed time (ms): ");
      Serial.println(elapsedTime);
      Serial.print("Total steps: ");
      Serial.println(totalSteps);

      // Pause for 10 seconds before starting the next rotation
      delay(10000);
  }
  Serial.println("5 Full Rotations Test Complete.");

  // Test Moving to Specific Angles
  Serial.println("Testing Angle Movements...");
  int angles[] = {45, 90, 45, 180, 270, 45, 270, 0};  // Test different angles
  for (int i = 0; i < 8; i++) {
    psi_d1 = angles[i];
    Serial.print("Setting Target Angle: ");
    Serial.println(psi_d1);
    targetSteps = angleToSteps(psi_d1);
    rotateToAngle(targetSteps);
    Serial.print("Moved to Angle: ");
    Serial.println(psi_d1);
    delay(10000);  // 10 second delay
  }
  Serial.print("Returned to 0 degrees. Current Position: ");
  Serial.println(currentPosition);
  Serial.println("Motor Control Tests Complete.");
}

