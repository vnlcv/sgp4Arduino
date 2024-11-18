#include "Arduino_APDS9960.h"
#include "Arduino_HS300x.h"

// Motor pin definitions
const int stepPin1 = 5;  //PUL -Pulse
const int dirPin1 = 6; //DIR -Direction
const int enPin1 = 7;  //ENA -Enable
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
  delay(500);
  Serial.print("Initial homingComplete: ");
  Serial.println(homingComplete);
  initializeSensors();
  initializeMotorPins();
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
    // // Control loop to move to target angle every 50 seconds
    // static unsigned long lastUpdate = 0;
    // if (millis() - lastUpdate >= 10000) {  // Update every 10 seconds
    //   lastUpdate = millis();
      
    //   // Example to update target angle (psi_d1) randomly 
    //   psi_d1 = int(psi_d1 + 85) % 360;  // Change target angle
    //   Serial.print("New psi_d1: ");
    //   Serial.println(psi_d1);
      
    //   // Calculate target steps from current position
    //   targetSteps = angleToSteps(psi_d1);
    //   rotateToAngle(targetSteps);
    // }
  }
}

// Initializes motor control pins
void initializeMotorPins() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, LOW);  // Enable motor
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
  digitalWrite(dirPin1, direction == 1 ? HIGH : LOW); // If direction == 1, set to HIGH; otherwise set to LOW
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
  // digitalWrite(enPin1, LOW);  // Enable motor
  digitalWrite(dirPin1, HIGH);  // Set direction
  
  for (int x = 0; x < motorSteps; x++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(magstep);
  }
}

// Homing process to find 0 degree notch
void homing(){
  Serial.println("Homing starting.");
  rotateMotor(1);  // Rotate one step at a time 

  // Read proximity sensor 
  if (APDS.proximityAvailable()) {
    proximity = APDS.readProximity(); 
    Serial.print("Proximity: ");
    Serial.println(proximity); 

    // If proximity between 0 and 10 is detected, 0 degree notch detected
    if (proximity >= 0 && proximity <= 10) {
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
  Serial.println("Calibration starting.");
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
    if (totalSteps >= 100 && proximity >= 0 && proximity <= 10) {
      countingSteps = false;  // Stop counting steps
      // digitalWrite(enPin1, HIGH);  // Disable motor
      Serial.println("Full revolution complete");
      Serial.print("Total steps for 360° rotation: ");
      Serial.println(totalSteps);
    }
  }
}
