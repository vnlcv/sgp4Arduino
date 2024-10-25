#include "Arduino_BMI270_BMM150.h"
#include "MadgwickAHRS.h"
#include "Arduino_APDS9960.h"
#include "Arduino_HS300x.h"

// initialize a Madgwick filter
Madgwick filter;
// Sensor sample rate is fixed at 104 Hz;
const float sensorRate = 104.00;

const int stepPin1 = 2;  // PUL -Pulse
const int dirPin1 = 3;   // DIR -Direction
const int enPin1 = 4;    // ENA -Enable
const int microstep = 4;
const int pulse_rev = 200 * microstep;  // steps for one full revolution
const int magstep = (700 / microstep);

int totalSteps = 0;      // Keep track of steps during 360° rotation
bool homingComplete = false;  // Homing flag
bool countingSteps = false;   // Flag to track step counting
int proximity = 1000;

void setup() {
  Serial.begin(9600);
  initialisation();
}

void loop() {
  homing();
  // If the motor is homed, start rotating and counting steps
  if (homingComplete && countingSteps) {
    calibration();
  }
  float temperature = HS300x.readTemperature();
  float humidity = HS300x.readHumidity();


  // Print sensor and motor data
  // Serial.print("Temperature: ");
  // Serial.print(temperature);
  // Serial.print(" °C");
  // Serial.print(" | ");
  // Serial.print("Humidity: ");
  // Serial.print(humidity);
  // Serial.println(" %");
}

void initialisation() {
  // Initialize motor pins
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, HIGH);  // motor disabled initially
  
  // Initialize sensors
  if (!APDS.begin()) {
    Serial.println("Error initializing APDS-9960 sensor!");
  }

  if (!HS300x.begin()) {
    Serial.println("Failed to initialize humidity temperature sensor!");
    while (1);
  }
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  filter.begin(sensorRate);
}

// Function to rotate the motor by a number of steps
void rotateMotor(int motorSteps) {  // Specify the number of steps to rotate
  digitalWrite(enPin1, LOW);  // Enable motor
  digitalWrite(dirPin1, HIGH);  // Set direction
  
  for (int x = 0; x < motorSteps; x++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(magstep);
    
    // Count the steps if we're measuring a full rotation
    if (countingSteps) {
      totalSteps++;
    }
  }
  
  delay(100);  
}

void homing() {
  // Homing process to detect proximity and stop motor
  if (!homingComplete) {
    Serial.println("Homing starting.");
    
    proximity = APDS.readProximity();  
    Serial.print("Proximity: ");
    Serial.print(proximity);
    // If proximity is between 0 and 10, stop motor and mark it as homed
    if (proximity >= 0 && proximity <= 10) {
      homingComplete = true;  // Homing complete
      Serial.println("Homing complete, starting 360° measurement");
      countingSteps = true;  // Start counting steps
      totalSteps = 0;        // Reset step count
    } else {
      // Keep rotating motor slowly for homing
      rotateMotor(1);  // Rotate 1 step at a time for homing
    }
    
  }
}

void calibration() {
  Serial.println("Calibration starting.");
  rotateMotor(1);  // Rotate one step at a time

  // Only check the proximity sensor after 10 steps
  if (totalSteps >= 10) {
    if (APDS.proximityAvailable()) {
      proximity = APDS.readProximity();  
      Serial.print("Proximity: ");
      Serial.print(proximity);
      // If proximity between 0 and 10 is detected, we've completed a full revolution
      if (proximity >= 0 && proximity <= 10) {
        countingSteps = false;  // Stop counting steps
        digitalWrite(enPin1, HIGH);  // Disable motor
        Serial.println("Full revolution complete");
        
        Serial.print("Total steps for 360° rotation: ");
        Serial.println(totalSteps);
      }
    }
  }
}
