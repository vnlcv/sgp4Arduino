#include "Arduino_APDS9960.h"
#include "Arduino_HS300x.h"

const int stepPin1 = 2;  //PUL -Pulse
const int dirPin1 = 3; //DIR -Direction
const int enPin1 = 4;  //ENA -Enable
const int microstep = 4;
const int pulse_rev = 200 * microstep;  // Steps for one full revolution
const int magstep = (700 / microstep); // Delay between pulses

int totalSteps = 0;  
bool homingComplete = false; 
bool countingSteps = false;  
int proximity = 0;

void setup() {
  Serial.begin(9600);
  initializeSensors();
  initializeMotorPins();
}

void loop() {
  float temperature = HS300x.readTemperature();
  float humidity    = HS300x.readHumidity();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C");
  Serial.print(" | ");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");

  // Perform homing and calibration if not complete
  if (!homingComplete) {
    homing();
  } else if (homingComplete && countingSteps) {
    calibration();
  } 
}

// Initializes motor control pins
void initializeMotorPins() {
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enPin1, OUTPUT);
  digitalWrite(enPin1, HIGH);  // Motor disabled initially
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

// Rotates the motor by a number of steps
void rotateMotor(int motorSteps) {  // Specify the number of steps to rotate
  digitalWrite(enPin1, LOW);  // Enable motor
  digitalWrite(dirPin1, HIGH);  // Set direction
  
  for (int x = 0; x < motorSteps; x++) {
    digitalWrite(stepPin1, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPin1, LOW);
    delayMicroseconds(magstep);
    
    // Count the steps 
    if (countingSteps) {
      totalSteps++;
    }
  }
  
  delay(100);  // Short delay between steps for sensor stability
}

// Homing process to find 0 degree notch
void homing(){
  if (APDS.proximityAvailable()) {
      proximity = APDS.readProximity();  // Read proximity value
      Serial.print("Proximity: ");
      Serial.println(proximity);
      
      // If proximity is between 0 and 10, stop motor and mark it as homed
      if (proximity >= 0 && proximity <= 10) {
        homingComplete = true;  // Homing complete
        Serial.println("Homing complete, starting 360° measurement");
        countingSteps = true;  // Start counting steps
        totalSteps = 0;  // Reset step count
      } else {
        rotateMotor(1);  // Rotate 1 step at a time 
      }
    }
}

// Calibration to count steps for a full rotation
void calibration(){
  rotateMotor(1);  // Rotate one step at a time 

  // Read proximity sensor 
  if (APDS.proximityAvailable()) {
    proximity = APDS.readProximity(); 
    Serial.print("Proximity: ");
    Serial.println(proximity); 

    if (totalSteps >= 10) {
      // If proximity between 0 and 10 is detected, full revolution completed
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
