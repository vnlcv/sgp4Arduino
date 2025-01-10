/*
  This code tracks a satellite using its TLE (Two-Line Element) data with the SGP4 algorithm. 
  It calculates and outputs azimuth and elevation angles every TIMER_INTERVAL_MS,
  indicating when the satellite is trackable (above 25 degrees).
  Required disc positional angles are calculated and motors rotate to achieve these angles.

  GPS module is setup and uses fallback coordinates and Unix time if no GPS fix is available. 
  This ensures continuous operation even without a GPS signal.

  Hardware Connections:
  ---------------------
  Adafruit 254 MicroSD Card Breakout:
    - 3.3V  -> 3.3V on Arduino
    - GND -> GND on Arduino
    - CLK -> D13
    - DO   -> D12
    - DI   -> D11
    - CS   -> D10

  SparkFun GPS Breakout - NEO-M9N (Qwiic):
    - 3.3V  -> 3.3V on Arduino
    - GND -> GND on Arduino
    - SDA -> A4
    - SCL -> A5

  Right Motor (Top Disc):
    - Pulse  -> D2 (purple)
    - Direction -> D3 (orange)
    - Enable -> D4 (yellow)

  Left Motor (Bottom Disc):
    - Pulse  -> D5 (purple)
    - Direction -> D6 (orange)
    - Enable -> D7 (yellow)

*/

#include <Sgp4.h>
// #include <TickTwo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
// #include <SD.h>
// #include <SPI.h>
#include <math.h>
#include "Arduino_APDS9960.h"
#include "Arduino_HS300x.h"

// -------------------- Constants and Definitions --------------------
const int scale = 1; 
// const int SD_CS_PIN = 10;                     // Chip Select pin for SD card
const unsigned long TIMER_INTERVAL_MS = 5; // Timer interval in milliseconds
const double TRACKABLE_ELEVATION = 25.0;      // Elevation threshold in degrees
// const char* TLE_FILE_NAME = "tle.txt";        // TLE data file name
const unsigned long FIX_TIMEOUT = 3600000;    // 1 hour in milliseconds
const float R_E_site = 6371.0; // Earth radius at observer's location in km
const float R_E_sat = 6371.0; // Earth radius at observer's location in km
const double fallbackLatitude = 51.5752;      // Latitude in degrees
const double fallbackLongitude = -1.3150;     // Longitude in degrees
float altitude = 182.21;                      // Fallback Altitude in meters
static unsigned long hardcodedUnixTime = 1731334682; // Fallback Unix time
unsigned long next_unixtime = hardcodedUnixTime + 1;

// I2C parameters
#define SLAVE_ADDRESS 8

// Hardcoded TLE data
const char* satelliteName = "ONEWEB-0352";
char tleLine1[] = "1 49216U 21083AG  24316.03750046 -.00000056  00000+0 -18780-3 0  9996";
char tleLine2[] = "2 49216  87.8915  76.2498 0001493  83.6536 276.4763 13.13470652155778";

char tleLine1_OneWeb0150[] = "1 48047U 21025F   24316.03908473  .00000152  00000+0  38222-3 0  9990";
char tleLine2_OneWeb0150[] = "2 48047  87.8909  76.2072 0001561 105.6218 254.5084 13.13470682176991";

char tleLine1_OneWeb0107[] = "1 48048U 21025G   24316.04068779  .00000004  00000+0 -23715-4 0  9997";
char tleLine2_OneWeb0107[] = "2 48048  87.8907  76.2047 0001234  98.9649 261.1620 13.13471972176972";

bool stopExecution = false; // Flag to stop execution when needed
bool isFirstRead = true; //Skip first elevation and azimuth angle

// Motor pin definitions
const int stepPinTop = 2;  //PUL -Pulse
const int dirPinTop = 3; //DIR -Direction
const int enPinTop = 4;  //ENA -Enable
const int stepPinBottom = 5;  //PUL -Pulse
const int dirPinBottom = 6; //DIR -Direction
const int enPinBottom = 7;  //ENA -Enable
const int microstep = 4;
const int pulse_rev = 800;  // Steps for one full revolution
const int magstep = (600 / microstep); // Delay between pulses

int totalStepsd1 = 0;  
int totalStepsd2 = 0; 
bool homingComplete = false; 
bool countingSteps = false; 
int proximity = 0;
int currentPositionBottom = 0;        // Current angle position in steps
int currentPositionTop = 0; 
// int targetStepsd1 = 0;            // Target position in steps
// int targetStepsd2 = 0;  
enum Motor { BOTTOM, TOP };

// -------------------- Global Objects --------------------
Sgp4 satellite;
SFE_UBLOX_GNSS gnss;

void update_unixtimes(unsigned long milliseconds, unsigned long *unix_before, unsigned long *unix_after) {
  const long dt = (milliseconds / 1000);
  *unix_before = hardcodedUnixTime + dt; 
  *unix_after = next_unixtime + dt;
}

// I2C Functions

void read_azi_ele_range(double *azi, double *ele, double *range) {
  int32_t az_int, el_int; 
  byte a1, b1, c1, d1, a2, b2, c2, d2; 
  a1 = Wire.read();
  b1 = Wire.read();
  c1 = Wire.read();
  d1 = Wire.read();
  az_int = a1;
  az_int = (az_int << 8) | b1;
  az_int = (az_int << 8) | c1;
  az_int = (az_int << 8) | d1;
  *azi = az_int / 1e6;
  a2 = Wire.read();
  b2 = Wire.read();
  c2 = Wire.read();
  d2 = Wire.read();
  el_int = a2;
  el_int = (el_int << 8) | b2;
  el_int = (el_int << 8) | c2;
  el_int = (el_int << 8) | d2;
  *ele = el_int / 1e6;
  uint32_t bigNum;
  byte a,b,c,d;
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();
  bigNum = a;
  bigNum = (bigNum << 8) | b;
  bigNum = (bigNum << 8) | c;
  bigNum = (bigNum << 8) | d;
  *range = bigNum / 1e2; 
} 

void write_unsigned_long(const unsigned long unix) {
  uint8_t a = (unix >> 24) & 0xFF ; 
  Wire.write(a);
  uint8_t b = (unix >> 16) & 0xFF;
  Wire.write(b);
  uint8_t c = (unix >> 8) & 0xFF;
  Wire.write(c);
  uint8_t d = unix & 0xFF; 
  Wire.write(d);
}

void send_unix_to_modem(const unsigned long t) {
  // Update the unix times 
  unsigned long unix_before = 1;
  unsigned long unix_after = 1; 
  update_unixtimes(t, &unix_before, &unix_after);
  // Send via I2C
  Wire.beginTransmission(SLAVE_ADDRESS);
  write_unsigned_long(unix_before);
  
  // Wire.write((unix_before >> 24) & 0xFF); // Send the highest byte
  // Wire.write((unix_before >> 16) & 0xFF); // Send the second byte
  // Wire.write((unix_before >> 8) & 0xFF);  // Send the third byte
  // Wire.write(unix_before & 0xFF);         // Send the lowest byte
  write_unsigned_long(unix_after);
  // Wire.write((unix_after >> 24) & 0xFF); // Send the highest byte
  // Wire.write((unix_after >> 16) & 0xFF); // Send the second byte
  // Wire.write((unix_after >> 8) & 0xFF);  // Send the third byte
  // Wire.write(unix_after & 0xFF);         // Send the lowest byte
  write_unsigned_long(t);
  // Wire.write((t >> 24) & 0xFF); // Send the highest byte
  // Wire.write((t >> 16) & 0xFF); // Send the second byte
  // Wire.write((t >> 8) & 0xFF);  // Send the third byte
  // Wire.write(t & 0xFF);         // Send the lowest byte
}

void read_target(uint16_t *target) {
  byte a, b; 
  uint16_t bigNum; 
  a = Wire.read(); 
  b = Wire.read(); 
  bigNum = a;
  bigNum = (bigNum << 8) | b; 
  *target = bigNum;
}

bool request_data_from_modem(uint16_t *targetStepsd1, uint16_t *targetStepsd2,  double *azi, double *ele, double *range) {
  Wire.requestFrom(SLAVE_ADDRESS,16); // Request 4 bytes back from slave
  delay(20);
  // Serial.print("Wire Available is ");
  // Serial.println(Wire.available());
  if(Wire.available() == 16) {
    // Read target Steps
    // uint16_t targetd1, targetd2; 
    read_target(targetStepsd1);
    read_target(targetStepsd2);
    read_azi_ele_range(azi, ele, range);
    if (*targetStepsd1 < 5000 && *targetStepsd2 < 5000) {
      return 1; 
    }
  }
  return 0; 
}


// void onSecondTick();
// // Initialize TickTwo with the callback, interval, repeat count, and resolution
// TickTwo timer(onSecondTick, TIMER_INTERVAL_MS, 0, MILLIS);

// -------------------- Global Variables --------------------
unsigned long unixTime = 0;
int timezoneOffset = 0; // UTC
int frameRate = 0;
int year, month, day, hour, minute;
double secondDouble;
float r_ctheta, r_cm;

// Timeout Tracking
unsigned long startTime;

struct XYCoordinates {
  float xc;
  float yc;
};

struct AngleResults {
    float psi_d1;
    float psi_d2;
};

// -------------------- Function Prototypes --------------------
void setupGPS();
bool initializeSDCard();
bool loadTLEFromSD();
double printGPSData();
static unsigned long lastUpdate = 0; 

// -------------------- Setup Function --------------------
void setup() {
  // Serial.begin(38400);
  Serial.begin(9600); 
  while (!Serial); // Wait for Serial Monitor to connect

  Wire.begin();
  // Serial.println("Start");
  // Serial.println("\n--- Satellite Tracking Initialization ---");
  startTime = millis(); // Initialize start time
  // setupGPS();

  // if (!initializeSDCard()) {
  //   Serial.println("SD Card initialization failed. Halting execution.");
  //   while (1);
  // }

  // if (!loadTLEFromSD()) {
  //   Serial.println("Failed to load TLE data. Halting execution.");
  //   while (1);
  // }
  initializeSensors();
  initializeMotorPins();
  // Give it a satellite site
  // satellite.site(fallbackLatitude, fallbackLongitude, altitude); 
  // timer.start(); // Start timer 
  // Serial.println("--- Initialization Complete ---\n");
}

// -------------------- Main Loop --------------------
void loop() {
  // Get current milliseconds
  const unsigned long t = scale * millis();
  unsigned long unix_before, unix_after; 
  update_unixtimes(t, &unix_before, &unix_after);

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
    // Write to I2C
    send_unix_to_modem(t);
    // Send total steps as an integer 
    Wire.write((totalStepsd1 >> 8) & 0xFF);  // Send the third byte
    Wire.write(totalStepsd1 & 0xFF);         // Send the lowest byte

    Wire.write((totalStepsd2 >> 8) & 0xFF);  // Send the third byte
    Wire.write(totalStepsd2 & 0xFF);         // Send the lowest byte
    Wire.endTransmission(); // End transmission

    // Read
    double azi, ele, range; 
    uint16_t targetStepsd1, targetStepsd2;
    if (request_data_from_modem(&targetStepsd1, &targetStepsd2, &azi, &ele, &range)){
      // Cast to integers
      int target_d1 = (int)targetStepsd1; 
      rotateToAngle(target_d1, BOTTOM, totalStepsd1);
      int target_d2 = (int)targetStepsd2; 
      rotateToAngle(target_d2, TOP, totalStepsd2);
      if (millis() - lastUpdate > 100) {
        lastUpdate = millis();
        Serial.print(azi); Serial.print(","); Serial.print(ele); 
        Serial.print(","); Serial.print(range);
        Serial.print(","); 
        Serial.print(target_d1);
        Serial.print(","); 
        Serial.println(target_d2);
      }
    }
  }
}

// -------------------- Function Implementations --------------------

// Initialize GPS Module
void setupGPS() {
  // Serial.println("Initializing GPS Module...");
  if (!gnss.begin()) {
    // Serial.println(F("u-blox GNSS not detected. Please check wiring. Halting execution."));
    while (1);
  }
  gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only
  // Serial.println("GPS module initialized successfully.");
}

// // Initialize SD Card
// bool initializeSDCard() {
//   Serial.print("Initializing SD card...");
//   if (!SD.begin(SD_CS_PIN)) {
//     Serial.println(" Initialization failed!");
//     return false;
//   }
//   Serial.println(" Initialization done.");
//   return true;
// }

// // Load TLE Data from SD Card
// bool loadTLEFromSD() {
//   File tleFile = SD.open(TLE_FILE_NAME);
//   if (!tleFile) {
//     Serial.println("Error opening TLE file.");
//     return false;
//   }

//   String satName = tleFile.readStringUntil('\n');
//   satName.trim(); // Trim trailing whitespace or newlines
//   String tleLine1 = tleFile.readStringUntil('\n');
//   tleLine1.trim();
//   String tleLine2 = tleFile.readStringUntil('\n');
//   tleLine2.trim();

//   tleFile.close();

//   Serial.println("Satellite Name: " + satName);
//   Serial.println("TLE Line 1: " + tleLine1);
//   Serial.println("TLE Line 2: " + tleLine2);

//   // Convert TLE lines to C-style strings
//   char tleLine1Char[130], tleLine2Char[130];
//   tleLine1.toCharArray(tleLine1Char, sizeof(tleLine1Char));
//   tleLine2.toCharArray(tleLine2Char, sizeof(tleLine2Char));

//   initializeSatellite(satName, tleLine1Char, tleLine2Char);
//   return true;
// }

// Initializes motor control pins
void initializeMotorPins() {
  pinMode(stepPinTop, OUTPUT);
  pinMode(dirPinTop, OUTPUT);
  pinMode(enPinTop, OUTPUT);
  digitalWrite(enPinTop, LOW);  // Enable motor

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

// Rotates motor to the desired angle
void rotateToAngle(int targetSteps, Motor motor, int totalSteps) {
  int currentPosition = (motor == BOTTOM) ? currentPositionBottom : currentPositionTop;
  int stepDifference = targetSteps - currentPosition;
  int direction = (stepDifference >= 0) ? 1 : -1;   // If stepdifference >= 0, direction = 1, clockwise. If stepdifference < 0, direction = -1, anticlockwise. 

  // Serial.println("targetSteps");
  // Serial.println(targetSteps);
  // Serial.println("currentPosition:");
  // Serial.println(currentPosition);
  // Serial.println("stepDifference");
  // Serial.println(stepDifference);

  // Calculate the shortest path
  int stepsToMove = abs(stepDifference);
  if (stepsToMove > totalSteps / 2) { // If longer path
    stepsToMove = totalSteps - stepsToMove; // Steps in opposite direction
    direction *= -1; // Reverse direction eg 1 to -1
  }

  // Rotate the required steps
  rotateMotor(stepsToMove, motor, direction);

  // Update current position within totalSteps for one disc revolution 
  if (motor == BOTTOM) {
    currentPositionBottom = (currentPosition + stepDifference + totalSteps) % totalSteps;
  } else if (motor == TOP) {
    currentPositionTop = (currentPosition + stepDifference + totalSteps) % totalSteps;
  }
}

// Rotates the motor by a number of steps
void rotateMotor(int motorSteps, Motor motor, int direction) {  // Specify the number of steps to rotate
  int dirPin = (motor == BOTTOM) ? dirPinBottom : dirPinTop;
  int stepPin = (motor == BOTTOM) ? stepPinBottom : stepPinTop;

  // Set direction based on the input
  digitalWrite(dirPin, direction == 1 ? HIGH : LOW); 

  // digitalWrite(enPinBottom, LOW);  // Enable motor
  // digitalWrite(dirPin, HIGH);  // Set direction
  // Serial.print("Commanding to move ");
  // Serial.println(motorSteps);
  for (int x = 0; x < motorSteps; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(magstep);
    // delay(10);
  }
}

// Homing process to find 0 degree notch
void homing(){
  digitalWrite(dirPinBottom, HIGH);  // Set direction
  
  while (!homingComplete) {
    digitalWrite(stepPinBottom, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPinBottom, LOW);
    delayMicroseconds(magstep);

    digitalWrite(stepPinTop, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPinTop, LOW);
    delayMicroseconds(magstep);

    // Read proximity sensor 
    if (APDS.proximityAvailable()) {
      proximity = APDS.readProximity(); 
      // Serial.print("Proximity: ");
      // Serial.println(proximity); 

      // If proximity between 0 and 10 is detected, 0 degree notch detected
      if (proximity == 0) {
          homingComplete = true;  // Homing complete
          // Serial.println("Homing complete.");
          countingSteps = true;  // Start counting steps
          totalStepsd1 = 0;  // Reset step count
          totalStepsd2 = 0;
          currentPositionBottom = 0; // Set current position as 0 degree reference
          currentPositionTop = 0;
          break;
      }
    }
  }
}

// Calibration to count steps for a full rotation
void calibration(){
  digitalWrite(dirPinBottom, HIGH);  // Set direction
  
  while (countingSteps) {
    digitalWrite(stepPinBottom, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPinBottom, LOW);
    delayMicroseconds(magstep);

    digitalWrite(stepPinTop, HIGH);
    delayMicroseconds(magstep);
    digitalWrite(stepPinTop, LOW);
    delayMicroseconds(magstep);
    totalStepsd1++;
    totalStepsd2++; 
   
    // Read proximity sensor 
    if (APDS.proximityAvailable()) {
      proximity = APDS.readProximity(); 
      // Serial.print("Proximity: ");
      // Serial.println(proximity); 

      // If proximity between 0 and 10 is detected, full revolution completed
      if (totalStepsd1 >= 3980 && proximity == 0) {
        countingSteps = false;  // Stop counting steps
        // digitalWrite(enPinBottom, HIGH);  // Disable motor
        // Serial.println("Full revolution complete");
        // Serial.print("Total steps for 360° rotation: ");
        // Serial.println(totalSteps);
        break;
      }
    }
  }
}

// Print GPS Information
double printGPSData() {
  if (gnss.getGnssFixOk()) {
    unixTime = gnss.getUnixEpoch();
    double latitude = gnss.getLatitude() / 1e7;
    double longitude = gnss.getLongitude() / 1e7;
    altitude = gnss.getAltitude() / 1e3; 

    satellite.site(latitude, longitude, altitude);

    char buffer[100];
    // snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f m, SIV: %d", latitude, longitude, altitude, gnss.getSIV());
    // Serial.println(buffer);

    // snprintf(buffer, sizeof(buffer), "GPS Time: %lu %04d-%02d-%02d %02d:%02d:%.2f\n", gnss.getUnixEpoch(), gnss.getYear(), gnss.getMonth(), gnss.getDay(), gnss.getHour(), gnss.getMinute(), gnss.getSecond());
    // Serial.print(buffer);
  } else {
    // Serial.println("GPS Status: Waiting for fix...");

    // Use fallback coordinates and unix time
    satellite.site(fallbackLatitude, fallbackLongitude, altitude); 
    // Serial.println("Using fallback coordinates:");
    char buffer[100];
    // snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f m", fallbackLatitude, fallbackLongitude, altitude);
    // Serial.println(buffer);
    unixTime = hardcodedUnixTime; // Assign the hardcoded unix time
    hardcodedUnixTime += TIMER_INTERVAL_MS / TIMER_INTERVAL_MS; // Increment the hardcoded time by 1 second
    // snprintf(buffer, sizeof(buffer), "Using hardcoded Unix Time: %lu (25/10/2024 12:00 PM UTC)", unixTime);
    // Serial.println(buffer);
  }
  return altitude;
}
