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

int totalSteps = 0;  
bool homingComplete = false; 
bool countingSteps = false; 
int proximity = 0;
int currentPositionBottom = 0;        // Current angle position in steps
int currentPositionTop = 0; 
int targetStepsd1 = 0;            // Target position in steps
int targetStepsd2 = 0;  
enum Motor { BOTTOM, TOP };

// -------------------- Global Objects --------------------
Sgp4 satellite;
SFE_UBLOX_GNSS gnss;

void update_unixtimes(unsigned long milliseconds, unsigned long *unix_before, unsigned long *unix_after) {
  const long dt = (milliseconds / 1000);
  *unix_before = hardcodedUnixTime + dt; 
  *unix_after = next_unixtime + dt;
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
void initializeSatellite();
double printGPSData();
void printSatelliteData();
void checkTrackable();
float calculate_r_ctheta(float h_site, float h_sat, float theta);
XYCoordinates calculateCentre(float r_ctheta, float r_cm, float phi);
AngleResults calculateAngles(float xc, float yc, float r_ctheta, float phi);
AngleResults unixtime_to_angles(double *azi, double *ele, double *range);
AngleResults angles; 
int psi_d1_degrees; 
static unsigned long lastUpdate = 0;
int psi_d2_degrees; 
static double find_milli_dt(const unsigned long mtime, const unsigned long unix_time) {
  const int du = unix_time - hardcodedUnixTime;
  const double dt = mtime - (du*1000.00);
  return dt/1000.00;
}


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

  initializeSatellite();
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
  Serial.print("Unix Before is ");
  Serial.print(unix_before);
  Serial.print(" "); Serial.print(unix_after);

  // Update Satellite Location
  satellite.findsat(unix_before);
  // Update your location
  satellite.site(fallbackLatitude, fallbackLongitude, altitude);
  // Switch satellite if required
  checkTrackable();

  // Find the satellite at the each time 
  double azi_before, ele_before, azi_after, ele_after, range_before, range_after; 
  AngleResults angles_before = unixtime_to_angles( &azi_before, &ele_before, &range_before);

  satellite.findsat(unix_after);
  AngleResults angles_after = unixtime_to_angles(&azi_after, &ele_after, &range_after);

  // Interpolate to provide the right angles
  const double dt = find_milli_dt(t, unix_before); 
  // Serial.print("dt is ");Serial.print(dt);
  const double azi = azi_before + (azi_after - azi_before) * dt;
  const double ele = ele_before + (ele_after - ele_before) * dt;
  const double range = range_before + (range_after - range_before) * dt; 
  angles.psi_d1 = angles_before.psi_d1 + (angles_after.psi_d1-angles_before.psi_d1)*dt; 
  angles.psi_d2 = angles_before.psi_d2 + (angles_after.psi_d2-angles_before.psi_d2)*dt; 
  Serial.print(" Azi "); Serial.print(azi); Serial.print(" Ele "); Serial.print(ele); Serial.print(" Angle Psi_d1 "); 
  Serial.print(angles.psi_d1);
  Serial.print(" Angle Psi_d2 "); 
  Serial.println(angles.psi_d2);
  // // timer.update();
  // if (unixTime > 0) { // Proceed only if time is valid
  //   satellite.findsat(unixTime); // Updates satellite properties based on unixTime
  //   frameRate++; // Increment frame rate counter
  // } else if (millis() - startTime > FIX_TIMEOUT) {
  //   // Serial.println("Timeout: Proceeding with estimated time.");
  //   unsigned long estimatedTime = millis() / TIMER_INTERVAL_MS; // Estimate time in seconds
  //   satellite.findsat(estimatedTime);
  // }

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
    // Control loop to move to target angle 
       
    psi_d1_degrees = angles.psi_d1;  // Change target angle
      // Serial.print("New psi_d1: ");
      // Serial.println(psi_d1_degrees);

    psi_d2_degrees = angles.psi_d2 ;  // Change target angle
      // Serial.print("New psi_d2: ");
      // Serial.println(psi_d2_degrees);
      
    // Calculate target steps from current position
    targetStepsd1 = angleToSteps(psi_d1_degrees);
    Serial.print("Target Steps ");
    Serial.print(targetStepsd1);
    rotateToAngle(targetStepsd1, BOTTOM);
    Serial.print(" ");
    targetStepsd2 = angleToSteps(psi_d2_degrees);
    Serial.println(targetStepsd2);
    rotateToAngle(targetStepsd2, TOP);
    // if (millis() - lastUpdate >= 100) {  // Update every TIMER_INTERVAL_MS
    //   lastUpdate = millis();
    //   Serial.print(azi);Serial.print(",");Serial.print(ele);Serial.print(",");
    //   Serial.print(angles.psi_d1);Serial.print(",");Serial.println( angles.psi_d2);
    // }
  }
}

// -------------------- Function Implementations --------------------


AngleResults unixtime_to_angles(double *azi, double *ele, double *range) {
  // Get Satellite parameters
  float h_site = altitude; 
  float h_sat = satellite.satAlt;
  float theta = satellite.satEl;
  float phi = satellite.satAz;
  *range = satellite.satDist;
  *ele = theta; 
  *azi = phi;
  float theta_min = TRACKABLE_ELEVATION;

  r_ctheta = calculate_r_ctheta(h_site, h_sat, theta);
  r_cm = calculate_r_ctheta(h_site, h_sat, theta_min);
  XYCoordinates coords = calculateCentre(r_ctheta, r_cm, phi);
  return calculateAngles(coords.xc, coords.yc, r_ctheta, phi);
}



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

// Initialize Satellite Object
void initializeSatellite() {
  // Initialize satellite with hardcoded TLE data
  if (!satellite.init(satelliteName, tleLine1, tleLine2)) {
    // Serial.println("ERROR: Failed to initialize satellite parameters.");
    while (1);
  }

  double jdEpoch = satellite.satrec.jdsatepoch;
  invjday(jdEpoch, timezoneOffset, true, year, month, day, hour, minute, secondDouble);
  

  char buffer[100];
  // snprintf(buffer, sizeof(buffer), "Epoch: %02d/%02d/%04d %02d:%02d:%.2f\n", day, month, year, hour, minute, secondDouble);
  // Serial.print(buffer);
}


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

// Converts angle in degrees to steps based on 0-degree reference
int angleToSteps(const double angle) {
  const double angle_ratio = angle / 360.00; 
  Serial.print("Angle Ratio is "); Serial.print(angle_ratio, 5);
  const int r = angle_ratio * totalSteps; 
  Serial.print(" r is "); Serial.println(r, 5);
  return r;
}

// Rotates motor to the desired angle
void rotateToAngle(int targetSteps, Motor motor) {
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
  Serial.print("Commanding to move ");
  Serial.println(motorSteps);
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
          totalSteps = 0;  // Reset step count
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
    totalSteps++;
   
    // Read proximity sensor 
    if (APDS.proximityAvailable()) {
      proximity = APDS.readProximity(); 
      // Serial.print("Proximity: ");
      // Serial.println(proximity); 

      // If proximity between 0 and 10 is detected, full revolution completed
      if (totalSteps >= 3980 && proximity == 0) {
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

// Function to switch to TLE for ONEWEB-0150
void switchTLE_OneWeb0150() {
  if (!satellite.init("ONEWEB-0150", tleLine1_OneWeb0150, tleLine2_OneWeb0150)) {
    // Serial.println("ERROR: Failed to initialize ONEWEB-0150 satellite parameters.");
    while (1);  // Halt if initialization fails
  }
  // Serial.println("Switched to satellite ONEWEB-0150.");
  // Serial.println("TLE Line 1: ");
  // Serial.println(tleLine1_OneWeb0150);
  // Serial.println("TLE Line 2: ");
  // Serial.println(tleLine2_OneWeb0150);
}

// Function to switch to TLE for ONEWEB-0107
void switchTLE_OneWeb0107() {
  if (!satellite.init("ONEWEB-0107", tleLine1_OneWeb0107, tleLine2_OneWeb0107)) {
    // Serial.println("ERROR: Failed to initialize ONEWEB-0107 satellite parameters.");
    while (1);  // Halt if initialization fails
  }
  // Serial.println("Switched to satellite ONEWEB-0107.");
}

// Timer Callback Function - Executes Every Second
void onSecondTick() {
  frameRate = 0; // Reset frame rate counter7
  // Serial.print("millis is ");
  // Serial.println(millis());

  printGPSData();
  invjday(satellite.satJd, timezoneOffset, true, year, month, day, hour, minute, secondDouble);

  char buffer[100];
  // snprintf(buffer, sizeof(buffer), "Satellite Time: %02d/%02d/%04d %02d:%02d:%.2f\n", day, month, year, hour, minute, secondDouble);
  // Serial.print(buffer);

  printSatelliteData();
  checkTrackable();
  // Serial.println();

  float h_site = altitude; 
  float h_sat = satellite.satAlt;
  float theta = satellite.satEl;
  float phi = satellite.satAz;
  float theta_min = TRACKABLE_ELEVATION;

  r_ctheta = calculate_r_ctheta(h_site, h_sat, theta);
  r_cm = calculate_r_ctheta(h_site, h_sat, theta_min);
  XYCoordinates coords = calculateCentre(r_ctheta, r_cm, phi);
  angles = calculateAngles(coords.xc, coords.yc, r_ctheta, phi);
  
  if (stopExecution) {
    // Serial.println("Execution halted.");
    while (1);  
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

// Print Satellite Information
void printSatelliteData() {
  // char buffer[150];
  // snprintf(buffer, sizeof(buffer), "Azimuth: %.2f°, Elevation: %.2f°, Distance: %.2f km", satellite.satAz, satellite.satEl, satellite.satDist);
  // Serial.println(buffer);

  // Send azimuth and elevation to GUI
  Serial.print(satellite.satAz);
  Serial.print(",");              
  Serial.println(satellite.satEl); 

  // snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f km", satellite.satLat, satellite.satLon, satellite.satAlt);
  // Serial.println(buffer);

  // const char* visibility;
  // switch (satellite.satVis) {
  //   case -2: visibility = "Under Horizon"; break;
  //   case -1: visibility = "Daylight"; break;
  //   default: snprintf(buffer, sizeof(buffer), "Visibility: %d", satellite.satVis); visibility = buffer; break;
  // }
  // Serial.println(visibility);
  
  // snprintf(buffer, sizeof(buffer), "Frame Rate: %d calculations/sec", frameRate);
  // Serial.println(buffer);
}

// Check if Satellite is Trackable (Elevation > TRACKABLE_ELEVATION)
void checkTrackable() {
  if (isFirstRead) {
    isFirstRead = false; 
    return; 
  }
  char buffer[100];
  // snprintf(buffer, sizeof(buffer), "Status: Satellite elevation is %s %.0f degrees.", (satellite.satEl > TRACKABLE_ELEVATION) ? "above" : "below", TRACKABLE_ELEVATION);
  // Serial.println(buffer);
  if (satellite.satEl < TRACKABLE_ELEVATION + 0.5) {
    if (strcmp(satellite.satName, "ONEWEB-0352") == 0) {
      switchTLE_OneWeb0150(); // Switch to ONEWEB-0150 if below 25 degrees for ONEWEB-0352
    } else if (strcmp(satellite.satName, "ONEWEB-0150") == 0) {
      switchTLE_OneWeb0107(); // Switch to ONEWEB-0107 if below 25 degrees for ONEWEB-0150
    } else if (strcmp(satellite.satName, "ONEWEB-0107") == 0) {
      stopExecution = true; // Stop execution if below 25 degrees for ONEWEB-0107
    }
  }
}

float calculate_r_ctheta(float h_site, float h_sat, float theta) {
  float theta_rad = theta * M_PI / 180.0;
  float alpha = asin((R_E_site + h_site) / (R_E_sat + h_sat) * sin(M_PI/2 + theta_rad));
  float beta = M_PI/2 - theta_rad - alpha;
  float r_l = (sin(beta) / sin(alpha)) * (R_E_site + h_site);
  float r_ctheta = r_l * cos(theta_rad);

  // Serial.println("Geometry calculations:");
  // Serial.print("alpha = "); Serial.println(alpha * 180.0 / M_PI);
  // Serial.print("beta = "); Serial.println(beta * 180.0 / M_PI);
  // Serial.print("r_l = "); Serial.println(r_l);
  // Serial.print("r_ctheta = "); Serial.println(r_ctheta);

  return r_ctheta;
}

XYCoordinates calculateCentre(float r_ctheta, float r_cm, float phi) {
  float phi_rad = phi * M_PI / 180.0;
  float eta = acos(r_ctheta / r_cm);
  float xc = (r_cm / 2) * sin(phi_rad - eta);
  float yc = (r_cm / 2) * cos(phi_rad - eta);

  // Serial.print("eta = "); Serial.println(eta * 180.0 / M_PI);
  // Serial.print("xc = "); Serial.println(xc);
  // Serial.print("yc = "); Serial.println(yc);

  return {xc, yc};
}


AngleResults calculateAngles(float xc, float yc, float r_ctheta, float phi) {
    float psi_d1 = atan2(xc, yc);
    float psi_d1_degrees = psi_d1 * 180.0 / M_PI;
    
    // Normalize psi_d1 to be between 0 and 360 degrees
    if (psi_d1_degrees < 0) {
        psi_d1_degrees += 360.0;
    }
    
    // Convert phi to radians
    float phi_rad = phi * M_PI / 180.0;
    
    // Calculate delta_psi
    float delta_psi = atan2((r_ctheta * cos(phi_rad) - yc), (r_ctheta * sin(phi_rad) - xc));
    
    // Convert delta_psi to degrees
    float delta_psi_degrees = delta_psi * 180.0 / M_PI;
    
    // Calculate psi_d2
    float psi_d2_degrees = delta_psi_degrees - psi_d1_degrees;
    
    // Normalize psi_d2 to be between 0 and 360 degrees
    while (psi_d2_degrees < 0) psi_d2_degrees += 360.0;
    while (psi_d2_degrees >= 360.0) psi_d2_degrees -= 360.0;

    // Serial.print("psi_d1: "); Serial.println(psi_d1_degrees);
    // Serial.print("delta_psi: "); Serial.println(delta_psi_degrees);
    // Serial.print("psi_d2: "); Serial.println(psi_d2_degrees);
    // Serial.println();

    return {psi_d1_degrees, psi_d2_degrees};
}
