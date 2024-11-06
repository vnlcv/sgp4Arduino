/*
  Satellite Tracking 

  This code tracks a satellite using its TLE (Two-Line Element) data with the SGP4 algorithm
  and TickTwo library. It calculates and outputs azimuth and elevation angles every second,
  indicating when the satellite is trackable (above 25 degrees).

  Hardware Connections:
  ---------------------
  Adafruit 254 MicroSD Card Breakout:
    - 5V  -> 5V on Arduino
    - GND -> GND on Arduino
    - CLK -> D13
    - DO   -> D12
    - DI   -> D11
    - CS   -> D10

  SparkFun GPS Breakout - NEO-M9N (Qwiic):
    - 5V  -> 5V on Arduino
    - GND -> GND on Arduino
    - SDA -> A4
    - SCL -> A5
*/

#include <Sgp4.h>
#include <TickTwo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Wire.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>

// -------------------- Constants and Definitions --------------------
const int SD_CS_PIN = 10;                     // Chip Select pin for SD card
const unsigned long TIMER_INTERVAL_MS = 1000; // Timer interval in milliseconds
const double TRACKABLE_ELEVATION = 25.0;      // Elevation threshold in degrees
const char* TLE_FILE_NAME = "tle.txt";        // TLE data file name
const unsigned long FIX_TIMEOUT = 3600000;    // 1 hour in milliseconds
const float R_E_site = 6371.0; // Earth radius at observer's location in km
const float R_E_sat = 6371.0;  // Earth radius at satellite's location in km

// -------------------- Global Objects --------------------
Sgp4 satellite;
SFE_UBLOX_GNSS gnss;

void onSecondTick();
// Initialize TickTwo with the callback, interval, repeat count, and resolution
TickTwo timer(onSecondTick, TIMER_INTERVAL_MS, 0, MILLIS);

// -------------------- Global Variables --------------------
unsigned long unixTime = 0;
int timezoneOffset = 0; // UTC
int frameRate = 0;
int year, month, day, hour, minute;
double secondDouble;
float r_ctheta, r_cm, altitude;

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

// -------------------- Setup Function --------------------
void setup() {
  Serial.begin(38400); 
  while (!Serial); // Wait for Serial Monitor to connect

  Wire.begin();
  Serial.println("\n--- Satellite Tracking Initialization ---");

  startTime = millis(); // Initialize start time

  setupGPS();

  // if (!initializeSDCard()) {
  //   Serial.println("SD Card initialization failed. Halting execution.");
  //   while (1);
  // }

  // if (!loadTLEFromSD()) {
  //   Serial.println("Failed to load TLE data. Halting execution.");
  //   while (1);
  // }

  initializeSatellite();

  timer.start(); // Start timer 

  Serial.println("--- Initialization Complete ---\n");
}

// -------------------- Main Loop --------------------
void loop() {
  timer.update();
  
  if (unixTime > 0) { // Proceed only if time is valid
    satellite.findsat(unixTime); // Updates satellite properties based on unixTime
    frameRate++; // Increment frame rate counter
  } else if (millis() - startTime > FIX_TIMEOUT) {
    Serial.println("Timeout: Proceeding with estimated time.");
    unsigned long estimatedTime = millis() / 1000; // Estimate time in seconds
    satellite.findsat(estimatedTime);
  }
}

// -------------------- Function Implementations --------------------

// Initialize GPS Module
void setupGPS() {
  Serial.println("Initializing GPS Module...");
  if (!gnss.begin()) {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Halting execution."));
    while (1);
  }
  gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only
  Serial.println("GPS module initialized successfully.");
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
  // Hardcoded TLE data for the ISS (ZARYA)
  const char* satelliteName = "ONEWEB-0208";
  char tleLine1[] = "1 48243U 21031AK  24298.74286871  .00000032  00000+0  51320-4 0  9995";
  char tleLine2[] = "2 48243  87.9019  49.0995 0001636  94.3263 265.8054 13.14505755169424";

  // Initialize satellite with hardcoded TLE data
  if (!satellite.init(satelliteName, tleLine1, tleLine2)) {
    Serial.println("ERROR: Failed to initialize satellite parameters.");
    while (1);
  }

  double jdEpoch = satellite.satrec.jdsatepoch;
  invjday(jdEpoch, timezoneOffset, true, year, month, day, hour, minute, secondDouble);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Epoch: %02d/%02d/%04d %02d:%02d:%.2f\n", day, month, year, hour, minute, secondDouble);
  Serial.print(buffer);
}

// Timer Callback Function - Executes Every Second
void onSecondTick() {
  frameRate = 0; // Reset frame rate counter

  printGPSData();
  invjday(satellite.satJd, timezoneOffset, true, year, month, day, hour, minute, secondDouble);

  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Satellite Time: %02d/%02d/%04d %02d:%02d:%.2f\n", day, month, year, hour, minute, secondDouble);
  Serial.print(buffer);

  printSatelliteData();
  checkTrackable();
  Serial.println();

  float h_site = altitude; 
  float h_sat = satellite.satAlt;
  float theta = 30; //satellite.satEl
  float phi = satellite.satAz;
  float theta_min = TRACKABLE_ELEVATION;

  r_ctheta = calculate_r_ctheta(h_site, h_sat, theta);
  r_cm = calculate_r_ctheta(h_site, h_sat, theta_min);
  XYCoordinates coords = calculateCentre(r_ctheta, r_cm, phi);
  AngleResults angles = calculateAngles(coords.xc, coords.yc, r_ctheta, phi);
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
    snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f m, SIV: %d", latitude, longitude, altitude, gnss.getSIV());
    Serial.println(buffer);

    snprintf(buffer, sizeof(buffer), "GPS Time: %lu %04d-%02d-%02d %02d:%02d:%.2f\n", gnss.getUnixEpoch(), gnss.getYear(), gnss.getMonth(), gnss.getDay(), gnss.getHour(), gnss.getMinute(), gnss.getSecond());
    Serial.print(buffer);
  } else {
    Serial.println("GPS Status: Waiting for fix...");

    // Set hardcoded coordinates when GPS is not available
    double fallbackLatitude = 51.5752;   // Hardcoded Latitude
    double fallbackLongitude = -1.3150;   // Hardcoded Longitude
    altitude = 182.21;                    // Hardcoded altitude in meters

    satellite.site(fallbackLatitude, fallbackLongitude, altitude); // Use fallback coordinates

    Serial.println("Using fallback coordinates:");
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f m", fallbackLatitude, fallbackLongitude, altitude);
    Serial.println(buffer);
    
    // Use a hardcoded Unix Time when GPS is not available and increment it
    static unsigned long hardcodedUnixTime = 1729860291; // Initial hardcoded Unix time
    unixTime = hardcodedUnixTime; // Assign the hardcoded time
    hardcodedUnixTime += TIMER_INTERVAL_MS / 1000; // Increment the hardcoded time by 1 second

    // Print the hardcoded time
    snprintf(buffer, sizeof(buffer), "Using hardcoded Unix Time: %lu (25/10/2024 12:00 PM UTC)", unixTime);
    Serial.println(buffer);
  }

  return altitude;
}

// Print Satellite Information
// Print Satellite Information
void printSatelliteData() {
  char buffer[150];
  snprintf(buffer, sizeof(buffer), "Azimuth: %.2f°, Elevation: %.2f°, Distance: %.2f km", satellite.satAz, satellite.satEl, satellite.satDist);
  Serial.println(buffer);

  // Send azimuth and elevation to computer
  Serial.print(satellite.satAz); // Send azimuth
  Serial.print(",");              // Comma as a separator
  Serial.println(satellite.satEl); // Send elevation

  snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f km", satellite.satLat, satellite.satLon, satellite.satAlt);
  Serial.println(buffer);

  const char* visibility;
  switch (satellite.satVis) {
    case -2: visibility = "Under Horizon"; break;
    case -1: visibility = "Daylight"; break;
    default: snprintf(buffer, sizeof(buffer), "Visibility: %d", satellite.satVis); visibility = buffer; break;
  }
  Serial.println(visibility);
  
  snprintf(buffer, sizeof(buffer), "Frame Rate: %d calculations/sec", frameRate);
  Serial.println(buffer);
}

// Check if Satellite is Trackable (Elevation > TRACKABLE_ELEVATION)
void checkTrackable() {
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Status: Satellite elevation is %s %.0f degrees.", (satellite.satEl > TRACKABLE_ELEVATION) ? "above" : "below", TRACKABLE_ELEVATION);
  Serial.println(buffer);
}

float calculate_r_ctheta(float h_site, float h_sat, float theta) {
  float theta_rad = theta * M_PI / 180.0;
  float alpha = asin((R_E_site + h_site) / (R_E_sat + h_sat) * sin(M_PI/2 + theta_rad));
  float beta = M_PI/2 - theta_rad - alpha;
  float r_l = (sin(beta) / sin(alpha)) * (R_E_site + h_site);
  float r_ctheta = r_l * cos(theta_rad);

  Serial.println("Geometry calculations:");
  Serial.print("alpha = "); Serial.println(alpha * 180.0 / M_PI);
  Serial.print("beta = "); Serial.println(beta * 180.0 / M_PI);
  Serial.print("r_l = "); Serial.println(r_l);
  Serial.print("r_ctheta = "); Serial.println(r_ctheta);

  return r_ctheta;
}

XYCoordinates calculateCentre(float r_ctheta, float r_cm, float phi) {
  float phi_rad = phi * M_PI / 180.0;
  float eta = acos(r_ctheta / r_cm);
  float xc = (r_cm / 2) * sin(phi_rad - eta);
  float yc = (r_cm / 2) * cos(phi_rad - eta);

  Serial.print("eta = "); Serial.println(eta * 180.0 / M_PI);
  Serial.print("xc = "); Serial.println(xc);
  Serial.print("yc = "); Serial.println(yc);

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

    Serial.print("psi_d1: "); Serial.println(psi_d1_degrees);
    Serial.print("delta_psi: "); Serial.println(delta_psi_degrees);
    Serial.print("psi_d2: "); Serial.println(psi_d2_degrees);
    Serial.println();

    return {psi_d1_degrees, psi_d2_degrees};
}