/*
  Satellite Tracking

  This code tracks a satellite using its TLE (Two-Line Element) data with the SGP4 algorithm
  and TickTwo library. It calculates and outputs azimuth and elevation angles every second,
  indicating when the satellite is trackable (above 25 degrees).

  Hardware Connections:
  ---------------------
  Adafruit 254 MicroSD Card Breakout:
    - 5V  -> 5V on Arduino
    - GND  -> GND on Arduino
    - CLK  -> D13
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

// -------------------- Constants and Definitions --------------------
const int SD_CS_PIN = 10;                     // Chip Select pin for SD card
const unsigned long TIMER_INTERVAL_MS = 1000; // Timer interval in milliseconds
const double TRACKABLE_ELEVATION = 25.0;      // Elevation threshold in degrees
const char* TLE_FILE_NAME = "tle.txt";         // TLE data file name

// -------------------- Global Objects --------------------
Sgp4 satellite;
SFE_UBLOX_GNSS gnss;

// Forward declaration of the callback function
void onSecondTick();

// Initialize TickTwo with the callback, interval, repeat count, and resolution
TickTwo timer(onSecondTick, TIMER_INTERVAL_MS, 0, MILLIS);

// -------------------- Global Variables --------------------
unsigned long unixTime = 0;
int timezoneOffset = 0; // UTC
int frameRate = 0;
bool isRunning = true;

// Date and Time Variables
int year, month, day, hour, minute;
double secondDouble;

// -------------------- Function Prototypes --------------------
void setupGPS();
bool initializeSDCard();
bool loadTLEFromSD();
void initializeSatellite(const String& satName, char* tleLine1Char, char* tleLine2Char);
void printGPSData();
void printSatelliteData();
void checkTrackable();

// -------------------- Setup Function --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to connect

  Wire.begin();
  Serial.println("\n--- Satellite Tracking Initialization ---");

  setupGPS();

  if (!initializeSDCard()) {
    Serial.println("SD Card initialization failed. Halting execution.");
    while (1);
  }

  if (!loadTLEFromSD()) {
    Serial.println("Failed to load TLE data. Halting execution.");
    while (1);
  }

  // Timer is already initialized globally with the callback
  timer.start();

  Serial.println("--- Initialization Complete ---\n");
}

// -------------------- Main Loop --------------------
void loop() {
  if (!isRunning) {
    return;
  }

  timer.update();
  satellite.findsat(unixTime);
  frameRate++;
}

// -------------------- Function Implementations --------------------

// Initialize GPS Module
void setupGPS() {
  if (!gnss.begin()) {
    Serial.println(F("u-blox GNSS not detected. Please check wiring. Halting execution."));
    while (1);
  }
  gnss.setI2COutput(COM_TYPE_UBX); // Set I2C port to output UBX only
  Serial.println("GPS module initialized successfully.");
}

// Initialize SD Card
bool initializeSDCard() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(" Initialization failed!");
    Serial.println("DEBUG: Check SD card connection and CS_PIN definition.");
    return false;
  }
  Serial.println(" Initialization done.");
  return true;
}

// Load TLE Data from SD Card
bool loadTLEFromSD() {
  File tleFile = SD.open(TLE_FILE_NAME);
  if (!tleFile) {
    Serial.println("Error opening " + String(TLE_FILE_NAME) + " for satellite initialization.");
    return false;
  }

  String satName = tleFile.readStringUntil('\n');
  String tleLine1 = tleFile.readStringUntil('\n');
  String tleLine2 = tleFile.readStringUntil('\n');

  tleFile.close();
  Serial.println("TLE data read and file closed.");

  // Trim newline and carriage return characters
  satName.trim();
  tleLine1.trim();
  tleLine2.trim();

  Serial.println("Satellite Name: " + satName);
  Serial.println("TLE Line 1: " + tleLine1);
  Serial.println("TLE Line 2: " + tleLine2);

  // Convert TLE lines to C-style strings
  char tleLine1Char[130]; // Adjusted size based on library expectation
  char tleLine2Char[130];
  tleLine1.toCharArray(tleLine1Char, sizeof(tleLine1Char));
  tleLine2.toCharArray(tleLine2Char, sizeof(tleLine2Char));

  // Initialize Satellite with TLE Data
  initializeSatellite(satName, tleLine1Char, tleLine2Char);

  return true;
}

// Initialize Satellite Object
void initializeSatellite(const String& satName, char* tleLine1Char, char* tleLine2Char) {
  if (!satellite.init(satName.c_str(), tleLine1Char, tleLine2Char)) {
    Serial.println("ERROR: Failed to initialize satellite parameters.");
    while (1);
  }
  Serial.println("Satellite parameters initialized successfully.");

  // Display TLE Epoch Time
  double jdEpoch = satellite.satrec.jdsatepoch;
  invjday(jdEpoch, timezoneOffset, true, year, month, day, hour, minute, secondDouble);

  // Replace Serial.printf with buffer and Serial.println
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Epoch: %02d/%02d/%04d %02d:%02d:%.2f\n\n", day, month, year, hour, minute, secondDouble);
  Serial.print(buffer);
}

// Timer Callback Function - Executes Every Second
void onSecondTick() {
  frameRate = 0; // Reset frame rate counter

  // Update GPS Data
  printGPSData();

  // Convert Satellite Julian Date to Calendar Date
  invjday(satellite.satJd, timezoneOffset, true, year, month, day, hour, minute, secondDouble);
  
  // Replace Serial.printf with buffer and Serial.println
  char buffer[100];
  snprintf(buffer, sizeof(buffer), "Satellite Time: %02d/%02d/%04d %02d:%02d:%.2f\n", day, month, year, hour, minute, secondDouble);
  Serial.print(buffer);

  // Print Satellite Data
  printSatelliteData();

  // Check and Display Trackable Status
  checkTrackable();

  Serial.println(); // Blank line for readability
}

// Print GPS Information
void printGPSData() {
  if (gnss.getGnssFixOk()) {
    unixTime = gnss.getUnixEpoch();
    double latitude = gnss.getLatitude() / 1e7;
    double longitude = gnss.getLongitude() / 1e7;
    double altitude = gnss.getAltitude() / 1e3; // Convert to meters

    satellite.site(latitude, longitude, altitude); // Update site location

    Serial.println("---- GPS Data ----");
    // Replace Serial.printf with buffer
    char buffer[100];
    snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f m, SIV: %d", latitude, longitude, altitude, gnss.getSIV());
    Serial.println(buffer);

    // Print GPS Time
    Serial.print("GPS Time: ");
    snprintf(buffer, sizeof(buffer), "%lu %04d-%02d-%02d %02d:%02d:%.2f\n",
             gnss.getUnixEpoch(),
             gnss.getYear(),
             gnss.getMonth(),
             gnss.getDay(),
             gnss.getHour(),
             gnss.getMinute(),
             gnss.getSecond());
    Serial.print(buffer);
  } else {
    Serial.println("GPS Status: Waiting for fix...");
  }
}

// Print Satellite Information
void printSatelliteData() {
  Serial.println("---- Satellite Data ----");
  // Replace Serial.printf with buffer
  char buffer[150];
  snprintf(buffer, sizeof(buffer), "Azimuth: %.2f°, Elevation: %.2f°, Distance: %.2f km",
           satellite.satAz, satellite.satEl, satellite.satDist);
  Serial.println(buffer);
  
  snprintf(buffer, sizeof(buffer), "Lat: %.6f°, Lon: %.6f°, Alt: %.2f km",
           satellite.satLat, satellite.satLon, satellite.satAlt);
  Serial.println(buffer);

  // Visibility Status
  switch (satellite.satVis) {
    case -2:
      Serial.println("Visibility: Under Horizon");
      break;
    case -1:
      Serial.println("Visibility: Daylight");
      break;
    default:
      snprintf(buffer, sizeof(buffer), "Visibility: %d", satellite.satVis);
      Serial.println(buffer);
      break;
  }

  snprintf(buffer, sizeof(buffer), "Frame Rate: %d calculations/sec", frameRate);
  Serial.println(buffer);
}

// Check if Satellite is Trackable (Elevation > 25 Degrees)
void checkTrackable() {
  char buffer[100];
  if (satellite.satEl > TRACKABLE_ELEVATION) {
    snprintf(buffer, sizeof(buffer), "Status: Satellite elevation is above %.0f degrees.", TRACKABLE_ELEVATION);
  } else {
    snprintf(buffer, sizeof(buffer), "Status: Satellite elevation is below %.0f degrees.", TRACKABLE_ELEVATION);
  }
  Serial.println(buffer);
}
