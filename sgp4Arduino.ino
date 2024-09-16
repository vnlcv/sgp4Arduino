#include <SparkFun_SGP4_Arduino_Library.h>
#include <TimeLib.h>

// Define constants
const float OBSERVER_LAT = 51.507406923983446;
const float OBSERVER_LON = -0.12773752212524414;
const float OBSERVER_ALT = 0.05; // km
const float MIN_ELEVATION = 45.0; // degrees
const int MAX_SATELLITES = 700;

// Create SGP4 object
sgp4 sat;

// Struct to hold satellite data
struct Satellite {
  char name[20];
  char line1[70];
  char line2[70];
};

// Array to hold satellite data (in program memory)
const Satellite satellites[MAX_SATELLITES] PROGMEM = {
  {"sample", 
   "1 25544U 98067A   24015.50555961  .00014393  00000+0  26320-3 0  9990",
   "2 25544  51.6415 174.3296 0005936  60.6159 110.5456 15.49564479432227"},
  // Add more satellites here...
};

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  // Set the current time (replace with actual time setting method)
  setTime(0, 0, 0, 1, 1, 2024); // 00:00:00 January 1, 2024
}

void loop() {
  selectClosestSatellite();
  delay(60000); // Update every minute
}

void selectClosestSatellite() {
  
}

float calculateElevation(float x, float y, float z) {
  // Convert ECI to topocentric coordinates (simplified)
  float dx = x - OBSERVER_LAT;
  float dy = y - OBSERVER_LON;
  float dz = z - OBSERVER_ALT;
  float range = sqrt(dx*dx + dy*dy + dz*dz);
  return asin(dz / range) * 180.0 / PI;
}