#include <Sgp4.h>
#include <SPI.h>
#include <SD.h>

#define CS_PIN 10 // Chip Select pin for SD card

Sgp4 sat;

unsigned long previousMillis = 0;
const long interval = 1000; // Update every 1000 ms

unsigned long unixtime = 1727085850; //starting unix timestamp
int timezone = 0;  //utc + 0
unsigned long framerate;

int year; int mon; int day; int hr; int minute; double sec;

bool continueRunning = true; // Flag to control the main loop

void Second_Tick()
{
  invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec);
  Serial.println("DEBUG: Converted Julian date to calendar date");
  
  Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));
  
  Serial.print("DEBUG: Visibility status - ");
  switch(sat.satVis){
    case -2:
      Serial.println("Under horizon");
      break;
    case -1:
      Serial.println("Daylight");
      break;
    default:
      Serial.println(String(sat.satVis) + " (0:eclipsed - 1000:visible)");
      break;
  }
  
  Serial.println("Framerate: " + String(framerate) + " calc/sec");
  Serial.println();
     
  framerate = 0;

  if (sat.satEl < 45.0) {
    Serial.println("WARNING: Satellite elevation angle is below 45 degrees. Stopping the program.");
    continueRunning = false;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("DEBUG: Serial communication initialized");
  
  sat.site(-0.5276847, 166.9359231, 34);
  Serial.println("DEBUG: Observer location set");

  Serial.print("Initializing SD card...");
  if (!SD.begin(CS_PIN)) {
    Serial.println("initialization failed!");
    Serial.println("DEBUG: Check SD card connection and CS_PIN definition");
    while (1);
  }
  Serial.println("initialization done.");

  File tleFile = SD.open("tle.txt");
  if (tleFile) {
    Serial.println("DEBUG: tle.txt opened successfully");
    char satname[25] = { 0 };
    char tle_line1[70] = { 0 };
    char tle_line2[70] = { 0 };

    tleFile.readBytesUntil('\n', satname, sizeof(satname) - 1);
    tleFile.readBytesUntil('\n', tle_line1, sizeof(tle_line1) - 1);
    tleFile.readBytesUntil('\n', tle_line2, sizeof(tle_line2) - 1);

    tleFile.close();
    Serial.println("DEBUG: TLE data read and file closed");

    Serial.println("DEBUG: Satellite name: " + String(satname));
    Serial.println("DEBUG: TLE Line 1: " + String(tle_line1));
    Serial.println("DEBUG: TLE Line 2: " + String(tle_line2));

    if (sat.init(satname, tle_line1, tle_line2)) {
      Serial.println("DEBUG: Satellite parameters initialized successfully");
    } else {
      Serial.println("ERROR: Failed to initialize satellite parameters");
      while(1);
    }

    double jdC = sat.satrec.jdsatepoch;
    invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
    Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    Serial.println();
  }
  else {
    Serial.println("Error opening tle.txt");
    Serial.println("DEBUG: Check if tle.txt exists on the SD card");
    while (1);
  }
}

void loop() {
  if (!continueRunning) {
    Serial.println("DEBUG: Program stopped due to low elevation angle");
    while(1);
  }

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    Serial.println("DEBUG: Calculating satellite position for unixtime: " + String(unixtime));
    sat.findsat(unixtime);
    Second_Tick();
    
    unixtime += 1;
  }

  framerate++;
}