/* This writes one satellite TLE data to SD card.
It runs the Closed Loop to determine the beam lookup angle for a satellite.
It takes one satellite TLE/NORAD data from SD card and runs SGP4.
Output azimuth and elevation angles of the satellite
The code stops running when the satellite has an elevation angle below 45 degrees.*/

#include <Sgp4.h>
#include <SPI.h>
#include <SD.h>

#define CS_PIN 10 // Chip Select pin for SD card

Sgp4 sat;
File myFile;

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

void writeTLEFile() {
  if (SD.exists("tle.txt")) {
    SD.remove("tle.txt");
    Serial.println("DEBUG: Existing tle.txt file removed");
  }

  myFile = SD.open("tle.txt", FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to tle.txt...");
    myFile.println("ISS (ZARYA)");
    myFile.println("1 25544U 98067A   24015.50555961  .00014393  00000+0  26320-3 0  9990");
    myFile.println("2 25544  51.6415 174.3296 0005936  60.6159 110.5456 15.49564479432227");
    myFile.close();
    Serial.println("done.");
  } else {
    Serial.println("error opening tle.txt for writing");
  }
}

void readTLEFile() {
  myFile = SD.open("tle.txt");
  if (myFile) {
    Serial.println("tle.txt contents:");
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    myFile.close();
  } else {
    Serial.println("error opening tle.txt for reading");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
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

  // Write new TLE data to file
  writeTLEFile();

  // Read and print TLE data
  readTLEFile();

  // Initialize satellite with TLE data
  File tleFile = SD.open("tle.txt");
  if (tleFile) {
    String satname = tleFile.readStringUntil('\n');
    String tle_line1 = tleFile.readStringUntil('\n');
    String tle_line2 = tleFile.readStringUntil('\n');

    satname.trim();
    tle_line1.trim();
    tle_line2.trim();

    tleFile.close();
    Serial.println("DEBUG: TLE data read and file closed");

    Serial.println("DEBUG: Satellite name: " + satname);
    Serial.println("DEBUG: TLE Line 1: " + tle_line1);
    Serial.println("DEBUG: TLE Line 2: " + tle_line2);
    
    Serial.println("DEBUG: Length of satname: " + String(satname.length()));
    Serial.println("DEBUG: Length of tle_line1: " + String(tle_line1.length()));
    Serial.println("DEBUG: Length of tle_line2: " + String(tle_line2.length()));

    char tle_line1_char[130];
    char tle_line2_char[130];
    tle_line1.toCharArray(tle_line1_char, sizeof(tle_line1_char));
    tle_line2.toCharArray(tle_line2_char, sizeof(tle_line2_char));

    if (sat.init(satname.c_str(), tle_line1_char, tle_line2_char)) {
      Serial.println("DEBUG: Satellite parameters initialized successfully");
    } else {
      Serial.println("ERROR: Failed to initialize satellite parameters");
      while(1);
    }

    double jdC = sat.satrec.jdsatepoch;
    invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
    Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    Serial.println();
  } else {
    Serial.println("Error opening tle.txt for satellite initialization");
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