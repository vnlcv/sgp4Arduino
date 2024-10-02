//Integrated code to test gps without sd card code.
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <Ticker.h>
#include <Wire.h>
#include <Sgp4.h>
#include <SD.h>
#include <SPI.h>
#include <brent.h>
#include <sgp4coord.h>
#include <sgp4ext.h>
#include <sgp4io.h>
#include <sgp4pred.h>
#include <sgp4unit.h>
#include <visible.h>
#include <TimeLib.h>

#define CS_PIN 10 // Chip Select pin for SD card

enum State {
  TRACKING,
  SEARCHING
};
uint32_t lastGPSUpdate = 0;
State currentState = SEARCHING;
Sgp4 sat;
File myFile;
SFE_UBLOX_GNSS myGNSS;
long lastTime = 0;
tmElements_t tm;
int syear; int smon; int sday; int shr; int sminute; double ssec;

//keeping track of time
unsigned long previousMillis = 0;
const long interval = 1000; // Update every 1000 ms

unsigned long unixtime = 1727085850; //starting unix timestamp
int timezone = 0;  //utc + 0
unsigned long framerate;

// Define constants
const uint16_t GPSI2CAddress = 0x42;
const int MAX_SATELLITES = 700;
const int chipSelect = 10; // SD card chip select pin

// Struct to hold satellite data
struct Satellite {
  char name[20];
  char line1[70];
  char line2[70];
};

Satellite currentSatellite;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 1000; //update every second 

bool continueRunning = true; // Flag to control the main loop

// Array to hold satellite data (in program memory)
const Satellite satellites[MAX_SATELLITES] PROGMEM = {
  {"ISS (ZARYA)", 
   "1 25544U 98067A   24015.50555961  .00014393  00000+0  26320-3 0  9990",
   "2 25544  51.6415 174.3296 0005936  60.6159 110.5456 15.49564479432227"},
  // Add more satellites here...
};

double unixToJulian(uint32_t unixTime) {
  return (unixTime / 86400.0) + 2440587.5;
}

uint32_t getGPSTime() {
  uint16_t uyear;
  uint8_t umonth, uday, uhour, uminute, usecond;
  uint32_t unixTime = 0;

  if (myGNSS.getDateValid() && myGNSS.getTimeValid()) {
    uyear = myGNSS.getYear();
    umonth = myGNSS.getMonth();
    uday = myGNSS.getDay();
    uhour = myGNSS.getHour();
    uminute = myGNSS.getMinute();
    usecond = myGNSS.getSecond();

    // Convert to Unix timestamp
    tm.Year = uyear - 1970;
    tm.Month = umonth;
    tm.Day = uday;
    tm.Hour = uhour;
    tm.Minute = uminute;
    tm.Second = usecond;
    unixTime = makeTime(tm);
  }
  return unixTime;
}

void Second_Tick(){
  invjday(sat.satJd, timezone, true, syear, smon, sday, shr, sminute, ssec);
  Serial.println("DEBUG: Converted Julian date to calendar date");
  
  Serial.println(String(sday) + '/' + String(smon) + '/' + String(syear) + ' ' + String(shr) + ':' + String(sminute) + ':' + String(ssec));
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

//TODO change this method to return some signifier that a satellite will pass over in the next.... hour? 2 hours? some amount of time
void Predict(int many){
    passinfo overpass;                       //structure to store overpass info
    sat.initpredpoint( unixtime , 0.0 );     //finds the startpoint
    
    bool error;
    unsigned long start = millis();
    for (int i = 0; i<many ; i++){
        error = sat.nextpass(&overpass,20);     //search for the next overpass, if there are more than 20 maximums below the horizon it returns false
        delay(0);
        
        if ( error == 1){ //no error, prints overpass information
          
          invjday(overpass.jdstart ,timezone ,true , syear, smon, sday, shr, sminute, ssec);
          Serial.println("Overpass " + String(sday) + ' ' + String(smon) + ' ' + String(syear));
          Serial.println("  Start: az=" + String(overpass.azstart) + "° " + String(shr) + ':' + String(sminute) + ':' + String(ssec));
          
          invjday(overpass.jdmax ,timezone ,true , syear, smon, sday, shr, sminute, ssec);
          Serial.println("  Max: elev=" + String(overpass.maxelevation) + "° " + String(shr) + ':' + String(sminute) + ':' + String(ssec));
          
          invjday(overpass.jdstop ,timezone ,true , syear, smon, sday, shr, sminute, ssec);
          Serial.println("  Stop: az=" + String(overpass.azstop) + "° " + String(shr) + ':' + String(sminute) + ':' + String(ssec));
          
          switch(overpass.transit){
              case none:
                  break;
              case enter:
                  invjday(overpass.jdtransit ,timezone ,true , syear, smon, sday, shr, sminute, ssec);
                  Serial.println("  Enter earth shadow: " + String(shr) + ':' + String(sminute) + ':' + String(ssec)); 
                  break;
              case leave:
                  invjday(overpass.jdtransit ,timezone ,true , syear, smon, sday, shr, sminute, ssec);
                  Serial.println("  Leave earth shadow: " + String(shr) + ':' + String(sminute) + ':' + String(ssec)); 
                  break;
          }
          switch(overpass.sight){
              case lighted:
                  Serial.println("  Visible");
                  break;
              case eclipsed:
                  Serial.println("  Not visible");
                  break;
              case daylight:
                  Serial.println("  Daylight");
                  break;
          }
  
          
        }else{
            Serial.println("Prediction error");
        }
        delay(0);
    }
    unsigned long einde = millis();
    Serial.println("Time: " + String(einde-start) + " milliseconds"); 
}



void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect
  }
  Serial.println("DEBUG: Serial communication initialized");

  sat.site(-0.5276847, 166.9359231, 34); //TODO REPLACE WITH GPS GET LOCATION
  Serial.println("DEBUG: Observer location set");

  //Init satellite with TLE data
  char satname[] = "ISS (ZARYA)";
  char tle_line1[] = "1 25544U 98067A   16065.25775256 -.00164574  00000-0 -25195-2 0  9990";  //Line one from the TLE data
  char tle_line2[] = "2 25544  51.6436 216.3171 0002750 185.0333 238.0864 15.54246933988812";  //Line two from the TLE data

  sat.init(satname, tle_line1, tle_line2); //initialize satellite parameters

  /*
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
    invjday(jdC, timezone, true, syear, smon, sday, shr, sminute, ssec);
    Serial.println("Epoch: " + String(sday) + '/' + String(smon) + '/' + String(syear) + ' ' + String(shr) + ':' + String(sminute) + ':' + String(ssec));
    Serial.println();
  } else {
    Serial.println("Error opening tle.txt for satellite initialization");
    while (1);
  }
  */

  //Read GPS module for user's GPS data
  Serial.println("SparkFun u-blox:");

  Wire.begin();

  if (myGNSS.begin() == false) //Conect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing"));
    while(1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //set the I2C port to output UBX only (turn off NMEA noise)
  myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save the communications port settings to flash and BBR
  
  unixtime=getGPSTime();
  Serial.println(unixtime);
}

void loop() {
  /*
  switch (currentState) { //TODO tighten up the logic of this; this is the state machine logic that switches between searching for satellite and tracking
    case TRACKING:
      { 
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

        if (!isSatelliteVisible()) {
          currentState = SEARCHING;
        }
      }
      break;
      
    case SEARCHING: //TODO 
      {
        // if (selectClosestSatellite()) {
        //   currentState = TRACKING;
        }
      }
      break;
  }
  */
}

/*
bool selectClosestSatellite() {
  myFile = SD.open("TLE.txt");
  if (!myFile) {
    Serial.println("Error opening TLE file");
    return false;
  }
  
  while (myFile.available()) {
    // Read TLE data for next satellite
    readTLEFile();

    //TODO USE SGP4 and satellite's TLE to check if it will overpass soon, using Predict()?
    
    // Initialize satellite with new TLE data
    sat.init(currentSatellite.name, currentSatellite.line1, currentSatellite.line2);
    
    // Set observer position from GPS data
    sat.site(myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getAltitude());
    
    // Check if this satellite will pass overhead soon
    if (willPassOverhead(sat, getGPSTime())) { //TODO WRITE THIS!!!
      myFile.close();
      return true;
    }
  }
  
  myFile.close();
  return false;
}

bool willPassOverhead(Sgp4 &sat, uint32_t currentTime) { //TODO
  // use Predict() and cycle through array of satellites to determine if it will pass overhead
  // might need to make parameters a satellite and the current time? 

  return false;
}

bool isSatelliteVisible() { // TODO
  // Check if satellite is currently visible; for now just use elevation angle of above 25 degrees

  return true;
}

void trackSatellite() { // TODO replace with SatTracker.ino's method

}

void Motion(long azimuth, long elevation) { // TODO

}
*/