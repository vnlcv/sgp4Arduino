/* Satellite Tracking 

This code tracks one satellite using its TLE (Two-Line Element) data using SGP4 algorithm and TickTwo library.
It calculates and outputs azimuth and elevation angles of the satellite every second.
It also shows when the satellite is trackable - above 25 degrees.

The reference coordinates and reference TLE are used to test the code. 
User is on equator and satellite orbits over equator (inclination and eccentricity = 0 in TLE). 
As satellite passes over user, azimuth switches from 270° to 90°.

Adafruit 254 MicroSD card breakout board+ 
-Connect the 5V pin to the 5V pin on the Arduino
-Connect the GND pin to the GND pin on the Arduino
-Connect CLK to pin D13 -Connect DO to pin D12
-Connect DI to pin D11
-Connect CS to pin D10

SparkFun GPS Breakout - NEO-M9N, Chip Antenna (Qwiic) (38400 baud)
-Connect the 5V pin to the 5V pin on the Arduino
-Connect the GND pin to the GND pin on the Arduino
-Connect SDA to pin A4 
-Connect SCL to pin A5
*/

#include <Sgp4.h>
#include <TickTwo.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // Include GNSS library
#include <Wire.h>
#include <SD.h>
#include <SPI.h>

const int chipSelect = 10; // Chip Select pin for SD card

Sgp4 sat;
SFE_UBLOX_GNSS myGNSS; // Create a GNSS object

// Variables for SGP4
unsigned long unixtime;
int timezone = 0; // UTC 
int framerate;
int year; int mon; int day; int hr; int minute; double sec;
bool continueRunning = true; // Bool to control the main loop

void Second_Tick();
TickTwo timer(Second_Tick, 1000); // Call Second_Tick every 1000ms (1 second)

// Function to update and display satellite information every second
void Second_Tick()
{
  // Read GPS
  // Update GPS data
  if (myGNSS.getGnssFixOk()) {
    unixtime = myGNSS.getUnixEpoch();
    double latitude = myGNSS.getLatitude() / 10000000.0;
    double longitude = myGNSS.getLongitude() / 10000000.0;
    double altitude = myGNSS.getAltitude() / 1000.0; // Convert to meters
    
    sat.site(latitude, longitude, altitude); // Update site location
    
    // Print GPS data
    Serial.println("GPS data");
    Serial.print("GPS: Lat=");
    Serial.print(latitude, 6);
    Serial.print(", Lon=");
    Serial.print(longitude, 6);
    Serial.print(", Alt=");
    Serial.print(altitude, 2);
    Serial.print("m, SIV=");
    Serial.println(myGNSS.getSIV());
    
    // Print GPS time
    Serial.print("GPS Time: ");
    Serial.print(myGNSS.getUnixEpoch());
    Serial.print(myGNSS.getYear());
    Serial.print("-");
    Serial.print(myGNSS.getMonth());
    Serial.print("-");
    Serial.print(myGNSS.getDay());
    Serial.print(" ");
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    Serial.println(myGNSS.getSecond());
  } else {
    Serial.println("Waiting for GPS fix...");
  }
       
  invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec); //convert satellite's Julian date to calendar date. sat.satJd depends on unixtime
  Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  //Satellite TLE
  Serial.println("TLE data");
  Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
  Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));

  switch(sat.satVis){
    case -2:
      Serial.println("Visible : Under horizon"); //elevation < 0
      break;
    case -1:
      Serial.println("Visible : Daylight"); //elevation > 0
      break;
    default:
      Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed, 1000:visible
      break;
  }

  Serial.println("Framerate: " + String(framerate) + " calc/sec");
  Serial.println();
     
  framerate = 0;// Reset to 0

  // Check if satellite is trackable (above 25 degrees)
  if (sat.satEl > 25) {
    Serial.println("Satellite elevation is above 25 degrees.");
    Serial.println();
  } else {
    Serial.println("Satellite elevation is below 25 degrees.");
    Serial.println();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Serial.println();
  
  //GPS
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)

  // // Satellite TLE data
  // char satname[] = "ONEWEB-0664";
  // char tle_line1[] = "1 55824U 23029AE  24276.62216158  .00000045  00000+0  78711-4 0  9998";  //Line one from the TLE data
  // char tle_line2[] = "2 55824  87.9273  38.3197 0001440 113.5941 246.5344 13.20770475 78960";  //Line two from the TLE data

  /*Reference coordinates and TLE to test user and satellite pass over equator.
  sat.site(0, -158, 120); //set site latitude[°], longitude[°] and altitude[m]

  char satname[] = "Reference";
  char tle_line1[] = "1 25544U 98067A   24276.37395833  .00047432  00000+0  83119-3 0  9996";  //Line one from the TLE data
  char tle_line2[] = "2 25544  00.0000 144.4720 0000000  54.4438 303.9262 15.50074020475160";  //Line two from the TLE data
  */
  
  //SD card 
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial);
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed!");
    Serial.println("DEBUG: Check SD card connection and CS_PIN definition");
    while (1);
  }
  Serial.println("initialization done.");

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

    char tle_line1_char[70];
    char tle_line2_char[70];
    tle_line1.toCharArray(tle_line1_char, sizeof(tle_line1_char));
    tle_line2.toCharArray(tle_line2_char, sizeof(tle_line2_char));

    if (sat.init(satname.c_str(), tle_line1_char, tle_line2_char)) {
      Serial.println("DEBUG: Satellite parameters initialized successfully");
    } else {
      Serial.println("ERROR: Failed to initialize satellite parameters");
      while(1);
    }   

  // Display TLE epoch time
  double jdC = sat.satrec.jdsatepoch;
  invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
  Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println();
  } else {
    Serial.println("Error opening tle.txt for satellite initialization");
    while (1);
  }

  // Start the TickTwo timer
  timer.start(); 
}

void loop() {
  // If continueRunning is false, stop the loop
  if (!continueRunning) {
    return;
  }

  // Update the TickTwo timer and find the satellite
  timer.update(); 
  sat.findsat(unixtime); // Update satellite position at unixtime
  framerate += 1; // Shows how many time sat.findsat(unixtime) is called within each 1000 ms interval 
}