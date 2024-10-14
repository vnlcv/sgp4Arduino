/* Satellite Tracking 

This code tracks one satellite using its TLE (Two-Line Element) data using SGP4 algorithm and TickTwo library.
It calculates and outputs azimuth and elevation angles of the satellite every second.
It also shows when the satellite is trackable - above 25 degrees.

The first set of angles where the satellite becomes trackable (elevation > 25 degrees) is stored.
The corresponding disc rotation angles are found from the lookup table on the SD card.
The stored angles are reset when the satellite goes below 25 degrees.

The reference coordinates and reference TLE are used to test the code. 
User is on equator and satellite orbits over equator (inclination and eccentricity = 0 in TLE). 
As satellite passes over user, azimuth switches from 270° to 90°.

GPS (Harwell Campus) and initial time are hardcoded for testing until hardware received.

Adafruit 254 MicroSD card breakout board+ 
-Connect the 5V pin to the 5V pin on the Arduino
-Connect the GND pin to the GND pin on the Arduino
-Connect CLK to pin D13 
-Connect DO to pin D12
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
#include <SD.h>
#include <SPI.h>

#define CS_PIN 10 // Chip Select pin for SD card

Sgp4 sat;

// Variables for SGP4
unsigned long unixtime = 1727962002;
int timezone = 0; // UTC 
int framerate;
int year; int mon; int day; int hr; int minute; double sec;
bool continueRunning = true; // Bool to control the main loop

// Variables for storing initial trackable angles
double initialTrackableElevation = -1;
double initialTrackableAzimuth = -1;
bool initialAngleStored = false;

void Second_Tick();
TickTwo timer(Second_Tick, 1000); // Call Second_Tick every 1000ms (1 second)

// Function to update and display satellite information every second
void Second_Tick()
{
  unixtime += 1;
       
  invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec); //convert satellite's Julian date to calendar date
  Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
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
  } else {
    Serial.println("Satellite elevation is below 25 degrees.");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  // GPS data
  sat.site(51.581468, -1.31088, 120); //set site latitude[°], longitude[°] and altitude[m]

  // Satellite TLE data
  char satname[] = "ONEWEB-0664";
  char tle_line1[] = "1 55824U 23029AE  24276.62216158  .00000045  00000+0  78711-4 0  9998";  //Line one from the TLE data
  char tle_line2[] = "2 55824  87.9273  38.3197 0001440 113.5941 246.5344 13.20770475 78960";  //Line two from the TLE data

  /*Reference coordinates and TLE to test user and satellite pass over equator.
  sat.site(0, -158, 120); //set site latitude[°], longitude[°] and altitude[m]

  char satname[] = "Reference";
  char tle_line1[] = "1 25544U 98067A   24276.37395833  .00047432  00000+0  83119-3 0  9996";  //Line one from the TLE data
  char tle_line2[] = "2 25544  00.0000 144.4720 0000000  54.4438 303.9262 15.50074020475160";  //Line two from the TLE data
  */
  
  // Initialize satellite parameters
  sat.init(satname, tle_line1, tle_line2);      

  // Display TLE epoch time
  double jdC = sat.satrec.jdsatepoch;
  invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
  Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println();

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
  sat.findsat(unixtime); // Update satellite position
  framerate += 1; // Shows how many time sat.findsat(unixtime) is called within each 1000 ms interval 
}