#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
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

Sgp4 sat;
File TLEFile;
SFE_UBLOX_GNSS myGNSS;
long lastTime = 0;

// Define constants
const float OBSERVER_LAT = 51.507406923983446;
const float OBSERVER_LON = -0.12773752212524414;
const float OBSERVER_ALT = 0.05; // km
const float MIN_ELEVATION = 45.0; // degrees
const int MAX_SATELLITES = 700;

// Struct to hold satellite data
struct Satellite {
  char name[20];
  char line1[70];
  char line2[70];
};

// Array to hold satellite data (in program memory)
const Satellite satellites[MAX_SATELLITES] PROGMEM = {
  {"ISS (ZARYA)", 
   "1 25544U 98067A   24015.50555961  .00014393  00000+0  26320-3 0  9990",
   "2 25544  51.6415 174.3296 0005936  60.6159 110.5456 15.49564479432227"},
  {"ONEWEB-0012", 
   "1 44057U 19010A   24266.50249996  .00000051  00000+0  98916-4 0  9997",
   "2 44057  87.9170 354.8071 0002128 125.2440 234.8890 13.16594802268316"},
  {"ONEWEB-0010", 
   "1 44058U 19010B   24266.52783455  .00000096  00000+0  21725-3 0  9995",
   "2 44058  87.9175 354.7837 0002526 107.7864 252.3543 13.16593880268368"},
  {"ONEWEB-0008", 
   "1 44059U 19010C   24266.55316812  .00001024  00000+0  26458-2 0  9997",
   "2 44059  87.9171 354.7794 0002017  92.0179 268.1183 13.16593499268481"},
  {"ONEWEB-0007", 
   "1 44060U 19010D   24266.61813633  .00000167  00000+0  40951-3 0  9991",
   "2 44060  87.9110  25.0916 0001828 113.0680 247.0643 13.15548120268207"},
  {"ONEWEB-0006", 
   "1 44061U 19010E   24266.62209708  .00000683  00000+0  17789-2 0  9990",
   "2 44061  87.9099  25.0649 0001956 106.9392 253.1952 13.15546575268258"},
  {"ONEWEB-0011", 
   "1 44062U 19010F   24266.82017936 -.00000037  00000+0 -13354-3 0  9993",
   "2 44062  87.9105  25.0285 0002011  97.8105 262.3254 13.15552120268278"},
  {"ONEWEB-0013", 
   "1 45131U 20008A   24266.77994130 -.00000385  00000+0 -11564-2 0  9990",
   "2 45131  87.8697 180.1009 0000881  77.5417 282.5808 13.09387186223307"},
  {"ONEWEB-0017", 
   "1 45132U 20008B   24266.45418600  .00000008  00000+0 -13904-4 0  9992",
   "2 45132  87.8984 162.2019 0001993  78.5359 281.5992 13.10375145225084"},
  {"ONEWEB-0020", 
   "1 45133U 20008C   24266.95882052 -.00000238  00000+0 -71881-3 0  9997",
   "2 45133  87.8974 162.0877 0001970  73.8093 286.3251 13.10369873225545"},
  {"ONEWEB-0021", 
   "1 45134U 20008D   24266.43924266 -.00000498  00000+0 -14640-2 0  9992",
   "2 45134  87.8989 162.2099 0001690  79.0158 281.1159 13.10372842225785"},
  {"ONEWEB-0023",            
  "1 45136U 20008F   24266.68076504 -.00000301  00000+0 -89770-3 0  9994",
  "2 45136  87.8986 162.1512 0001730  62.6388 297.4915 13.10374589225242"},
  {"ONEWEB-0024",             
  "1 45137U 20008G   24266.95715725 -.00000204  00000+0 -62071-3 0  9997",
  "2 45137  87.8967 162.0743 0001746  76.5990 283.5331 13.10369309225441"},
  {"ONEWEB-0025",             
  "1 45138U 20008H   24266.66997620 -.00000732  00000+0 -21340-2 0  9991",
  "2 45138  87.8978 162.1313 0001696  73.1098 287.0215 13.10373648225737"},
  {"ONEWEB-0026",             
  "1 45139U 20008J   24266.96353175  .00000544  00000+0  14996-2 0  9994",
  "2 45139  87.8964 146.9814 0001909  75.1202 285.0137 13.11417986228240"},
  // Add more satellites here...
};

void setup() {
  //Read SD card for TLE data
  Serial.begin(9600);
  while (!Serial) {
    ;// Wait for Serial to be ready
  }
  Serial.print("Initialising SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialisation failed");
    while (1);
  }
  Serial.println("intiialisation done.");

  // open the file to read TLE data. Only one file can be open at a time
  // so we need to close it when we're done.
  TLEFile = SD.open("TLE.txt");
  if (TLEFile) {
    Serial.println("TLE.txt:");

    //Read from file until there's nothing else in it
    while (TLEFile.available()) {
      Serial.write(TLEFile.read());
    }
    TLEFile.close();
  } else {
    // if the file didn't open, print an error
    Serial.println("error opening TLE.txt");
  }

  //Read GPS module for user's GPS data
  Serial2.begin(115200);
  while (!Serial2) 
    ; //Wait for user to open terminal
  Serial2.println("SparkFun u-blox:");

  Wire.begin();

  if (myGNSS.begin() == false) //Conect to the u-blox module using Wire port
  {
    Serial2.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing"));
    while(1)
      ;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //set the I2C port to output UBX only (turn off NMEA noise)



  //sat.init(satname,tle_line1,tle_line2);
  

  // Set the current time (replace with actual time setting method)
  setTime(0, 0, 0, 1, 1, 2024); // 00:00:00 January 1, 2024
}

void loop() {
  selectClosestSatellite();
  delay(60000); // Update every minute
}

void selectClosestSatellite() {
  
}

void Motion() {

}

float calculateElevation(float x, float y, float z) {
  // Convert ECI to topocentric coordinates (simplified)
  float dx = x - OBSERVER_LAT;
  float dy = y - OBSERVER_LON;
  float dz = z - OBSERVER_ALT;
  float range = sqrt(dx*dx + dy*dy + dz*dz);
  return asin(dz / range) * 180.0 / PI;
}