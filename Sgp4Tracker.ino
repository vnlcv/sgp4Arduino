/*This is the Closed Loop to determine the beam lookup angle for a satellite.
It takes one satellite TLE/NORAD data from SD card and runs SGP4.
Output azimuth and elevation angles of the satellite
The code stops running when the satellite has an elevation angle below 45 degrees.
*/
#include <Sgp4.h
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#define CS_PIN 10 // Chip Select pin for SD card

//Create object
Sgp4 sat;
SFE_UBLOX_GNSS myGNSS;

unsigned long previousMillis = 0;
const long interval = 1000; // Update every 1000 ms

//unsigned long unixtime = 1727085850; //starting unix timestamp
//int timezone = 0;  //utc + 0
unsigned long framerate;

//int year; int mon; int day; int hr; int minute; double sec;

bool continueRunning = true; // Flag to control the main loop

//print satellite info
void Second_Tick()
{
    long latitude, longitude, altitude;
    uint8_t SIV;
    uint16_t year;
    uint8_t month, day, hour, minute, second;

    // Get the latest GPS time and position
    if (myGNSS.getGnssFixOk()) {
        latitude = myGNSS.getLatitude();
        longitude = myGNSS.getLongitude();
        altitude = myGNSS.getAltitude();
        SIV = myGNSS.getSIV();
        year = myGNSS.getYear();
        month = myGNSS.getMonth();
        day = myGNSS.getDay();
        hour = myGNSS.getHour();
        minute = myGNSS.getMinute();
        second = myGNSS.getSecond();

        // Print GPS info
        Serial.print(F("Date/Time: "));
        Serial.print(year); Serial.print(F("/"));
        Serial.print(month); Serial.print(F("/"));
        Serial.print(day); Serial.print(F(" "));
        Serial.print(hour); Serial.print(F(":"));
        Serial.print(minute); Serial.print(F(":"));
        Serial.println(second);

        Serial.print(F("Lat: ")); Serial.print(latitude / 10000000.0, 7);
        Serial.print(F(" Long: ")); Serial.print(longitude / 10000000.0, 7);
        Serial.print(F(" Alt: ")); Serial.print(altitude / 1000.0);
        Serial.print(F(" SIV: ")); Serial.println(SIV);
    }
    else {
        Serial.println(F("No GPS fix"));
    }

    //invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec); //convert Julian date to calendar date
    
    //Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    
    //Print satellite info
    Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
    Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));
    //Print visibility status
    switch (sat.satVis) {
    case -2:
        Serial.println("Visible : Under horizon");
        break;
    case -1:
        Serial.println("Visible : Daylight");
        break;
    default:
        Serial.println("Visible : " + String(sat.satVis));   //0:eclipsed - 1000:visible
        break;
    }
    //Print framerate (calc/sec)
    Serial.println("Framerate: " + String(framerate) + " calc/sec");
    Serial.println();

    framerate = 0;

    // Check if elevation is below 45 degrees
    if (sat.satEl < 45.0) {
        Serial.println("Satellite elevation angle is below 45 degrees. Stopping the program.");
        continueRunning = false; // Set flag to stop the main loop
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set observer location
    //sat.site(-0.5276847, 166.9359231, 34); //set site latitude[°], longitude[°] and altitude[m]

    if (myGNSS.begin() == false) {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save the communications port settings to flash and BBR

    /*
    //Init satellite with TLE data
    char satname[] = "ISS (ZARYA)";
    char tle_line1[] = "1 25544U 98067A   16065.25775256 -.00164574  00000-0 -25195-2 0  9990";  //Line one from the TLE data
    char tle_line2[] = "2 25544  51.6436 216.3171 0002750 185.0333 238.0864 15.54246933988812";  //Line two from the TLE data
    */

    //Read tle.txt
    Serial.print("Initializing SD card...");
    if (!SD.begin(CS_PIN)) {
        Serial.println("SD card initialization failed!");
        while (1);
    }
    Serial.println("SD card initialization done.");

    File tleFile = SD.open("tle.txt");
    if (tleFile) {
        char satname[25] = { 0 };
        char tle_line1[70] = { 0 };
        char tle_line2[70] = { 0 };

        tleFile.readBytesUntil('\n', satname, sizeof(satname) - 1);
        tleFile.readBytesUntil('\n', tle_line1, sizeof(tle_line1) - 1);
        tleFile.readBytesUntil('\n', tle_line2, sizeof(tle_line2) - 1);

        tleFile.close();

        sat.init(satname, tle_line1, tle_line2);     //initialize satellite parameters 

        Serial.println("Satellite initialized: " + String(satname));
    }
    else {
        Serial.println("Error opening tle.txt");
        while (1);
    }
    /*
        //Display TLE epoch time
        double jdC = sat.satrec.jdsatepoch;
        invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
        Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
        Serial.println();
    }
    else {
        Serial.println("Error opening tle.txt");
        while (1);
    }
    */
}

void loop() {
    if (!continueRunning) {
        // If continueRunning is false, stop the loop
        return;
    }

    unsigned long currentMillis = millis();

    //If 1 second passed, update satellite position and call Second_Tick
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        // Update observer location from GPS
        if (myGNSS.getGnssFixOk()) {
            long latitude = myGNSS.getLatitude();
            long longitude = myGNSS.getLongitude();
            long altitude = myGNSS.getAltitude();
            sat.site(latitude / 10000000.0, longitude / 10000000.0, altitude / 1000.0);
        }

        // Get current time from GPS
        uint16_t year;
        uint8_t month, day, hour, minute, second;
        myGNSS.getDateTime(year, month, day, hour, minute, second);
        unsigned long unixtime = myGNSS.getUnixEpoch();

        sat.findsat(unixtime);
        Second_Tick();

        //unixtime += 1;
    }

    framerate++;
}