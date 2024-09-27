#include <Sgp4.h>

#define NUM_SATELLITES 10
#define MIN_ELEVATION 45

Sgp4 satellites[NUM_SATELLITES];

unsigned long previousMillis = 0;
const long interval = 1000; // Update every 1000 ms

unsigned long unixtime = 1727085850; //starting unix timestamp
int timezone = 0;  //utc + 0
unsigned long framerate;

int year, mon, day, hr, minute;
double sec;

// Satellite TLE data
const char* satNames[NUM_SATELLITES] = {
    "ISS (ZARYA)", "NOAA 15", "NOAA 18", "NOAA 19", "METEOR-M 2", 
    "FENGYUN 3C", "METOP-B", "METOP-C", "SUOMI NPP", "GOES 16"
};
const char* tleLine1[NUM_SATELLITES] = {
    "1 25544U 98067A   21156.30527927  .00003432  00000-0  70541-4 0  9993",
    "1 25338U 98030A   21156.49443143  .00000015  00000-0  23146-4 0  9991",
    "1 28654U 05018A   21156.51989105  .00000069  00000-0  64560-4 0  9994",
    "1 33591U 09005A   21156.51342878  .00000076  00000-0  67467-4 0  9992",
    "1 40069U 14037A   21156.49971475  .00000015  00000-0  21323-4 0  9996",
    "1 39260U 13052A   21156.44938837 -.00000002  00000-0  16162-4 0  9991",
    "1 38771U 12049A   21156.50599514  .00000004  00000-0  25323-4 0  9992",
    "1 43689U 18087A   21156.49539461  .00000006  00000-0  26323-4 0  9995",
    "1 37849U 11061A   21156.50840276  .00000012  00000-0  29323-4 0  9997",
    "1 41866U 16071A   21156.28888889 -.00000289  00000-0  00000-0 0  9990"
};
const char* tleLine2[NUM_SATELLITES] = {
    "2 25544  51.6455 339.0709 0003508  68.0432  78.3395 15.48957524286754",
    "2 25338  98.7340 123.5114 0010992 170.2867 189.8483 14.25943737196736",
    "2 28654  99.0417 123.4112 0014379 123.5850 163.5117 14.12385378823243",
    "2 33591  99.1979 123.9344 0013926 235.5256 124.4959 14.12385933626155",
    "2 40069  98.5981 123.2280 0004565 123.5989 161.5257 14.20656511359632",
    "2 39260  98.6303 246.4855 0001739 123.6790 261.4600 14.12501077405027",
    "2 38771  98.7316 123.1263 0000124 114.6001 245.5266 14.21477593454208",
    "2 43689  98.7087 123.4004 0001230  92.0340 268.1024 14.21477076127869",
    "2 37849  98.7444  89.1261 0001184  90.0124 270.1199 14.19552880507133",
    "2 41866  98.0243 223.7955 0000972  94.1735  76.3560 14.00270186 17132"
};

void printSatelliteInfo(int index) {
    Sgp4& sat = satellites[index];
    invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec);
    Serial.println("Satellite: " + String(satNames[index]));
    Serial.println(String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
    Serial.println("azimuth = " + String(sat.satAz) + " elevation = " + String(sat.satEl) + " distance = " + String(sat.satDist));
    Serial.println("latitude = " + String(sat.satLat) + " longitude = " + String(sat.satLon) + " altitude = " + String(sat.satAlt));
    switch(sat.satVis){
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
    Serial.println();
}

void Second_Tick() {
    int closestSatIndex = -1;
    double closestDistance = 1e9;  // Initialize with a very large number

    for (int i = 0; i < NUM_SATELLITES; i++) {
        satellites[i].findsat(unixtime);
        if (satellites[i].satEl >= MIN_ELEVATION) {
            if (satellites[i].satDist < closestDistance) {
                closestDistance = satellites[i].satDist;
                closestSatIndex = i;
            }
        }
    }
    
    if (closestSatIndex != -1) {
        printSatelliteInfo(closestSatIndex);
    } else {
        Serial.println("No satellites above " + String(MIN_ELEVATION) + " degrees elevation.");
    }
    
    Serial.println("Framerate: " + String(framerate) + " calc/sec");
    Serial.println();
    
    framerate = 0;
}

void setup() {
    Serial.begin(115200);
    Serial.println();
  
    // Set observer location and initialize satellites
    for (int i = 0; i < NUM_SATELLITES; i++) {
        satellites[i].site(-0.5276847, 166.9359231, 34);
        
        // Create temporary non-const copies of the TLE strings
        char temp1[130];
        char temp2[130];
        strncpy(temp1, tleLine1[i], 129);
        strncpy(temp2, tleLine2[i], 129);
        temp1[129] = '\0';
        temp2[129] = '\0';
        
        // Initialize the satellite with the temporary strings
        satellites[i].init(satNames[i], temp1, temp2);
    }

    Serial.println("Satellite tracking initialized for 10 satellites.");
    Serial.println();
}

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        Second_Tick();
        unixtime += 1;
    }

    framerate++;
}