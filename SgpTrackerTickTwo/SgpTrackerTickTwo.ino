/* It takes one satellite TLE data and runs SGP4 using TickTwo.
Output azimuth and elevation angles of the satellite.*/

#include <Sgp4.h>
#include <TickTwo.h>

Sgp4 sat;

unsigned long unixtime = 1727803310;
int timezone = 0;  //utc 
int framerate;

int year; int mon; int day; int hr; int minute; double sec;

void Second_Tick();
TickTwo timer(Second_Tick, 1000); // Call Second_Tick every 1000ms (1 second)

void Second_Tick()
{
  unixtime += 1;
       
  invjday(sat.satJd, timezone, true, year, mon, day, hr, minute, sec);
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

  Serial.println("Framerate: " + String(framerate) + " calc/sec");
  Serial.println();
     
  framerate = 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  
  sat.site(51.581468, -1.31088, 120); //set site latitude[°], longitude[°] and altitude[m]

  char satname[] = "ISS (ZARYA)";
  char tle_line1[] = "1 25544U 98067A   24276.37395833  .00047432  00000+0  83119-3 0  9996";  //Line one from the TLE data
  char tle_line2[] = "2 25544  51.6382 144.4720 0007356  54.4438 303.9262 15.50074020475160";  //Line two from the TLE data

  /*sat.site(0, -158, 120); //set site latitude[°], longitude[°] and altitude[m]

  char satname[] = "Reference";
  char tle_line1[] = "1 25544U 98067A   24276.37395833  .00047432  00000+0  83119-3 0  9996";  //Line one from the TLE data
  char tle_line2[] = "2 25544  00.0000 144.4720 0000000  54.4438 303.9262 15.50074020475160";  //Line two from the TLE data
  */
  
  sat.init(satname, tle_line1, tle_line2);     //initialize satellite parameters 

  //Display TLE epoch time
  double jdC = sat.satrec.jdsatepoch;
  invjday(jdC, timezone, true, year, mon, day, hr, minute, sec);
  Serial.println("Epoch: " + String(day) + '/' + String(mon) + '/' + String(year) + ' ' + String(hr) + ':' + String(minute) + ':' + String(sec));
  Serial.println();

  timer.start(); // Start the TickTwo timer
}

void loop() {
  timer.update(); // Update the TickTwo timer

  sat.findsat(unixtime);
  framerate += 1;
}