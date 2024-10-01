Sgp4Tracker.ino - Takes one satellite TLE data from SD card and run SGP4. Continuously outputs azimuth and elevation angles until satellite is below 45 degrees

ReadWriteSgp4Tracker.ino - Writes one satellite TLE data to tle.txt on SD card. Reads TLE data from SD card, reads GPS and time from GPS module, and run SGP4. Continuously outputs azimuth and elevation angles until satellite is below 45 degrees. (time method to be verified, satellite has accelerated orbit)

Sgp4TrackerV2.ino - Select the closest satellite above 45° elevation (contains error, new code in branch project 2322)
