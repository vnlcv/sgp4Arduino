Tracking.ino - This code tracks one satellite using its TLE (Two-Line Element) data using SGP4 algorithm and TickTwo library.
It calculates and outputs azimuth and elevation angles of the satellite every second.
It also shows when the satellite is trackable - above 25 degrees.

The reference coordinates and reference TLE are used to test the code. 
User is on equator and satellite orbits over equator (inclination and eccentricity = 0 in TLE). 
As satellite passes over user, azimuth switches from 270° to 90°.
