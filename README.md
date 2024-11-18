softwareControl.ino - This code tracks a satellite using its TLE (Two-Line Element) data with the SGP4 algorithm. 
  It calculates and outputs azimuth and elevation angles,
  indicating when the satellite is trackable (above 25 degrees).
  Required disc positional angles are calculated and motors rotate to achieve these angles.

  GPS module is setup and uses fallback coordinates and Unix time if no GPS fix is available. 
  This ensures continuous operation even without a GPS signal.

  Hardware Connections:
  ---------------------
  Adafruit 254 MicroSD Card Breakout:
    - 3.3V  -> 3.3V on Arduino
    - GND -> GND on Arduino
    - CLK -> D13
    - DO   -> D12
    - DI   -> D11
    - CS   -> D10

  SparkFun GPS Breakout - NEO-M9N (Qwiic):
    - 3.3V  -> 3.3V on Arduino
    - GND -> GND on Arduino
    - SDA -> A4
    - SCL -> A5

  Right Motor (Top Disc):
    - Pulse  -> D2 (purple)
    - Direction -> D3 (orange)
    - Enable -> D4 (yellow)

  Left Motor (Bottom Disc):
    - Pulse  -> D5 (purple)
    - Direction -> D6 (orange)
    - Enable -> D7 (yellow)

gui.py - This code reads elevation and azimuth from the Serial communication to display a Skyplot of satellites.

(motorControl - This is to test motor control code only.)
