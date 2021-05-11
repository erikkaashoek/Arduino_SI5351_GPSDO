# Arduino_SI5351_GPSDO
 Very simple Arduino nano and SI5351 GPSDO using the PPS from a GPS receiver based on:
 http://roland.cordesses.free.fr/GPS_Si2cor.html
 
 Instead of a fixed 40 seconds measuring time this version scales the measurement time with the measured stability.
 Shortest measurement time is 1 second.
 Max measurement time is 400 seconds. 
 
 The serial input from the GPS is not needed, only the PPS pulse. This allows to use the nano USB serial port for status messages and loading new software without having to  decouple the GPS serial.
 The rest of the ino file is simplified
 
