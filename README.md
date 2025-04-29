# Dead Reckoning Countermeasure Against GNSS Jamming
## Sensors and Systems Miniproject - Spring 2025

### Project Description
This project proposes a countermeasure against GNSS outages using a dead reckoning approach, where the system can continue to provide positional updates through state estimation using an unscented Kalman filter. The estimation is derived from accelerometer data of an inertial measurement unit in the XY-plane.

### Hardware

The project uses the \textbf{Sparkfun u-blox MAX-M10S GNSS receiver} with serial data read from I2C. The .ino program is flashed to an Arduino Nano 33T, the microcontroller features the LSM6DS3 IMU as an embedded IC, providing accelerometer data.



