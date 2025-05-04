#include "UKF.cpp" // NB. should be a cpp + .h file
#include "WGS84toCartesian.hpp"
#include "IMUReader.h"
#include "GNSSReader.h"
#include <Wire.h> // Required for I2C 
#include <BasicLinearAlgebra.h>
#include <array>
Hello

GNSSReader gnss;
IMUReader imu{ Wire, LSM6DS3_ADDRESS };
UKF ukf;


void setup()
{
    Serial.begin(115200);
    while (!Serial); // Wait for the serial port to initialize
    Wire.begin(); // Initialize I2C
    if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
    if (!imu.begin()) { Serial.println("Failed to initialize IMU"); } 
    imu.setAccRate13Hz();
}

void loop() {
    
    static bool referenceInitialized = false;
    static std::array<double, 2> lastReference;
    static std::array<double, 2> currentPosition;
    static BLA::Matrix<ukf.n_m> measurements;


    if (gnss.updateCoordinates() && imu.updateAccelerometer()) { // NB. GNSS receiver sample rate is 1 Hz
        const auto& coords = gnss.getCoordinates();

		if (!referenceInitialized) {
			lastReference = { coords.latitude, coords.longitude };
			referenceInitialized = true;
			return;
		}

        currentPosition = { coords.latitude, coords.longitude };
        currentPosition = wgs84::toCartesian(lastReference, currentPosition); // Convert to Cartesian in meters using the UTM projection
        lastReference = currentPosition;
        
        const auto& accData = imu.getAccData();
        measurements = { currentPosition[0], currentPosition[1], accData.accX, accData.accY };
        
        // Compute the positional state prediction using the UKF
        ukf.timeUpdate(); 
        ukf.measurementUpdate(measurements);

        // Print the measurement and estimate for data acquisition using serial_reader.py
        gnss.printToSerial(currentPosition, ukf.getPosition(), 10);
        
        // Debugging
        // Serial.println("Position data: Lat=" + String(referencePosition[0]) + " Lon=" + String(referencePosition[1]));
        // Serial.println("Accelerometer data: X=" + String(accData.accX) + " Y=" + String(accData.accY));


    }
}

