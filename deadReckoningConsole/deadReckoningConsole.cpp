#include "WGS84toCartesian.hpp"
#include "IMUReader.h"
#include "GNSSReader.h"
#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <BasicLinearAlgebra.h>
#include <iostream>
#include <cmath>
#include <cstdint>


class GNSSReader
{
    constexpr int baudRate{ 115200 };
    Serial.begin(baudRate);
    while (!Serial);
    Wire.begin(); // Initialize I2C
    if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
	if (!imu.begin()) { Serial.println("Failed to initialize IMU"); }
}

void loop() {
    if (gnss.updateCoordinates() && imu.updateAccelerometer()) {
        const GNSSCoordinates& coords = gnss.getCoordinates();
        if (!referenceInitialized) {
            referencePosition = { coords.latitude, coords.longitude };
            Serial.println("Reference position initialized: Lat=" + String(referencePosition[0]) + " Lon=" + String(referencePosition[1]));
            referenceInitialized = true;
            return;
        }

        const AccData& accData = imu.getAccData();
		Serial.println("Accelerometer data: X=" + String(accData.accX) + " Y=" + String(accData.accY));
       
        // Conversion to Cartesian
       
        std::array<double, 2> currentPosition = { coords.latitude, coords.longitude };
        std::array<double, 2> cartesian = wgs84::toCartesian(referencePosition, currentPosition);
        //gnss.printToSerial(currentPosition, cartesian, 10);

    }
}

