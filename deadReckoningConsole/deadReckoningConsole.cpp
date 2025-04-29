#include "UKF.hpp" // NB. should be a cpp file
#include "WGS84toCartesian.hpp"
#include "IMUReader.h"
#include "GNSSReader.h"
#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <BasicLinearAlgebra.h>
#include <cmath>
#include <cstdint>


bool referenceInitialized = false;
std::array<double, 2> referencePosition;

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
    
    if (gnss.updateCoordinates() && imu.updateAccelerometer()) {
        const GNSSCoordinates& coords = gnss.getCoordinates();

        if (!referenceInitialized) {
            referencePosition = { coords.latitude, coords.longitude };
            Serial.println("Reference position initialized: Lat=" + String(referencePosition[0]) + " Lon=" + String(referencePosition[1]));
            referenceInitialized = true;
            return;
        }

        // Convert the coordinates in degrees to cartesian in meters using the UTM projection
        std::array<double, 2> cartesian = wgs84::toCartesian(referencePosition, currentPosition);

        const AccData& accData = imu.getAccData();
        Serial.println("Accelerometer data: X=" + String(accData.accX) + " Y=" + String(accData.accY));

        BLA::Matrix<ukf.n_x> measurements = { cartesian[0], cartesian[0], accData.accX, accData.accY}; 
        ukf.timeUpdate();
        ukf.measurementUpdate(measurements);
        //gnss.printToSerial(currentPosition, cartesian, 10);
        

        
        
   
        

    }
}

