#include "UKF.hpp" // NB. should be a cpp file
#include "WGS84toCartesian.hpp"
#include "IMUReader.h"
#include "GNSSReader.h"
#include <Wire.h> // Required for I2C 
#include <BasicLinearAlgebra.h>
#include <array>



GNSSReader gnss;
IMUReader imu{ Wire, LSM6DS3_ADDRESS };
UKF ukf;

static bool referenceInitialized{ false };
static std::array<double, 2> lastReference;
static std::array<double, 2> currentPosition;
static std::array<double, 2> cartesianMeas;
static BLA::Matrix<ukf.n_x> measurements;

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
        const auto& coords = gnss.getCoordinates();

        if (!referenceInitialized) {
            lastReference = { coords.latitude, coords.longitude };
            referenceInitialized = true;
            return;
        }
        
        currentPosition = { coords.latitude, coords.longitude };
        cartesianMeas = wgs84::toCartesian(lastReference, currentPosition); // Convert to Cartesian in meters using the UTM projection
        lastReference = currentPosition;

        measurements = { cartesianMeas[0], cartesianMeas[1], imu.getAccData().accX, imu.getAccData().accY };

        
        // Compute the positional state prediction using the UKF
        ukf.timeUpdate(); 
        ukf.measurementUpdate(measurements);

        // Print estimate and measurements for data acquisition using serial_reader.py
        gnss.printToSerial(cartesianMeas, ukf.getPosition(), 10);
        
        // Debugging
        // Serial.println("Position data: Lat=" + String(referencePosition[0]) + " Lon=" + String(referencePosition[1]));
        // Serial.println("Accelerometer data: X=" + String(accData.accX) + " Y=" + String(accData.accY));


    }
}

