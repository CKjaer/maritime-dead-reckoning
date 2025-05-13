#include "UKF.cpp" // NB. should be a cpp + .h file
#include "WGS84toCartesian.hpp"
#include "IMUReader.h"
#include "GNSSReader.h"
#include <Wire.h> // Required for I2C 
#include <BasicLinearAlgebra.h>
#include <array>


GNSSReader gnss;
IMUReader imu{ Wire, LSM6DS3_ADDRESS };
constexpr float headingDegrees{ 180.0f };
UKF ukf(headingDegrees);

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
    
    static bool referenceInitialized{ false };
    static std::array<double, 2> lastReference;
    static std::array<double, 2> currentPosition;
    static std::array<double ,2> cartesianPosition;
    static BLA::Matrix<ukf.n_m> measurements;
    
    if (gnss.updateCoordinates() && imu.updateAccelerometer()) { // NB. GNSS receiver sample rate is 1 Hz
        const auto& coords = gnss.getCoordinates();

      if (!referenceInitialized) {
        lastReference = { coords.latitude, coords.longitude };
        referenceInitialized = true;
        return;
      }

        currentPosition = { coords.latitude, coords.longitude };
        cartesianPosition = wgs84::toCartesian(lastReference, currentPosition); // Convert to Cartesian in meters using the UTM projection
        lastReference = currentPosition;
        
        const auto& accData = imu.getAccData();
        measurements = { cartesianPosition[0], cartesianPosition[1], accData.accX, accData.accY };
        
        // Compute the positional state prediction using the UKF
        ukf.timeUpdate(); 
        ukf.measurementUpdate(measurements);

        // Print the measurement and estimate for data acquisition using serial_reader.py
        gnss.printToSerial(cartesianPosition, ukf.getPosition(), 10);

        // Increase the noise covariance by 10x to after 50 measurements
        // to simulate GNSS intereference 
        ukf.simulateOutage(5);
    }
}

