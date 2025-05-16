#include "UKF.h"
#include "IMUReader.h"
#include "GNSSReader.h"
#include "WGS84toCartesian.hpp"
#include <Wire.h>  // Required for I2C
#include <BasicLinearAlgebra.h>
#include <array>


GNSSReader gnss;
IMUReader imu{ Wire, LSM6DS3_ADDRESS };
constexpr float headingDegrees{ 180.0f };
UKF ukf(headingDegrees);

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for the serial port to initialize
  Wire.begin();     // Initialize I2C
  if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
  if (!imu.begin()) { Serial.println("Failed to initialize IMU"); }
  imu.setAccRate13Hz();
  delay(5000);
}

void loop() {

  static bool referenceInitialized{ false };
  static std::array<double, 2> currentPosition;
  static std::array<double, 2> cartesianPosition;
  static std::array<double, 2> reference;
  static BLA::Matrix<ukf.n_m> measurements;

  if (gnss.updateCoordinates() && imu.updateAccelerometer()) {  // NB. Default GNSS receiver sample rate is 1 Hz
    const auto& coords = gnss.getCoordinates();

    if (!referenceInitialized) {
      reference = { coords.latitude, coords.longitude };
      referenceInitialized = true;
      return;
    }

    // Convert to Cartesian position in meters using the UTM projection
    currentPosition = { coords.latitude, coords.longitude };
    cartesianPosition = wgs84::toCartesian(reference, currentPosition);

    const auto& accData = imu.getAccData();
    measurements = { static_cast<float>(cartesianPosition[0]), static_cast<float>(cartesianPosition[1]), accData.accX, accData.accY };

    // Compute the positional state prediction using the UKF
    ukf.timeUpdate();
    ukf.measurementUpdate(measurements);

    // Print the measurement and estimate for data acquisition using serial_reader.py
    gnss.printToSerial(cartesianPosition, ukf.getPosition(), 10);
   
    // Toggle the noise covariance by scaling it with 20x after 30 samples 
    // to simulate GNSS interference
    ukf.simulateOutage(30);

  }
}
