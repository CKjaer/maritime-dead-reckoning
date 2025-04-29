#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_v3.h>
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <iostream>
#include <BasicLinearAlgebra.h>
#include <cmath>
#include <cstdint>
#include <vector>
#include <stdint.h>
#include <Gaussian.h>

struct AccData
{
    float accX{ 0 };
    float accY{ 0 };
    float accZ{ 0 };
};

class IMUReader {
public:
    // Constructor
    IMUReader() = default;

    // Initialize IMU
    void initIMU() {
        if (!IMU.begin()) {
            std::cout << "Failed to initialize IMU" << std::endl;
            while (1);
        }
        else {
            std::cout << "IMU initialized successfully" << std::endl;
            float sampleRate = IMU.accelerationSampleRate();
            std::cout << "Sample rate: " << sampleRate << " Hz" << std::endl;
        }
    }

    // Read accelerometer data
    void readAccelerometer() {
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(accData.accX, accData.accY, accData.accZ); // Takes references to store data
        }
        else {
            std::cerr << "Accelerometer data not available" << std::endl;
        }
    }

    // Getter for accData
    const AccData& getAccData() const {
        return accData;
    }


private:
    AccData accData; // instantiate struct
};


class GNSSReader
{
public:

private:

};

int main(void)
{
    std::cout << "Hello World!\n";
	Serial.begin(115200);
	IMUReader imu;
	imu.initIMU();
    imu.readAccelerometer();
    
	// get accelerometer data
	const AccData& accData = imu.getAccData();

	std::cout << "Accelerometer Data: " << std::endl;
	std::cout << "X: " << accData.accX << std::endl;
	std::cout << "Y: " << accData.accY << std::endl;
}
