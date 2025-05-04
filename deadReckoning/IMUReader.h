#pragma once
#include <Arduino_LSM6DS3.h>
#include <Wire.h> // Required for I2C

/**
* @brief Interface for reading X, Y, Z accelerometer data from the IMU.
*/
class IMUReader : public LSM6DS3Class {
public:
    IMUReader(TwoWire& wire, uint8_t slaveAddress);

    struct AccData
    {
        float accX{ 0 };
        float accY{ 0 };
        float accZ{ 0 };
    };

    // Set different sample rates
    void setAccRate13Hz();

    void setAccRate26Hz();

    void setAccRate52Hz();

    void setAccRate104Hz(); // Default


    bool updateAccelerometer();


    const AccData& getAccData() const;


private:
    AccData accData;
    const float gravitationConst{ 9.816 }; // NB. dependent on latitude
};