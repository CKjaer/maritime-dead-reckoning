#pragma once
#include <array>
#include <Wire.h> // Required for I2C
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

class GNSSReader : public SFE_UBLOX_GNSS {
public:
    GNSSReader() : debugEnabled(false) {}
    GNSSReader(bool enableDebug) : debugEnabled(enableDebug) {}

    struct GNSSCoordinates {
        double latitude;
        double longitude;
        double altitude;
    };

    bool begin();

    bool updateCoordinates(float samplingRate = 1.0f);

    const GNSSCoordinates& getCoordinates() const;

    void printToSerial(const std::array<double, 2>& measured,
                       const std::array<double, 2>& estimated, 
                       int numDecimals = 10);

private:
    unsigned long previousTimeUpdate{ 0UL };
    bool debugEnabled;
    GNSSCoordinates coordinates;
};