#include "WGS84toCartesian.hpp"
#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <tuple>
#include <array>
#include <cmath>

struct GNSSCoordinates {
    double latitude;
    double longitude;
    double altitude;
};

/**
* @brief Interface for GNSS receiver
* Handles communication with the u-blox GNSS receiver module via I2C,
* providing methods to initialize the module, update coordinates at a specified
* sampling rate, and access the location data.
* @todo Move initial coordinate reference as a method instead of in loop()
*/
class GNSSReader {
public:
    GNSSReader() : debugEnabled(false) {}
    GNSSReader(bool enableDebug) : debugEnabled(enableDebug) {}

    bool begin() {
        if (!gnss.begin()) {
            Serial.println("GNSS receiver not detected at default I2C address. Check wiring.");
            return false;
        }
        if (debugEnabled) {
            gnss.enableDebugging();
        }
        gnss.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
        gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR      

        return true;
    }

    bool updateCoordinates(float samplingRate) {
        unsigned long currentTime = millis();
        if (static_cast<float>(currentTime - previousTimeUpdate) > (1000.0 / samplingRate)) {
            previousTimeUpdate = currentTime;
            coordinates.longitude = static_cast<double>(gnss.getLongitude()) * 1e-7; // [degrees]
            coordinates.latitude = static_cast<double>(gnss.getLatitude()) * 1e-7; // [degrees]
            coordinates.altitude = static_cast<double>(gnss.getAltitude()) * 1e-3; // [m above ellipsoid]
            return true;
        }
        return false;
    }

    const GNSSCoordinates& getCoordinates() { return coordinates; }

    void printToSerial(const std::array<double, 2>& measured, const std::array<double, 2>& estimated, int numDecimals) {
        Serial.print("measured=");
        Serial.print(measured[0], numDecimals);
        Serial.print(",");
        Serial.print(measured[1], numDecimals);
        Serial.print(" | ");
        Serial.print("estimated=");
        Serial.print(estimated[0], numDecimals);
        Serial.print(",");
        Serial.println(estimated[1], numDecimals);
    }

private:
    unsigned long previousTimeUpdate{ 0UL };
    bool debugEnabled;
    SFE_UBLOX_GNSS gnss;
    GNSSCoordinates coordinates;
};

constexpr struct {
    float sampRate{ 1.0f }; // [Hz] Recommended to avoid excessive I2C traffic
    int decimalsToDisplay{ 10 };
} gnssSettings;

bool referenceInitialized = false;
std::array<double, 2> referencePosition;

GNSSReader gnss;

void setup()
{
    constexpr int baudRate{ 115200 };
    Serial.begin(baudRate);
    Wire.begin(); // Initialize I2C
    if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
}

void loop() {
    if (gnss.updateCoordinates(gnssSettings.sampRate)) {
        const GNSSCoordinates& coords = gnss.getCoordinates();

        if (!referenceInitialized) {
            referencePosition = { coords.latitude, coords.longitude };
            Serial.println("Reference position initialized:");
            referenceInitialized = true;
            return;
        }

        std::array<double, 2> currentPosition = { coords.latitude, coords.longitude };
        std::array<double, 2> cartesian = wgs84::toCartesian(referencePosition, currentPosition);
        gnss.printToSerial(currentPosition, cartesian, gnssSettings.decimalsToDisplay);
    }
}