#include "GNSSReader.h"

GNSSReader::GNSSReader()
    : debugEnabled(false) {}

GNSSReader::GNSSReader(bool enableDebug)
    : debugEnabled(enableDebug) {}


bool GNSSReader::begin() 
{
        if (!SFE_UBLOX_GNSS::begin()) {
            Serial.println("GNSS receiver not detected at default I2C address. Check wiring.");
            return false;
        }
        if (debugEnabled) {
            enableDebugging();
        }
        setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
        saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to flash and BBR      

        return true;
}

bool GNSSReader::updateCoordinates(float samplingRate)
{
        unsigned long currentTime = millis();
        if (static_cast<float>(currentTime - previousTimeUpdate) > (1000.0 / samplingRate)) {
            previousTimeUpdate = currentTime;
            coordinates.longitude = static_cast<double>(getLongitude()) * 1e-7; // [degrees]
            coordinates.latitude = static_cast<double>(getLatitude()) * 1e-7; // [degrees]
            coordinates.altitude = static_cast<double>(getAltitude()) * 1e-3; // [m above ellipsoid]
            return true;
        }
        return false;
}

const GNSSCoordinates& GNSSReader::getCoordinates() const { return coordinates; }

void GNSSReader::printToSerial(const std::array<double, 2>& measured, const std::array<double, 2>& estimated, int numDecimals) {
	Serial.print("measured=" + String(measured[0], numDecimals) + "," + String(measured[1], numDecimals));
	Serial.print(" | ");
	Serial.println("estimated=" + String(estimated[0], numDecimals) + "," + String(estimated[1], numDecimals));

}