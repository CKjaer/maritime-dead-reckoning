#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <iostream>
#include <tuple>


class GNSSReader
{
public:
    GNSSReader() : debugEnabled(false) {}
    GNSSReader(bool enableDebug) : debugEnabled(enableDebug) {}

    bool begin()
    {
        if (!gnss.begin()) {
            Serial.println("GNSS receiver not detected at default I2C address. Check wiring.");

            return false;
        }
        if (debugEnabled) { gnss.enableDebugging(); }
        gnss.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise) 
        gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR      

        return true;
    }

    bool updateCoordinates(float samplingRate)
    {
        unsigned long currentTime = millis();
        if (static_cast<float>(currentTime - previousTimeUpdate) > (1000.0 / samplingRate))
        {
            previousTimeUpdate = currentTime;
            longitude = static_cast<double>(gnss.getLongitude());
            latitude = static_cast<double>(gnss.getLatitude());
            altitude = static_cast<double>(gnss.getAltitude());
            return true;
        }

        return false;
    }

    std::tuple<double, double, double> const getCoordinates()
    {
        return
        {
            static_cast<double>(longitude),
            static_cast<double>(latitude),
            static_cast<double>(altitude)
        };
    }
private:
    unsigned long previousTimeUpdate{ 0L };
    bool debugEnabled;
    SFE_UBLOX_GNSS gnss;

    long latitude;
    long longitude;
    long altitude;
};

GNSSReader gnss;

void setup()
{
    const int baudRate{ 115200 };
    Serial.begin(baudRate); 
    Wire.begin();
    if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
}


constexpr struct {
    float sampRate{ 1.0f }; // [Hz] Recommended to avoid excessive I2C traffic
} gnssSettings;

void loop()
{
    if (gnss.updateCoordinates(gnssSettings.sampRate))
    {
        auto [lon, lat, _] = gnss.getCoordinates();
        Serial.print("Latitude ");
        Serial.println(lat);

        Serial.print("Longitude ");
        Serial.println(lon);

    }

}

