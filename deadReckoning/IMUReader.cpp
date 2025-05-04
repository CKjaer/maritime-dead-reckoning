#include "IMUReader.h"

IMUReader::IMUReader(TwoWire& wire, uint8_t slaveAddress) : LSM6DS3Class{ wire, slaveAddress } {}
void IMUReader::setAccRate13Hz() 
{
    writeRegister(0x10, 0b00011000);
    writeRegister(0x11, 0b00011100);
}

void IMUReader::setAccRate26Hz()
{
    writeRegister(0x10, 0b00101000);
    writeRegister(0x11, 0b00101100);
}

void IMUReader::setAccRate52Hz()
{
    writeRegister(0x10, 0b00111000);
    writeRegister(0x11, 0b00111100);
}

void IMUReader::setAccRate104Hz()
{ // Default
    writeRegister(0x10, 0b01001000);
    writeRegister(0x11, 0b01001100);
}

bool IMUReader::updateAccelerometer()
{
    if (accelerationAvailable()) {
        readAcceleration(accData.accX, accData.accY, accData.accZ);

        // Conversion from g to m/s^2            
        accData.accX *= gravitationConst;
        accData.accY *= gravitationConst;
        accData.accZ *= gravitationConst;

        return true;
    }
    return false;
}

const AccData& IMUReader::getAccData() const
{
    return accData;
}