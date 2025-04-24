#include "WGS84toCartesian.hpp"
#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <BasicLinearAlgebra.h>

#include <iostream>
#include <cmath>
#include <cstdint>
#include <vector>
#include <Gaussian.h>
#include <array>

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

class GNSSReader : public SFE_UBLOX_GNSS {
public:
    GNSSReader() : debugEnabled(false) {}
    GNSSReader(bool enableDebug) : debugEnabled(enableDebug) {}

    bool begin() {
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

    bool updateCoordinates(float samplingRate = 1.0f) {
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

    const GNSSCoordinates& getCoordinates() { return coordinates; }

    void printToSerial(const std::array<double, 2>& measured, const std::array<double, 2>& estimated, int numDecimals = 10) {
        Serial.print("measured=" + String(measured[0], numDecimals) + "," + String(measured[1], numDecimals));
        Serial.print(" | ");
        Serial.print("estimated=" + String(estimated[0], numDecimals) + "," + String(estimated[1], numDecimals));
    }

private:
    unsigned long previousTimeUpdate{ 0UL };
    bool debugEnabled;
    GNSSCoordinates coordinates;
};


struct AccData
{
    float accX{ 0 };
    float accY{ 0 };
    float accZ{ 0 };
};

/**
* @brief Interface for reading X, Y, Z accelerometer data from the IMU.
*/
class IMUReader : public LSM6DS3Class {
public:
    IMUReader(TwoWire& wire, uint8_t slaveAddress) : LSM6DS3Class{ wire, slaveAddress } {}

    // Set different sample rates
    void setAccRate13Hz() {
        writeRegister(0x10, 0b00011000);
        writeRegister(0x11, 0b00011100);
    }
    void setAccRate26Hz() {
        writeRegister(0x10, 0b00101000);
        writeRegister(0x11, 0b00101100);
    }
    void setAccRate52Hz() {
        writeRegister(0x10, 0b00111000);
        writeRegister(0x11, 0b00111100);
    }
    void setAccRate104Hz() { // Default
        writeRegister(0x10, 0b01001000);
        writeRegister(0x11, 0b01001100);
    }

    bool updateAccelerometer() {
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

    const AccData& getAccData() const {
        return accData;
    }

private:
    AccData accData;
    const float gravitationConst{ 9.816 }; // NB. dependent on latitude
};

class UnscentedKalmanFilter
{
public:
    static constexpr std::uint8_t n_x { 6 }; // State vector size
    static constexpr std::uint8_t n_m { 4 }; // Measurement vector size
    static constexpr std::uint8_t num_sigma_points{ 2 * n_x + 1 }; // Number of sigma points

	// Constructor
    UnscentedKalmanFilter(float alpha, float beta, float kappa, float Ts, float phi, float sigma_a)
        : alpha(alpha), beta(beta), kappa(kappa), Ts(Ts), phi(phi), sigma_a(sigma_a)
    {
		// Calculate UKF parameters lambda and k
        lambda = std::pow(alpha, 2) * (n_x + kappa) - n_x;
		k = std::sqrt(n_x + lambda);
        
        // Initialize state and covariance matrix
        x_hat.Fill(0.f);
        P = { 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1 // Identity matrix
        };

		// Initialize process noise covariance matrix
        float sigma_qa = sigma_a * (1 - phi);
        Q.Fill(0);
        // add noise to acceleration, 0 for other states
		Q(4, 4) = sigma_qa;
		Q(5, 5) = sigma_qa;

		// Initialize measurement noise covariance matrix
        // CHECK THIS, WE NEED TO CHANGE THE NOISE VALUES
		R(0, 0) = 0.1; // GPS X
		R(1, 1) = 0.1; // GPS Y
		R(2, 2) = 0.1; // IMU X
		R(3, 3) = 0.1; // IMU Y

		// Compute weights for sigma points
        compute_weights();

    }

private:
    // UKF params
    float alpha, beta, kappa, lambda, k;
	float Ts; // Sampling time
	float phi; // acceleration smoothing factor
	float sigma_a; // acceleration noise standard deviation

    // States
	BLA::Matrix<n_x> x_hat; // State estimate
	BLA::Matrix<n_x, n_x> P; // State covariance matrix

    // Noise
	BLA::Matrix<n_x, n_x> Q; // Process noise covariance matrix
	BLA::Matrix<n_m, n_m> R; // Measurement noise covariance matrix

	// Sigma points
	BLA::Matrix<num_sigma_points, n_x> X_sigma; // Sigma points state matrix
	BLA::Matrix<num_sigma_points, n_m> Y_sigma; // Sigma points measurement matrix

    // Weights
	BLA::Matrix<num_sigma_points> W_m; // Weights for mean
	BLA::Matrix<num_sigma_points> W_c; // Weights for covariance

	// A and H matrices
    BLA::Matrix<n_x, n_x> A = { 1, 0, Ts, 0, 0, 0,
	                            0, 1, 0, Ts, 0, 0,
		                        0, 0, 1, 0, Ts, 0,
		                        0, 0, 0, 1, 0, Ts,
		                        0, 0, 0, 0, phi, 0,
		                        0, 0, 0, 0, 0, phi }; // State transition matrix

    BLA::Matrix<n_m, n_x> H = { 1, 0, 0, 0, 0, 0,
							    0, 1, 0, 0, 0, 0,
								0, 0, 0, 0, 1, 0,
								0, 0, 0, 0, 0, 1 }; // Measurement matrix

    // Function for process model
    BLA::Matrix<n_x, 1> process_model(const BLA::Matrix<n_x, 1>& x)
    {
        BLA::Matrix<n_x, 1> x_pred = A * x; // NOT SURE IF THERE SHOULD BE A NOISE TERM HERE
		return x_pred;
    }

	// Function for measurement model
    BLA::Matrix<n_m, 1> measurement_model(const BLA::Matrix<n_x, 1>& x)
    {
		BLA::Matrix<n_m, 1> y_pred = H * x; // NOT SURE IF THERE SHOULD BE A NOISE TERM HERE
		return y_pred;
    }

    void compute_weights()
    {
        // init to zero
        W_m.Fill(0);
        W_c.Fill(0);

        W_m(0) = lambda / (n_x + lambda);
        W_c(0) = lambda / (n_x + lambda) + 1 - std::pow(alpha, 2) + beta;

        for (std::uint8_t i = 1; i < num_sigma_points; ++i)
        {
			W_m(i) = 1 / (2 * (n_x + lambda));
			W_c(i) = 1 / (2 * (n_x + lambda));
        }
    }
};

bool referenceInitialized = false;
std::array<double, 2> referencePosition;

GNSSReader gnss;
IMUReader imu{ Wire, LSM6DS3_ADDRESS };

void setup()
{
    constexpr int baudRate{ 115200 };
    Serial.begin(baudRate);
    while (!Serial);
    Wire.begin(); // Initialize I2C
    if (!gnss.begin()) { Serial.println("Failed to initialize GNSS"); }
	if (!imu.begin()) { Serial.println("Failed to initialize IMU"); }
}

void loop() {
    if (gnss.updateCoordinates() && imu.updateAccelerometer()) {
        const GNSSCoordinates& coords = gnss.getCoordinates();
        if (!referenceInitialized) {
            referencePosition = { coords.latitude, coords.longitude };
            Serial.println("Reference position initialized: Lat=" + String(referencePosition[0]) + " Lon=" + String(referencePosition[1]));
            referenceInitialized = true;
            return;
        }

        const AccData& accData = imu.getAccData();
		Serial.println("Accelerometer data: X=" + String(accData.accX) + " Y=" + String(accData.accX));
       
        // Conversion to Cartesian
       
        std::array<double, 2> currentPosition = { coords.latitude, coords.longitude };
        std::array<double, 2> cartesian = wgs84::toCartesian(referencePosition, currentPosition);
        // gnss.printToSerial(currentPosition, cartesian, 10);

    }
}

