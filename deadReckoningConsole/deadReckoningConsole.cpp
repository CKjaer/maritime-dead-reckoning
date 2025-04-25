#include <Wire.h> // Required for I2C 
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> 
#include <Arduino_LSM6DS3.h> // Required for IMU
#include <iostream>
#include <BasicLinearAlgebra.h>
#include <cmath>
#include <cstdint>
#include <vector>
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


// Unscented Kalman Filter class
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
