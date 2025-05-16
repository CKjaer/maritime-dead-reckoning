#pragma once
#include <cmath>
#include <BasicLinearAlgebra.h>
#include <array>
#include <cstdint>

class UKF
{
public:
    static constexpr int n_x{ 6 };  // State vector dimension
    static constexpr int n_m{ 4 };  // Measurement vector dimension
    static constexpr int num_sigma_vec{ 2 * n_x + 1 };

    UKF(float theta = 0.0f, float alpha = 1.0f, float beta = 0.0f, float kappa = 2.0f, float Ts = 1.0f,
        float phi = 0.95f, float sigma_a = 0.2f);

    void timeUpdate();
 
    void measurementUpdate(BLA::Matrix<n_m>& y_k);
 
    std::array<double, 2> getPosition() const;

    void setGNSSNoise(float scale);
    
    void setHeading(float theta);

    void simulateOutage(const int numMeasurements);

private:
    float lambda;   // Scaling parameter
    float alpha;    // Sigma point spread
    float beta;     // Distribution prior parameter
    float kappa;    // Secondary scaling parameter
    float k;        // Sigma point distance factor
    float Ts;       // Sampling time
    float phi;      // Acceleration smoothing factor
    float sigma_a;  // Acceleration noise standard deviation
    float theta;    // Compass heading

    // Standard deviation measured for sensors
    float sigma_gnss_x{ 2.461963 };
    float sigma_gnss_y{ 1.420877 };
    float sigma_imu_x{ 0.006728 };
    float sigma_imu_y{ 0.006481 };

    BLA::Matrix<4, 4> RotM; // Rotation matrix

    // States
    BLA::Matrix<n_x> x_hat;  // State estimate
    BLA::Matrix<n_x, n_x> P; // State covariance matrix

    // Noise
    BLA::Matrix<n_x, n_x> Q; // Process noise covariance matrix
    BLA::Matrix<n_m, n_m> R; // Measurement noise covariance matrix

    // Sigma points
    BLA::Matrix<n_x, num_sigma_vec> X_sigma; // Sigma points state matrix
    BLA::Matrix<n_m, num_sigma_vec> Y_sigma; // Sigma points measurement matrix

    BLA::Matrix<n_x, n_m> K;  // Kalman gain

    // Weights
    BLA::Matrix<num_sigma_vec> W_m; // Weights for mean
    BLA::Matrix<num_sigma_vec> W_c; // Weights for covariance

    BLA::Matrix<n_x, n_x> A; // State transition matrix
    BLA::Matrix<n_m, n_x> H; // Measurement matrix

    void computeWeights();

    void createSigmaPoints();

    void updateRotation();

    void updateMeasurementNoise();

};