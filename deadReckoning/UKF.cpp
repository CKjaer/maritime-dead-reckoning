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

    UKF(float theta = 0.0f, float alpha = 1.0f, float beta = 2.0f, float kappa = 2.0f, float Ts = 0.01f, 
        float phi = 0.95f, float sigma_a = 0.2f)
        : theta(theta * (180.0f / M_PI)), alpha(alpha), beta(beta), kappa(kappa), Ts(Ts), phi(phi), sigma_a(sigma_a)
      {
        lambda = std::pow(alpha, 2) * (static_cast<float>(n_x) + kappa) - static_cast<float>(n_x); // Scaling parameter
        k = std::sqrt(static_cast<float>(n_x) + lambda);

        // Initialize state and covariance matrix
        x_hat.Fill(0.f); //

        // Initialize covariance matrix as identity matrix
        P.Fill(0.0f);
        for(size_t i = 0; i < n_x; ++i) {
            P(i, i) = 1.0f;
        }
        // Process noise covariance matrix
        float sigma_qa = sigma_a * (1 - phi); 
       
        // Add noise to for only acceleration in the state space
        Q.Fill(0);
        Q(4, 4) = sigma_qa; 
        Q(5, 5) = sigma_qa;

        // Initialize measurement noise covariance matrix 
        R(0, 0) = std::pow(sigma_gnss_x, 2); 
        R(1, 1) = std::pow(sigma_gnss_y, 2); 
        R(2, 2) = std::pow(sigma_imu_x, 2);    
        R(3, 3) = std::pow(sigma_imu_y, 2);     

        computeWeights();
        setHeading(theta);
    }
    
    void timeUpdate()
    {
        createSigmaPoints();

        // Apply process model to sigma points
        X_sigma = A * X_sigma; 

        // Compute predicted state mean
        x_hat = X_sigma * W_m; 

        // Compute predicted state covariance Cov(x)
        BLA::Matrix<n_x> x_diff;
        for(size_t i{ 0 }; i < num_sigma_vec; ++i)
        {
            x_diff = X_sigma.Column(i) - x_hat;
            P += W_c(i) * x_diff * (~x_diff);
        }

        P += Q; // Add process noise covariance to state covariance
    }

    void measurementUpdate(BLA::Matrix<n_m>& y_k)
    {
        // Rotate measurements to global frame
        y_k = RotM * y_k;
        
        // Apply measurement model to sigma points
        Y_sigma = H * X_sigma; 
        
        // Compute predicted measurement mean
        BLA::Matrix<n_m> y_hat;
        y_hat = Y_sigma * W_m; 

        // Compute predicted measurement covariance Cov(y)
        BLA::Matrix<n_m> y_diff;
        BLA::Matrix<n_x> x_diff;
        BLA::Matrix<n_m, n_m> P_yy;
        BLA::Matrix<n_x, n_m> P_xy;
        P_yy.Fill(0.0f);
        P_xy.Fill(0.0f);

        for(size_t i{ 0 }; i < num_sigma_vec; ++i) 
        {
            y_diff = Y_sigma.Column(i) - y_hat;
            P_yy += W_c(i) * y_diff * (~y_diff);
            
            x_diff = X_sigma.Column(i) - x_hat; 
            P_xy += W_c(i) * x_diff * (~y_diff);
        }

        P_yy += R; // Add measurement noise covariance

        // Compute Kalman gain
        auto P_yy_inv = BLA::Inverse(P_yy);       
        K = P_xy * P_yy_inv;
    
        // Compute residuals between reading predicted measurement 
        BLA::Matrix<n_m> y_tilde = y_k - y_hat;

        // Update state estimation
        x_hat = x_hat + K * y_tilde;

        // Update state covariance
        P = P - K * P_yy * ~K;
    }
    
    std::array<double, 2> getPosition() const { 
        return { static_cast<double>(x_hat(0)), static_cast<double>(x_hat(1)) };
    }

    void setGNSSNoise(float scale)
    {
        this->sigma_gnss_x *= scale;
        this->sigma_gnss_y *= scale;
        updateMeasurementNoise();
    }

    void setHeading(float theta)
    {
        this->theta = theta * (180.0f / M_PI);
        updateRotation();
    }

    void simulateOutage(const int numMeasurements) {
        static size_t count{ 1 };
        static bool offline{ false };

        if(count % numMeasurements == 0) {
          offline = !offline;
          if(offline) {
            setGNSSNoise(10.0f);          
          } else {
            setGNSSNoise(0.25f);
          }
        }    
      count++;
    }

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

    BLA::Matrix<4,4> RotM; // Rotation matrix

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

    void computeWeights()
    {
        W_m.Fill(0);
        W_c.Fill(0);

        W_m(0) = lambda / (n_x + lambda);
        W_c(0) = lambda / (n_x + lambda) + 1 - std::pow(alpha, 2) + beta;

        for (size_t i{ 1 }; i < num_sigma_vec; ++i)
        {
            W_m(i) = 1 / (2 * (n_x + lambda));
            W_c(i) = 1 / (2 * (n_x + lambda));
        }
    }

    void createSigmaPoints() {
        X_sigma.Column(0) = x_hat; // Set the first sigma points to the mean

        // Calculate the square root of the scaled covariance matrix 
        auto P_scaled = (n_x + lambda) * P;
        auto P_sqrt = BLA::CholeskyDecompose(P_scaled).L;

        for (size_t i{ 0 }; i < n_x; ++i) {
            X_sigma.Column(i + 1) = x_hat + P_sqrt.Column(i); // First half of columns
            X_sigma.Column(i + n_x + 1) = x_hat - P_sqrt.Column(i); // Second half of the columns
        }
    }

    void updateRotation()
    {
        float sin_theta = std::sin(theta);
        float cos_theta = std::cos(theta);
        RotM = {1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, cos_theta, -sin_theta, 
                0, 0, sin_theta, cos_theta};
    }
    
    void updateMeasurementNoise() {
        R(0, 0) = std::pow(sigma_gnss_x, 2); 
        R(1, 1) = std::pow(sigma_gnss_y, 2); 
    }
};