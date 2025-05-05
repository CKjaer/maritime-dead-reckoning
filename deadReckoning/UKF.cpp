#include <BasicLinearAlgebra.h>
#include <array>
#include <cstdint>

class UKF
{
public:
    static constexpr std::uint8_t n_x{ 6 }; // State vector size
    static constexpr std::uint8_t n_m{ 4 }; // Measurement vector size
    static constexpr std::uint8_t num_sigma_points{ 2 * n_x + 1 }; // No. sigma vectors

    UKF(float alpha = 1.0f, float beta = 2.0f, float kappa = 2.0f, float Ts = 0.01f, 
        float phi = 0.95f, float sigma_a = 0.2f, float theta = 0)
        : alpha(alpha), beta(beta), kappa(kappa), Ts(Ts), phi(phi), sigma_a(sigma_a), theta(theta)
      {
       
        lambda = std::pow(alpha, 2) * (n_x + kappa) - n_x; // Scaling parameter
        k = std::sqrt(n_x + lambda);

        // Initialize state and covariance matrix
        x_hat.Fill(0.f);
        P = { 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1 
        };

        // Process noise covariance matrix
        float sigma_qa = sigma_a * (1 - phi); 
       
        // Add noise to for only acceleration in the state space
        Q.Fill(0);
        Q(4, 4) = sigma_qa; 
        Q(5, 5) = sigma_qa;

        // Initialize measurement noise covariance matrix 
        R(0, 0) = std::pow(sigma_gnss_x, 2); // GPS X
        R(1, 1) = std::pow(sigma_gnss_y, 2); // GPS Y
        R(2, 2) = std::pow(0.006728, 2); // IMU X
        R(3, 3) = std::pow(0.006481, 2); // IMU Y

        computeWeights();
        setHeading(theta);
    }
    
    void timeUpdate()
    {
        createSigmaPoints();

        // Apply process model to sigma points
        X_sigma = A * X_sigma; // [n_x, n_x] * [n_x, num_sigma_points]

        // Compute predicted state mean
        x_hat = X_sigma * W_m; // [n_x, num_sigma_points] * [n_x, 1]

        // Compute predicted state covariance Cov(x)
        for (uint8_t i = 0; i < num_sigma_points; ++i) { // cols
            BLA::Matrix<n_x> x_diff;
            for (uint8_t j = 0; j < n_x; ++j) { // rows
                x_diff(j) = X_sigma(j, i) - x_hat(j); // Get difference between sigma point state vectors and mean
            }

            // Update P with weighted outer product
            P += W_c(i) * x_diff * (~x_diff);
        }

        P += Q; // Add process noise covariance to state covariance
    }

    void measurementUpdate(BLA::Matrix<n_m>& y_k)
    {
        // Rotate measurements to global frame
        y_k = RotM * y_k;

        // Apply measurement model to sigma points
        Y_sigma = H * X_sigma; // [n_m, n_x] * [n_x, num_sigma_points]

        // Compute predicted measurement mean
        BLA::Matrix<n_m> y_hat;
        y_hat = Y_sigma * W_m; // [n_m, num_sigma_points] * [num_sigma_points, 1]

        // Compute predicted measurement covariance Cov(y)
        BLA::Matrix<n_m> y_diff;
        BLA::Matrix<n_m, n_m> P_yy;
        for (std::uint8_t i = 0; i < num_sigma_points; ++i)
        {
            // Get difference between sigma point measurement vectors and mean
            for (std::uint8_t j = 0; j < n_m; ++j)
            {
                y_diff(j) = Y_sigma(i, j) - y_hat(j);
            }
            P_yy += W_c(i) * y_diff * (~y_diff);
        }

        P_yy += R; // Add measurement noise covariance to measurement covariance

        // Compute cross covariance Cov(x, y)
        BLA::Matrix<n_x, n_m> P_xy;
        for (std::uint8_t i = 0; i < num_sigma_points; ++i)
        {
            // Get difference between sigma point state vectors and mean state
            BLA::Matrix<n_x> x_diff;
            for (std::uint8_t j = 0; j < n_x; ++j)
            {
                x_diff(j) = X_sigma(i, j) - x_hat(j);
            }

            P_xy += W_c(i) * x_diff * (~y_diff); // outer product
        }

        // Compute Kalman gain
        auto P_yy_inv = BLA::Inverse(P_yy); // Precompute inverse of P_yy
        BLA::Matrix<n_x, n_m> K = P_xy * P_yy_inv; // Kalman gain

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

    void setGNSSNoise(float sigma_x, float sigma_y)
    {
        this->sigma_gnss_x = sigma_x;
        this->sigma_gnss_y = sigma_y;
    }

    void setHeading(float theta)
    {
        this->theta = theta;
        updateRotation();
    }

private:
    float alpha, beta, kappa, lambda, k;
    float Ts; // Sampling time
    float phi; // Acceleration smoothing factor
    float sigma_a; // Acceleration noise standard deviation
    float sigma_gnss_x = 2.461963; // Gnss noise parameters
    float sigma_gnss_y = 1.420877;

    float theta; // heading
    BLA::Matrix<4,4> RotM; // Rotation matrix

    // States
    BLA::Matrix<n_x> x_hat; // State estimate
    BLA::Matrix<n_x, n_x> P; // State covariance matrix

    // Noise
    BLA::Matrix<n_x, n_x> Q; // Process noise covariance matrix
    BLA::Matrix<n_m, n_m> R; // Measurement noise covariance matrix

    // Sigma points
    BLA::Matrix<n_x, num_sigma_points> X_sigma; // Sigma points state matrix
    BLA::Matrix<n_m, num_sigma_points> Y_sigma; // Sigma points measurement matrix

    // Weights
    BLA::Matrix<num_sigma_points> W_m; // Weights for mean
    BLA::Matrix<num_sigma_points> W_c; // Weights for covariance

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

        for (std::uint8_t i = 1; i < num_sigma_points; ++i)
        {
            W_m(i) = 1 / (2 * (n_x + lambda));
            W_c(i) = 1 / (2 * (n_x + lambda));
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

    void createSigmaPoints() {
        X_sigma.Column(0) = x_hat; // Set the first sigma points to the mean

        // Calculate the square root of the scaled covariance matrix 
        auto P_scaled = (n_x + lambda) * P;
        auto P_sqrt = BLA::CholeskyDecompose(P_scaled).L;

        for (std::uint8_t i{ 0 }; i < n_x; ++i) {
            X_sigma.Column(i + 1) = x_hat + P_sqrt.Column(i); // First half of columns
            X_sigma.Column(i + n_x + 1) = x_hat - P_sqrt.Column(i); // Second half of the columns
        }
    }
};