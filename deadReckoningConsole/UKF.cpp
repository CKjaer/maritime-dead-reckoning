// Unscented Kalman Filter class
class UKF
{
public:
    static constexpr std::uint8_t n_x{ 6 }; // State vector size
    static constexpr std::uint8_t n_m{ 4 }; // Measurement vector size
    static constexpr std::uint8_t num_sigma_points{ 2 * n_x + 1 }; // Number of sigma points

    // Constructor
    UKF(float alpha, float beta, float kappa, float Ts, float phi, float sigma_a)
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
        R(0, 0) = std::pow(2.461963, 2); // GPS X
        R(1, 1) = std::pow(1.420877, 2); // GPS Y
        R(2, 2) = std::pow(0.006728, 2); // IMU X
        R(3, 3) = std::pow(0.006481, 2); // IMU Y

        // Compute weights for sigma points
        compute_weights();

    }

    // Perform a step of the UKF
    void step(const BLA::Matrix<n_m>& y_k)
    {
        timeUpdate();
        measurementUpdate(y_k);
    }

    // Getter for the state estimate
    BLA::Matrix<n_x> getState() const
    {
        return x_hat;
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
    BLA::Matrix<n_x, num_sigma_points> X_sigma; // Sigma points state matrix
    BLA::Matrix<n_m, num_sigma_points> Y_sigma; // Sigma points measurement matrix

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

    // Containers for sigma points
    BLA::Matrix<num_sigma_points, n_x> X; // After pass through of process model
    BLA::Matrix<num_sigma_points, n_x> Y; // After pass thorugh of measurement model

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

    void createSigmaPoints() {
        // Set the first sigma points to the mean
        X_sigma.Column(0) = x_hat;

        // Calculate the square root of the scaled covariance matrix 
        auto P_scaled = (n_x + lambda) * P;
        auto P_sqrt = BLA::CholeskyDecompose(P_scaled).L;

        for (std::uint8_t i{ 0 }; i < n_x; ++i) {
            X_sigma.Column(i + 1) = x_hat + P_sqrt.Column(i) // First half of columns
            X_sigma.Column(i + n_x + 1) = x_hat + P_sqrt.Column(i) // Second half of the columns
        }
    }

    void timeUpdate()
    {
        createSigmaPoints();

        // Apply process model to sigma points
        X_sigma = A * X_sigma; // [num_sigma_points, n_x] * [n_x, n_x]

        // Compute predicted state mean
        x_hat = X_sigma * W_m; // [num_sigma_points, n_x] * [n_x, 1]

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

    void measurementUpdate(const BLA::Matrix<n_m> y_k)
    {
        // Apply measurement model to sigma points
        Y_sigma = H * X_sigma; // [n_m, n_x] * [n_x, num_sigma_points]

        // Compute predicted measurement mean
        BLA::Matrix<n_m> y_hat; // predicted measurement mean
        y_hat = Y_sigma * W_m; // [n_m, num_sigma_points] * [num_sigma_points, 1]

        // Compute predicted measurement covariance Cov(y)
        BLA::Matrix<n_m> y_diff;
        BLA::Matrix<n_m, n_m> P_yy;
        for (std::uint8_t i = 0; i < num_sigma_points; ++i)
        {
            // get difference between sigma point measurement vectors and mean
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
            // get difference between sigma point state vectors and mean state
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

        // Get residuals
        BLA::Matrix<n_m> y_tilde;
        y_tilde = y_k - y_hat; // residual (diff between reading and predicted measurement)

        // Update state estimation
        x_hat = x_hat + K * y_tilde;

        // Update state covariance
        P = P - K * (~P_xy); // ~P_xy = P_yx
    }
};