#include "UKF.h"
#include <cstring>
#include "const.h"

constexpr float ACC_NOISE {0.01f};
constexpr float MAG_NOISE {0.02f};

// constructor
UKF::UKF(const Params &p)
    :
      _alpha(p.alpha),
      _beta(p.beta),
      _kappa(p.kappa),
      _x{{1, 0, 0, 0, 0, 0, 0}},
      _P{{{1e-3, 0, 0, 0, 0, 0, 0}, {0, 1e-3, 0, 0, 0, 0, 0}, {0, 0, 1e-3, 0, 0, 0, 0}, {0, 0, 0, 1e-3, 0, 0, 0}, {0, 0, 0, 0, 1e-4, 0, 0}, {0, 0, 0, 0, 0, 1e-4, 0}, {0, 0, 0, 0, 0, 0, 1e-4}}},
      _Q{{{1e-6, 0, 0, 0, 0, 0, 0}, {0, 1e-6, 0, 0, 0, 0, 0}, {0, 0, 1e-6, 0, 0, 0, 0}, {0, 0, 0, 1e-6, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}}},
      _R{{{ACC_NOISE, 0.0, 0.0}, {0.0, ACC_NOISE, 0.0}, {0.0, 0.0, ACC_NOISE}}}
{
    _lambda = _alpha * _alpha * (L + _kappa) - L;
    _base_weight = 1.0f / (2.0f * (L + _lambda));

    compute_weights();
}

void UKF::compute_weights()
{
    // $$\lambda = \alpha^2 (L + \kappa) - L$$
    
    // $$W_m^{(0)} = \frac{\lambda}{L + \lambda}$$

    // $$W_c^{(0)} = \frac{\lambda}{L + \lambda} + (1 - \alpha^2 + \beta)$$

    // $$W_m^{(i)} = W_c^{(i)} = \frac{1}{2(L + \lambda)}$$


    weights_for_mean[0] = _lambda / (L + _lambda);
    weights_for_covariance[0] = _lambda / (L + _lambda) + (1 - _alpha * _alpha + _beta);

    for (std::size_t i = 1; i < _n_sigma; i++) // tady counter začínal na i = 0, ale nedavalo mi to smysl se zbytkem funkce
    {
        weights_for_mean[i] = _base_weight;
        weights_for_covariance[i] = _base_weight;
    }
}


/*
//
// Create sigma points, then propagate them through the process model.
//
*/
void UKF::predict(const Vector<float, 3>& gyro_readings, float dt)
{
    compute_sigma_points();

    // propagate each sigma through process model, create an array of states
    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        // separate quaternion and bias from each sigma point
        Quaternion sigma_point_q {sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]};
        Vector<float, 3> sigma_point_bias {sigma_points[i][4], sigma_points[i][5], sigma_points[i][6]};

        // substract bias from each gyro reading, this is basically useless as far as I know (bias is 0)
        // $$\boldsymbol{\omega}^{(i)} = \boldsymbol{\omega}_{meas} - \mathbf{b}^{(i)}$$
        Vector<float, 3> omega = gyro_readings - sigma_point_bias;

        // $$\Delta\mathbf{q}^{(i)} = \text{integrate}(\boldsymbol{\omega}^{(i)}, \Delta t)$$
        Quaternion dq {integrate_omega(omega, dt)};

        // $$\mathbf{q}_{prop}^{(i)} = \frac{\mathbf{q}^{(i)} \otimes \Delta\mathbf{q}^{(i)}}{\| \mathbf{q}^{(i)} \otimes \Delta\mathbf{q}^{(i)} \|}$$

        // propagate sigma point through process model and normalize (not optimal)
        Quaternion q_rotated {sigma_point_q * dq};
        normalize_quaternion(q_rotated);

        // attach to list of all predicted states
        _propagated_states[i][0] = q_rotated.w;
        _propagated_states[i][1] = q_rotated.x;
        _propagated_states[i][2] = q_rotated.y;
        _propagated_states[i][3] = q_rotated.z;
        _propagated_states[i][4] = sigma_point_bias[0];
        _propagated_states[i][5] = sigma_point_bias[1];
        _propagated_states[i][6] = sigma_point_bias[2];
    }

    // weighted average of all 14 sigma points
    State predicted_mean {0};

// $$\hat{\mathbf{x}}_{k|k-1} = \sum_{i=0}^{2L} W_m^{(i)} \mathcal{X}_{k|k-1}^{(i)}$$


// $$\hat{\mathbf{x}}_{k|k-1} = \sum_{i=0}^{2L} W_m^{(i)} \mathcal{X}_{prop}^{(i)}$$

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        for (std::size_t j = 0; j < L; j++)
        {
            predicted_mean[j] += weights_for_mean[i] * _propagated_states[i][j];
        }
    }

    // normalize the quaternion part of x_predicted
    normalize_quaternion(*reinterpret_cast<Quaternion*>(&predicted_mean[0]));

    Matrix<float, L, L> predicted_covariance {_Q};


    // $$P_{k|k-1} = \mathbf{Q} + \sum_{i=0}^{2L} W_c^{(i)} \left( \tilde{\mathbf{x}}^{(i)} \right) \left( \tilde{\mathbf{x}}^{(i)} \right)^T$$


    // $$\mathbf{P}_{k|k-1} = \mathbf{Q} + \sum_{i=0}^{2L} W_c^{(i)} \left( \mathcal{X}_{prop}^{(i)} - \hat{\mathbf{x}}_{k|k-1} \right) \left( \mathcal{X}_{prop}^{(i)} - \hat{\mathbf{x}}_{k|k-1} \right)^T$$


    // Covariance matrix is symmetrical, therefore:
    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Vector<float, L> diff;
        for (std::size_t j = 0; j < L; j++)
        {
            diff[j] = _propagated_states[i][j] - predicted_mean[j];
        }

        for (std::size_t a = 0; a < L; a++)
        {
            // Precompute constant
            float weight_x_diff_a = weights_for_covariance[i] * diff[a]; 
            
            // start from "a" ([a][a] = diagonal) and continue with the upper triangle
            for (std::size_t b = a; b < L; b++) 
            {
                float val {weight_x_diff_a * diff[b]};
                predicted_covariance[a][b] += val;
                
                // Mirror to lower triangle (if not on a diagonal)
                if (a != b) {
                    predicted_covariance[b][a] += val;
                }
            }
        }
    }

    sigma_points = _propagated_states;

    // save covariance matrix and state
    _P = predicted_covariance;
    _x = predicted_mean;
}

// Update using accelerometer data, adaptive noise variation
void UKF::update_accelerometer(const Vector<float, 3> &accelerometer_readings)
{
    float ax {accelerometer_readings[0]};
    float ay {accelerometer_readings[1]};
    float az {accelerometer_readings[2]};
    float acc_magnitude {std::sqrt(ax * ax + ay * ay + az * az)};

    // $$\Delta g = \left| \|\mathbf{a}_{meas}\| - 1.0 \right|$$
    float gravity_deviation {std::abs(acc_magnitude - 1.0f)};

    float base_R = {0.005f};
    float vibration_sensitivity {50.0f};

    // $$R_{adapt} = R_{base} + \Delta g^2 \cdot f_{adapt}$$

    float adaptive_R {base_R + (gravity_deviation * gravity_deviation * vibration_sensitivity)};

    Matrix<float, _n_sigma, 3> z_sigma;

    // convert sigma points to measurement space
    // $$\mathcal{Z}^{(i)} = (\mathbf{q}^{(i)})^* \otimes \mathbf{g} \otimes \mathbf{q}^{(i)}$$
    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Quaternion sigma_point_q{sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]};
        //normalize_quaternion(sigma_point_q);

        Quaternion sigma_point_q_conj{sigma_point_q.w, -sigma_point_q.x, -sigma_point_q.y, -sigma_point_q.z};
        
        z_sigma[i] = rotate_vector_by_quaternion(_g, sigma_point_q_conj);
    }

    // $$\hat{\mathbf{z}} = \sum_{i=0}^{2L} W_m^{(i)} \mathcal{Z}^{(i)}$$

    Vector<float, 3> z_mean{0, 0, 0};

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        for (std::size_t k = 0; k < 3; k++)
            z_mean[k] += weights_for_mean[i] * z_sigma[i][k];
    }

    // measurement covariance S

    // $$\mathbf{S} = \mathbf{R}_{adapt} + \sum_{i=0}^{2L} W_c^{(i)} \left( \mathcal{Z}^{(i)} - \hat{\mathbf{z}} \right) \left( \mathcal{Z}^{(i)} - \hat{\mathbf{z}} \right)^T$$

    Matrix<float, 3, 3> S;
    for (std::size_t i = 0; i < 3; i++)
    {
        for (std::size_t j = 0; j < 3; j++)
        {
            // použití adaptivní variance na diagonále
            S[i][j] = (i == j) ? adaptive_R : 0.0f;
        }
    }

    for (std::size_t i = 0; i < _n_sigma; i++) // projde všechny sigma pointy
    {
        Vector<float, 3> z_diff;
        for (std::size_t k = 0; k < 3; k++) // projde 3 hodnoty sigma pointu
            z_diff[k] = z_sigma[i][k] - z_mean[k];

        for (std::size_t a = 0; a < 3; a++)
            for (std::size_t b = 0; b < 3; b++)
                S[a][b] += weights_for_covariance[i] * z_diff[a] * z_diff[b];
    }

    // cross covariance Pxz

    // $$\mathbf{P}_{xz} = \sum_{i=0}^{2L} W_c^{(i)} \left( \mathcal{X}^{(i)} - \hat{\mathbf{x}}_{k|k-1} \right) \left( \mathcal{Z}^{(i)} - \hat{\mathbf{z}} \right)^T$$

    Matrix<float, L, 3> Pxz{};

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Vector<float, L> x_diff;
        for (std::size_t a = 0; a < L; a++)
            x_diff[a] = sigma_points[i][a] - _x[a];

        Vector<float, 3> z_diff;
        for (std::size_t k = 0; k < 3; k++)
            z_diff[k] = z_sigma[i][k] - z_mean[k];

        for (std::size_t a = 0; a < L; a++)
            for (std::size_t b = 0; b < 3; b++)
                Pxz[a][b] += weights_for_covariance[i] * x_diff[a] * z_diff[b];
    }

    Matrix<float, 3, 3> Sinv;
    if (!mat3x3_inverse(S, Sinv))
        return; // matrix is singular, something went wrong

    // Kalman gain K

    // $$\mathbf{K} = \mathbf{P}_{xz} \mathbf{S}^{-1}$$

    Matrix<float, L, 3> K {Pxz * Sinv}; // matrix multiplication

    // Residual
    Vector<float, 3> residual {accelerometer_readings - z_mean};

    // Update State
    Vector<float, 7> dx {K * residual}; // matrix vector multiplication

    for(int i = 0; i<L; i++){
        _x[i] += dx[i];
    }
    // normalize the quaternion part of _x
    normalize_quaternion(*reinterpret_cast<Quaternion*>(&_x[0]));

    Matrix<float, L, 3> KS {K * S}; // matrix multiplication

    // $$\mathbf{P}_{k} = \mathbf{P}_{k|k-1} - \mathbf{K} \mathbf{S} \mathbf{K}^T$$

    _P = _P - (KS * transpose_matrix(K));
}

void UKF::update_magnetometer(const Vector<float, 3> &magnetometer_readings)
{
    float mx {magnetometer_readings[0]};
    float my {magnetometer_readings[1]};
    float mz {magnetometer_readings[2]};

    float mag_magnitude = std::sqrt(mx * mx + my * my + mz * mz);
    
    Vector<float, 3> m_normalized {0, 0, 0};
    if (mag_magnitude > 0.0f) 
    {
        m_normalized[0] = mx / mag_magnitude;
        m_normalized[1] = my / mag_magnitude;
        m_normalized[2] = mz / mag_magnitude;
    } 
    else 
    {
        return;
    }


    // $$\mathbf{m}_{earth\_raw} = \mathbf{q}_{state} \otimes \frac{\mathbf{m}_{meas}}{\|\mathbf{m}_{meas}\|} \otimes (\mathbf{q}_{state})^*$$

    Quaternion q_state {_x[0], _x[1], _x[2], _x[3]};
    normalize_quaternion(q_state); // state should always be normalized but whatever...

    // body frame -> earth frame
    // rotate state by magnetometer data

    Vector<float, 3> q_x_earth = rotate_vector_by_quaternion(m_normalized, q_state);



    // $$\mathbf{m}_{earth} = \begin{bmatrix} \sqrt{(m_{earth\_raw}^x)^2 + (m_{earth\_raw}^y)^2} \\ 0 \\ m_{earth\_raw}^z \end{bmatrix}$$


    Vector<float, 3> earth_mag_ref {std::sqrt(q_x_earth[0]*q_x_earth[0] + q_x_earth[1]*q_x_earth[1]), 0.0f, q_x_earth[2]};

    compute_sigma_points();

    Matrix<float, _n_sigma, 3> z_sigma;

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Quaternion sigma_q {sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]};
        normalize_quaternion(sigma_q);
        
        Quaternion sigma_q_conj {sigma_q.w, -sigma_q.x, -sigma_q.y, -sigma_q.z};
        
        // predicted measurement for this sigma point
        z_sigma[i] = rotate_vector_by_quaternion(earth_mag_ref, sigma_q_conj);
    }

    // Mean predicted measurement
    Vector<float, 3> z_mean {0, 0, 0};
    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        for (std::size_t k = 0; k < 3; k++)
            z_mean[k] += weights_for_mean[i] * z_sigma[i][k];
    }

    // measurement covariance
    Matrix<float, 3, 3> S;
    //float mag_noise_var = 0.05f; // magnetometer noise
    
    for (std::size_t i = 0; i < 3; i++)
    {
        for (std::size_t j = 0; j < 3; j++)
        {
            S[i][j] = (i == j) ? MAG_NOISE : 0.0f;
        }
    }

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Vector<float, 3> z_diff;
        for (std::size_t k = 0; k < 3; k++)
            z_diff[k] = z_sigma[i][k] - z_mean[k];

        for (std::size_t a = 0; a < 3; a++)
            for (std::size_t b = 0; b < 3; b++)
                S[a][b] += weights_for_covariance[i] * z_diff[a] * z_diff[b];
    }


    Matrix<float, L, 3> Pxz{};
    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        Vector<float, L> x_diff;
        for (std::size_t a = 0; a < L; a++)
            x_diff[a] = sigma_points[i][a] - _x[a];

        Vector<float, 3> z_diff;
        for (std::size_t k = 0; k < 3; k++)
            z_diff[k] = z_sigma[i][k] - z_mean[k];

        for (std::size_t a = 0; a < L; a++)
            for (std::size_t b = 0; b < 3; b++)
                Pxz[a][b] += weights_for_covariance[i] * x_diff[a] * z_diff[b];
    }

    Matrix<float, 3, 3> Sinv;
    if (!mat3x3_inverse(S, Sinv))
        return; // matrix is singular

    Matrix<float, L, 3> K {Pxz * Sinv}; // kalman gain

    Vector<float, 3> residual {m_normalized - z_mean};
    
    Vector<float, 7> diff_x = K * residual;

    for(std::size_t i = 0; i < L; i++) 
    {
        _x[i] += diff_x[i];
    }
    
    normalize_quaternion(*reinterpret_cast<Quaternion*>(&_x[0]));

    Matrix<float, L, 3> KS {K * S};
    _P = _P - (KS * transpose_matrix(K));
}

void UKF::get_quaternion(Quaternion &q) const
{
    q.w = _x[0];
    q.x = _x[1];
    q.y = _x[2];
    q.z = _x[3];
}

void UKF::get_euler(float &roll, float &pitch, float &yaw) const
{
    float qw = _x[0], qx = _x[1], qy = _x[2], qz = _x[3];
    float sinr_cosp { 2.0f * (qw * qx + qy * qz)};
    float cosr_cosp {1.0f - 2.0f * (qx * qx + qy * qy)};
    roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp) * 180.0f / PI);

    float sinp = 2.0f * (qw * qy - qz * qx);

    if (std::fabs(sinp) >= 1.0f)
        pitch = -1.0f * static_cast<float>(std::copysign(PI / 2, sinp) * 180.0f / PI);
    else
        pitch = -1.0f * static_cast<float>(std::asin(sinp) * 180.0f / PI);

    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    yaw = -1.0f * static_cast<float>(std::atan2(siny_cosp, cosy_cosp) * 180.0f / PI);
}


// TAKE A LOOK AT THIS -1, probably cancels out somewhere in the code and looks pretty messy
void UKF::get_radian(float &roll, float &pitch, float &yaw) const

{
    float qw = _x[0], qx = _x[1], qy = _x[2], qz = _x[3];
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);

    roll = static_cast<float>(std::atan2(sinr_cosp, cosr_cosp));

    float sinp = 2.0f * (qw * qy - qz * qx);

    if (std::fabs(sinp) >= 1.0f)
        pitch = -1.0f * static_cast<float>(std::copysign(PI / 2.0f, sinp));
    else
        pitch = -1.0f * static_cast<float>(std::asin(sinp));
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);

    yaw = -1.0f * static_cast<float>(std::atan2(siny_cosp, cosy_cosp));
}


/*
//
// https://arxiv.org/pdf/2104.01958
//
*/
void UKF::compute_sigma_points()
{
    // compute sqrt((L+lambda)*P), where sqrt is cholesky decomposition

    Matrix<float, L, L> scaledP;

    
    // $$\mathbf{P}_{scaled} = (L + \lambda)\mathbf{P}_{k-1} + \epsilon \mathbf{I}$$

    for (std::size_t i = 0; i < L; i++)
    {
        for (std::size_t j = 0; j < L; j++)
        {
            scaledP[i][j] = _P[i][j] * (L + _lambda); // scaled copy of P
        }
    }

    /// possibly get rid of this???
    for (std::size_t i = 0; i < L; i++)
    {
        scaledP[i][i] += 1e-6; // safety factor --------------------------------------------------------------------
    }

    Matrix<float, L, L> Lmat;


    // $$\mathbf{A} \mathbf{A}^T = \mathbf{P}_{scaled}$$
    
    cholesky_7x7(scaledP, Lmat); // Lmat lower-triangular


    // $$\mathcal{X}^{(0)} = \mathbf{x}_{k-1}$$ 

    // first sigma variable is the mean
    // 0th row of sigma_points = state

    for (std::size_t i = 0; i < L; i++)
    {
        sigma_points[0][i] = _x[i];
    }

    // $$\mathcal{X}^{(i)} = \mathbf{x}_{k-1} + \mathbf{A}_i$$
    // $$\mathcal{X}^{(i+L)} = \mathbf{x}_{k-1} - \mathbf{A}_i$$

    for (std::size_t i = 0; i < L; i++)
    {
        // take ith column from Lmat
        Vector<float, L> col;
        for (std::size_t r = 0; r < L; r++)
            col[r] = Lmat[r][i];

        // 1 + i row = x + col

        // x + col

        for (std::size_t r = 0; r < L; r++)
            sigma_points[1 + i][r] = _x[r] + col[r];

        // 1 + L + i row = x - col

        // x - col

        for (std::size_t r = 0; r < L; r++)
            sigma_points[1 + L + i][r] = _x[r] - col[r];
    }
}

void UKF::quat_to_euler(const Quaternion &q, float &roll, float &pitch, float &yaw)
{
    float sinr_cosp {2.0f * (q.w * q.x + q.y * q.z)};
    float cosr_cosp {1.0f - 2.0f * (q.x * q.x + q.y * q.y)};
    roll = std::atan2(sinr_cosp, cosr_cosp) * 180.0f / PI;

    float sinp {2.0f * (q.w * q.y - q.z * q.x)};
    /*
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(PI / 2.0f, sinp) * 180.0f / PI;
    else
        pitch = std::asin(sinp) * 180.0f / PI;
    */
    pitch = std::asin(sinp) * 180.0f / PI;
    float siny_cosp {2.0f * (q.w * q.z + q.x * q.y)};
    float cosy_cosp {1.0f - 2.0f * (q.y * q.y + q.z * q.z)};
    yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0f / PI;

    // some weird inversion idk, can be fixed
    pitch *= -1.0f;
    yaw *= -1.0f;
}

void UKF::get_euler_statistics(float &roll, float &pitch, float &yaw, float &std_roll, float &std_pitch, float &std_yaw)
{

    // $$\sigma_c^2 = \sum_{i=0}^{2L} W_c^{(i)} (\Delta c^{(i)})^2$$

    compute_sigma_points();

    get_euler(roll, pitch, yaw);

    float var_r = 0, var_p = 0, var_y = 0;

    for (std::size_t i = 0; i < _n_sigma; i++)
    {
        float euler_roll, euler_pitch, euler_yaw;
        Quaternion q {sigma_points[i][0], sigma_points[i][1], sigma_points[i][2], sigma_points[i][3]};
        normalize_quaternion(q);
        quat_to_euler(q, euler_roll, euler_pitch, euler_yaw);

        float diff_roll {euler_roll - roll};
        float diff_pitch {euler_pitch - pitch};
        float diff_yaw {euler_yaw - yaw};

        if (diff_roll > 180.0f)
            diff_roll -= 360.0f;
        else if (diff_roll < -180.0f)
            diff_roll += 360.0f;

        if (diff_pitch > 180.0f)
            diff_pitch -= 360.0f;
        else if (diff_pitch < -180.0f)
            diff_pitch += 360.0f;

        if (diff_yaw > 180.0f)
            diff_yaw -= 360.0f;
        else if (diff_yaw < -180.0f)
            diff_yaw += 360.0f;

        // weighted average of squares
        var_r += weights_for_covariance[i] * diff_roll * diff_roll;
        var_p += weights_for_covariance[i] * diff_pitch * diff_pitch;
        var_y += weights_for_covariance[i] * diff_yaw * diff_yaw;
    }

    std_roll = std::sqrt(var_r);
    std_pitch = std::sqrt(var_p);
    std_yaw = std::sqrt(var_y);
}