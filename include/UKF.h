#pragma once
#include <array>
#include <functional>
#include <vector>
#include "maths.h"

struct Params
{
    int L;
    float alpha;
    float beta;
    float kappa;
};

class UKF
{
public:
    static constexpr int L = 7;

    // qw, qx, qy, qz, wbx, wby, wbz
    using State = Vector<float, L>;

    UKF(const Params &p);

    void predict(const Vector<float, 3> &gyroReadings, float dt);
    void update_accelerometer(const Vector<float, 3> &accelerometer_readings);
    void update_magnetometer(const Vector<float, 3> &magnetometer_readings);

    void get_quaternion(Quaternion &q) const;
    void get_euler(float &roll, float &pitch, float &yaw) const;
    void get_radian(float &roll, float &pitch, float &yaw) const;

    void set_accelerometer_noise(float r);

    float compute_tilt_compensated_yaw(float mx, float my, float mz, float roll, float pitch);

    void get_euler_statistics(float &roll, float &pitch, float &yaw, float &std_roll, float &std_pitch, float &std_yaw);

    void quat_to_euler(const Quaternion &q, float &roll, float &pitch, float &yaw);

private:
    static constexpr int _n_sigma = (2 * L) + 1; // 15

    void compute_sigma_points();
    
    void compute_weights();

    State _x;

    float _alpha;
    float _beta;
    float _kappa;

    Matrix<float, 7, 7> _Q; // process noise
    Matrix<float, 3, 3> _R; // accelerometer measurement noise
    Matrix<float, 7, 7> _P; // covariance matrix

    Vector<float, 3> _g{0.0f, 0.0f, 1.0f};

    Vector<State, _n_sigma> sigma_points;

    Vector<float, _n_sigma> weights_for_mean;

    Vector<float, _n_sigma> weights_for_covariance;

    float _lambda;
    
    Vector<State, _n_sigma> _propagated_states;

    float _base_weight;
};