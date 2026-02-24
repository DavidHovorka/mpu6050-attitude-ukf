# mpu6050-attitude-ukf
Unscented Kalman Filter for real-time attitude estimation using the MPU6050 IMU.

---

The project is configured to work with the MPU6050 gyroscope + accelerometer and QMC5883P magnetometer, however the functions in **UKF.h** are easily portable to other sensors.


### Unscented Transform
The main advantage of an unscented kalman filter compared to its predecessor is the precision of applying non-linear transformations to gaussians. UKF's approach is simpler yet more precise than EKF's, which does it via jacobian matrices. On the other hand, UKF uses three meticulously selected and weighted points to represent the mean and covariance of the gaussian curve during the transformation. The unscented transform is exact to 2nd order for any nonlinear function and any distribution (and is exact to 3rd order for a gaussian distribution).

A simple visualization of the unscented transformation can be found here:
https://davidhovorka.github.io/unscented-transform-visualizer/

### Preferred Hardware
- MPU6050 IMU
- QMC5883P 3 axis magnetometer
- NRF24 for radio communication

## TO-DO:
- Include a calibration program
- Include 3D visualization script (perhaps create a new repo?)
- Find ways to increase performance