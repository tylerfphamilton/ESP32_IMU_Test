// got all this code from github: https://github.com/hobbeshunter/IMU_EKF/tree/master/src

// removed all the code relating to a mag for the IMU (only using a 6-axis IMU, not a 9-axis)

#ifndef ESKF_IMPL
#define ESKF_IMPL

#include "ESKF.h"
#include "utilities.h"

#include <Eigen/LU>
// #include <Wire.h>

#ifndef SGN
#define SGN(X) ((X > 0) - (X < 0))
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif

#ifndef MS2_TO_G
#define MS2_TO_G (1.0 / 9.81)
#endif

#ifndef G_TO_MS2
#define G_TO_MS2 (9.81)
#endif

namespace IMU_EKF
{

template <typename precision>
ESKF<precision>::ESKF()
{
    init();
}

// Sets all matrices to their starting values — zeroes the state, sets identity matrices, loads the hardcoded Q/R noise values.
template <typename precision>
void ESKF<precision>::init()
{
    qref_ = Quaternion<precision>();
    x_.setZero();
    P_.setIdentity();
    Q_.setIdentity();
    // TODO don't hardcode this
    Q_(0, 0) = 0.05;
    Q_(1, 1) = 0.05;
    Q_(2, 2) = 0.05;
    Q_(3, 3) = 0.05;
    Q_(4, 4) = 0.05;
    Q_(5, 5) = 0.05;
    Q_(6, 6) = 0.025;
    Q_(7, 7) = 0.025;
    Q_(8, 8) = 0.025;
    Q_(9, 9) = 0.025;
    Q_(10, 10) = 0.025;
    Q_(11, 11) = 0.025;
    Q_(12, 12) = 0.01;
    Q_(13, 13) = 0.01;
    Q_(14, 14) = 0.01;

    R_Gyr_.setIdentity();
    R_Gyr_(0, 0) = 0.0000045494 * DEG_TO_RAD;
    R_Gyr_(1, 1) = 0.0000039704 * DEG_TO_RAD;
    R_Gyr_(2, 2) = 0.0000093844 * DEG_TO_RAD;

    R_Acc_.setIdentity();
    R_Acc_(0, 0) = 0.0141615383 * G_TO_MS2;
    R_Acc_(1, 1) = 0.0164647549 * G_TO_MS2;
    R_Acc_(2, 2) = 0.0100094303 * G_TO_MS2;

    // R_Mag_.setIdentity();
    // R_Mag_(0, 0) = 0.00004;
    // R_Mag_(1, 1) = 0.00004;
    // R_Mag_(2, 2) = 0.00004;
}

// uses first accelerometer reading to compute the actual starting roll and pitch. Call this instead of relying on the plain constructor.
template <typename precision>
void ESKF<precision>::initWithAcc(const float ax, const float ay, const float az)
{
    init();

    // see https://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
    // rotation sequence R = Rx * Ry * Rz
    // eq. 38
    precision roll = std::atan2(ay, SGN(-az) * std::sqrt(az * az + 0.01 * ax * ax));
    // eq. 37
    precision pitch = std::atan(-ax / std::sqrt(ay * ay + az * az));

    precision sr05 = std::sin(0.5 * roll);
    precision cr05 = std::cos(0.5 * roll);
    precision sp05 = std::sin(0.5 * pitch);
    precision cp05 = std::cos(0.5 * pitch);
    qref_ = Quaternion<precision>(sr05, 0, 0, cr05) * Quaternion<precision>(0, sp05, 0, cp05);
}

// called every loop iteration
// dt is the time in seconds since the last call.
// Advances the filter's internal state forward in time using the motion model — it's what causes the filter to "drift" slightly between corrections.
template <typename precision>
void ESKF<precision>::predict(precision dt)
{
    // eq. 23
    Quaternion<precision> angular_velocity_quat(x_[12], x_[13], x_[14], 0);
    qref_ += dt * ((0.5 * angular_velocity_quat) * qref_);
    qref_.normalize();

    Eigen::Matrix<precision, 9, 9> A;
    A.setIdentity();
    A(0, 3) = dt;
    A(0, 6) = dt * dt * 0.5;
    A(1, 4) = dt;
    A(1, 7) = dt * dt * 0.5;
    A(2, 5) = dt;
    A(2, 8) = dt * dt * 0.5;
    A(3, 6) = dt;
    A(4, 7) = dt;
    A(4, 8) = dt;

    x_.segment(0, 9) = A * x_.segment(0, 9);
    P_.topLeftCorner(9, 9) = A * P_.topLeftCorner(9, 9) * A.transpose() + dt * Q_.topLeftCorner(9, 9);

    // eq. 38
    Eigen::Matrix<precision, 3, 1> angular_velocity = x_.segment(12, 3);
    Eigen::Matrix<precision, 3, 1> error = x_.segment(9, 3);
    Eigen::Matrix<precision, 6, 6> Jac = Eigen::Matrix<precision, 6, 6>::Zero();
    Jac.topLeftCorner(3, 3) = toCrossMatrix<precision>(error);
    Jac.topRightCorner(3, 3) = -toCrossMatrix<precision>(angular_velocity);
    // eq. 39
    Eigen::Matrix<precision, 6, 6> G = Eigen::Matrix<precision, 6, 6>::Identity();
    G.bottomRightCorner(3, 3) *= -1;

    // eq. 33 adpated
    P_.bottomRightCorner(6, 6) = Jac * P_.bottomRightCorner(6, 6) * Jac.transpose() + G * dt * Q_.bottomRightCorner(6, 6) * G.transpose();
}

// Feeds a gyroscope reading into the filter to correct angular velocity. Takes degrees per second.
template <typename precision>
void ESKF<precision>::correctGyr(const float gx, const float gy, const float gz)
{
    // z
    Eigen::Matrix<precision, MEASSUREMENT_GYR_SIZE, 1> z;
    z << gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD;

    // Kalman Gain
    // K = P * H' (H * P * H' + V * R * V')^-
    // H = eye(3)
    Eigen::Matrix<precision, 3, MEASSUREMENT_GYR_SIZE> K;
    K = P_.bottomRightCorner(3, 3) * (P_.bottomRightCorner(3, 3) + R_Gyr_).inverse();

    // x = x + K * (z - H * x)
    x_.segment(12, 3) += K * (z - x_.segment(12, 3));

    // P = (I - KH)P
    Eigen::Matrix<precision, 3, 3> IKH = Eigen::Matrix<precision, 3, 3>::Identity();
    IKH -= K;

    P_.bottomRightCorner(3, 3) = IKH * P_.bottomRightCorner(3, 3);
}

// Feeds an accelerometer reading to correct tilt (roll/pitch). Takes values in g's — the filter converts them internally to m/s²
template <typename precision>
void ESKF<precision>::correctAcc(const float ax, const float ay, const float az)
{
    // z
    Eigen::Matrix<precision, MEASSUREMENT_ACC_SIZE, 1> z;
    z << ax * G_TO_MS2, ay * G_TO_MS2, az * G_TO_MS2;

    Eigen::Matrix<precision, 3, 1> gravity;
    gravity << 0.0, 0.0, -9.81;

    if (std::abs(z.norm() - 9.81) < 0.7)
    {
        // set acc to zero
        // z
        Eigen::Matrix<precision, 3, 1> z_acc = Eigen::Matrix<precision, 3, 1>::Zero();

        // Kalman Gain
        // K = P * H' (H * P * H' + V * R * V')^-
        // H = eye(3)
        Eigen::Matrix<precision, 3, 3> K_acc;
        K_acc = P_.block(6, 6, 3, 3) * (P_.block(6, 6, 3, 3) + R_Acc_).inverse();

        // x = x + K * (z - H * x)
        x_.segment(6, 3) += K_acc * (z_acc - x_.segment(6, 3));

        // P = (I - KH)P
        Eigen::Matrix<precision, 3, 3> IKH = Eigen::Matrix<precision, 3, 3>::Identity();
        IKH -= K_acc;

        P_.block(6, 6, 3, 3) = IKH * P_.block(6, 6, 3, 3);

        // correct tilt
        Eigen::Matrix<precision, 3, 1> vi = gravity;

        // eq. 42
        Eigen::Matrix<precision, 3, 1> error = x_.segment(9, 3);
        // Eigen::Matrix<precision, 3, 3> Aa = toRotationMatrix<precision>(error);
        Eigen::Matrix<precision, 3, 3> Aq = qref_.toRotationMatrix();
        Eigen::Matrix<precision, 3, 1> vb_pred = Aq * vi;

        // H
        // eq. 44
        Eigen::Matrix<precision, 3, 3> Ha = toCrossMatrix<precision>(vb_pred);

        // eq. 46
        Eigen::Matrix<precision, 3, 3> K;
        K = P_.block(9, 9, 3, 3) * Ha.transpose() * (Ha * P_.block(9, 9, 3, 3) * Ha.transpose() + R_Acc_).inverse();

        // eq. 47
        // h = vb_pred
        x_.segment(9, 3) += K * (z - vb_pred - Ha * error);

        // eq. 48
        P_.block(9, 9, 3, 3) -= K * Ha * P_.block(9, 9, 3, 3);
    }
    else
    {
        Eigen::Matrix<precision, 3, 1> vi = gravity + x_.segment(6, 3);

        // eq. 42
        Eigen::Matrix<precision, 3, 1> error = x_.segment(9, 3);
        Eigen::Matrix<precision, 3, 3> Aa = toRotationMatrix<precision>(error);
        Eigen::Matrix<precision, 3, 3> Aq = qref_.toRotationMatrix();
        Eigen::Matrix<precision, 3, 1> vb_pred = Aq * vi;

        // H
        // eq. 44
        Eigen::Matrix<precision, 3, 6> H;
        Eigen::Matrix<precision, 3, 3> Ha = toCrossMatrix<precision>(vb_pred);
        H.topLeftCorner(3, 3) = Aa * Aq;
        H.bottomRightCorner(3, 3) = Ha;

        // eq. 46
        Eigen::Matrix<precision, 6, 3> K;
        K = P_.block(6, 6, 6, 6) * H.transpose() * (H * P_.block(6, 6, 6, 6) * H.transpose() + R_Acc_).inverse();

        // eq. 47
        // h = v_bred
        x_.segment(6, 6) += K * (z - vb_pred - Ha * error);

        // eq. 48
        P_.block(6, 6, 6, 6) -= K * H * P_.block(6, 6, 6, 6);
    }
}

// Called after corrections to fold the orientation error back into the reference quaternion and zero out the error state
// Must call this every loop after your corrections or the filter accumulates error.
template <typename precision>
void ESKF<precision>::reset()
{
    // eq. 21
    qref_ = Quaternion<precision>(x_(9), x_(10), x_(11), 2.0) * qref_;
    // eq. 22
    qref_.normalize();
    x_(9) = 0.0;
    x_(10) = 0.0;
    x_(11) = 0.0;
}

// Returns the full raw 15-element state vector (probably won't use this directly)
template <typename precision>
Eigen::Matrix<precision, STATE_SIZE, 1> ESKF<precision>::getState() const
{
    return x_;
}

// gives roll, pitch, and yaw in radians after the filter runs. Note yaw will drift without a magnetometer since there's nothing to correct it.
template <typename precision>
void ESKF<precision>::getAttitude(float &roll, float &pitch, float &yaw) const
{
    Eigen::Matrix<precision, 3, 1> angles = qref_.toEulerAngles();
    roll = angles(0);
    pitch = angles(1);
    yaw = angles(2);
}

// Alternative output — same orientation but as a quaternion instead of Euler angles.
template <typename precision>
Quaternion<precision> ESKF<precision>::getAttitude() const
{
    return qref_;
}

// Returns the filter's estimated linear acceleration (gravity removed). Useful if you need motion detection.
template <typename precision>
void ESKF<precision>::getAcceleration(float &x, float &y, float &z) const
{
    x = x_(6);
    y = x_(7);
    z = x_(8);
}
} // namespace IMU_EKF

#endif // ESKF_IMPL