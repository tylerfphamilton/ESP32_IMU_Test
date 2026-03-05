#pragma once        // only compile once

// headers needed
#include <stdint.h>
#include <Eigen.h>
#include "Quaternion.h"

// there are 15 positions to track (3 position + 3 velocity + 3 acceleration + 3 orientation error + 3 angular velocity.)
#define STATE_SIZE 15

// Both gyro and accel give 3 values each (x, y, z)
#define MEASSUREMENT_GYR_SIZE 3
#define MEASSUREMENT_ACC_SIZE 3
// #define MEASSUREMENT_MAG_SIZE 3

// named scope
namespace IMU_EKF
{

template <typename precision>   // precision is a place holder for float or double
class ESKF
{
public:
    ESKF();     // calls init
    void init();    
    void initWithAcc(const float ax, const float ay, const float az);   
    void predict(precision dt);
    void correctGyr(const float gx, const float gy, const float gz);
    void correctAcc(const float ax, const float ay, const float az);
    void reset();

    Eigen::Matrix<precision, STATE_SIZE, 1> getState() const;
    void getAttitude(float &roll, float &pitch, float &yaw) const;
    Quaternion<precision> getAttitude() const;
    void getAcceleration(float &x, float &y, float &z) const;

private:
    Eigen::Matrix<precision, STATE_SIZE, 1> x_;                                        // The state vector — a 15x1 column of floats holding all the values the filter is currently estimating.
    Quaternion<precision> qref_;                                                       // The reference orientation quaternion — the filter's best current estimate of which way the IMU is pointing.
    Eigen::Matrix<precision, STATE_SIZE, STATE_SIZE> P_;                               // The covariance matrix — a 15x15 matrix tracking how uncertain the filter is about each part of the state. Starts as identity and evolves over time.
    Eigen::Matrix<precision, STATE_SIZE, STATE_SIZE> Q_;                               // Process noise matrix — the hardcoded tuning values. How much uncertainty to add each predict step.  
    Eigen::Matrix<precision, MEASSUREMENT_GYR_SIZE, MEASSUREMENT_GYR_SIZE> R_Gyr_;     // Measurement noise matrices — 3x3 each. How much the filter trusts the gyro and accel readings respectively. Also hardcoded tuning values.
    Eigen::Matrix<precision, MEASSUREMENT_ACC_SIZE, MEASSUREMENT_ACC_SIZE> R_Acc_;     // Measurement noise matrices — 3x3 each. How much the filter trusts the gyro and accel readings respectively. Also hardcoded tuning values.
};
} // namespace IMU_EKF

// ugly but necessary
// to get templates working
#include "ESKF.cpp"


/*

The loop in your `main.c` will essentially be:
```
initWithAcc(ax, ay, az)      // once at startup
loop:
    predict(dt)
    correctGyr(gx, gy, gz)
    correctAcc(ax, ay, az)
    reset()
    getAttitude(roll, pitch, yaw)


*/