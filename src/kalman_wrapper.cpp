#include "ESKF.h"

static IMU_EKF::ESKF<float>* filter = nullptr;

extern "C" {

    // function for init
    void kalman_init(float ax, float ay, float az){
        filter = new IMU_EKF::ESKF<float>();
        filter->initWithAcc(ax, ay, az);
    }

    // function for update
    void kalman_update(float ax, float ay, float az, float gx, float gy, float gz, float dt){

        filter->predict(dt);
        filter->correctAcc(ax, ay, az);
        filter->correctGyr(gx, gy, gz);
        filter->reset();
    }



    // function for attitude
    void kalman_get_attitude(float *roll, float *pitch, float *yaw){

        filter->getAttitude(*roll, *pitch,  *yaw);
    }

}

