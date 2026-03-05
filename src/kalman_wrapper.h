#pragma once

#ifdef __cplusplus
extern "C" {
#endif


void kalman_init(float ax, float ay, float az);
void kalman_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);
void kalman_get_attitude(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif