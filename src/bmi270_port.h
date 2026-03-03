#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "bmi2.h"      // from Bosch driver
#include "bmi270.h"    // from Bosch driver

typedef struct {
    i2c_port_t port;
    uint8_t addr;      // 7-bit I2C address (0x68)
} bmi270_i2c_ctx_t;

// Initialize device + enable accel+gyro + set configs
int8_t bmi270_esp_init(bmi270_i2c_ctx_t *ctx, struct bmi2_dev *dev);

// Read accel+gyro
int8_t bmi270_esp_read_ag(struct bmi2_dev *dev, struct bmi2_sens_axes_data *acc, struct bmi2_sens_axes_data *gyr);
