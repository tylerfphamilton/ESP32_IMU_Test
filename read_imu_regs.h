// #pragma once
// #include <stdint.h>
// #include "esp_err.h"
// #include "driver/i2c.h"
// #include "esp_log.h"



// #define I2C_PORT I2C_NUM_0

// /*
// Registers: 
// DATA8 - LSB for ax
// DATA9 - MSB for ax
// DATA10 - LSB for ay
// DATA11 - MSB for ay
// DATA12 - LSB for az
// DATA13 - MSB for az
// DATA14 - LSB for gx
// DATA15 - MSB for gx
// DATA16 - LSB for gy
// DATA17 - MSB for gy
// DATA18 - LSB for gz
// DATA19 - MSB for gz
// */

// esp_err_t imu_i2c_init(int sda_gpio, int scl_gpio, uint32_t clk_hz);
// // esp_err_t i2c_ping(i2c_port_t port, uint8_t addr);
// void i2c_scan(i2c_port_t port);
// esp_err_t read_regs(uint8_t start_reg, uint8_t *out, size_t len);
// int16_t concatinate(uint8_t msb, uint8_t lsb);
// esp_err_t read_accel_gyr(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
// esp_err_t bmi270_basic_init(void);