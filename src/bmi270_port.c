#include "bmi270_port.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"  

// ---- Bosch callback signatures ----
// int8_t read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
// int8_t write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
// void delay_us(uint32_t period, void *intf_ptr);

static int8_t bosch_i2c_read(uint8_t reg, uint8_t *data, uint32_t len, void *intf_ptr) {
    bmi270_i2c_ctx_t *ctx = (bmi270_i2c_ctx_t*)intf_ptr;
    esp_err_t err = i2c_master_write_read_device(ctx->port, ctx->addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static int8_t bosch_i2c_write(uint8_t reg, const uint8_t *data, uint32_t len, void *intf_ptr) {
    bmi270_i2c_ctx_t *ctx = (bmi270_i2c_ctx_t*)intf_ptr;

    // Write buffer
    uint8_t buf[1 + 256];
    if (len > 256) return BMI2_E_INVALID_INPUT;

    buf[0] = reg;
    for (uint32_t i = 0; i < len; i++) buf[1 + i] = data[i];

    esp_err_t err = i2c_master_write_to_device(ctx->port, ctx->addr, buf, 1 + len, pdMS_TO_TICKS(100));
    return (err == ESP_OK) ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void bosch_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;
    // Bosch driver expects microseconds
    esp_rom_delay_us(period);
}

int8_t bmi270_esp_init(bmi270_i2c_ctx_t *ctx, struct bmi2_dev *dev) {
    if (!ctx || !dev) return BMI2_E_INVALID_INPUT;

    // Hook interface
    dev->intf = BMI2_I2C_INTF;
    dev->read = bosch_i2c_read;
    dev->write = bosch_i2c_write;
    dev->delay_us = bosch_delay_us;
    dev->intf_ptr = ctx;
    dev->read_write_len = 32;   // safe default; Bosch uses it for burst transfers
    dev->config_file_ptr = NULL;

    // 1) Bosch init
    int8_t rslt = bmi270_init(dev);
    if (rslt != BMI2_OK) return rslt;

    // 2) Enable accel + gyro
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi2_sensor_enable(sens_list, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    // 3) Configure accel + gyro (ODR/range/bw)
    struct bmi2_sens_config cfg[2];

    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;

    rslt = bmi2_get_sensor_config(cfg, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    // Accel config
    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_4G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;     // bandwidth / filter
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    // Gyro config
    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;     // bandwidth / filter
    cfg[1].cfg.gyr.filter_perf = BMI2_GYR_OSR4_MODE;

    rslt = bmi2_set_sensor_config(cfg, 2, dev);
    if (rslt != BMI2_OK) return rslt;

    // Give it a moment to start producing samples
    vTaskDelay(pdMS_TO_TICKS(20));

    return BMI2_OK;
}

int8_t bmi270_esp_read_ag(struct bmi2_dev *dev, struct bmi2_sens_axes_data *acc, struct bmi2_sens_axes_data *gyr) {
    struct bmi2_sens_data sens = {0};

    int8_t rslt = bmi2_get_sensor_data(&sens, dev);
    if (rslt != BMI2_OK) return rslt;

    if (acc) *acc = sens.acc;
    if (gyr) *gyr = sens.gyr;

    return BMI2_OK;
}


