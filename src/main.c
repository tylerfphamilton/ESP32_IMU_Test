#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "bmi270_port.h"  
#include "bmi2.h"
#include "bmi270.h"

// I2C Config
#define GPIO_SDA 8
#define GPIO_SCL 9
#define I2C_PORT I2C_NUM_0
#define BMI270_ADDR 0x68
#define I2C_FREQ_HZ 100000   

static const char *TAG = "IMU";

// I2C init
esp_err_t imu_i2c_init(int sda_gpio, int scl_gpio, uint32_t clk_hz)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_gpio,
        .scl_io_num = scl_gpio,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_hz,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void app_main(void) {

    // ESP_LOGI(TAG, "Starting BMI270 example");

    // Initialize I2C
    ESP_ERROR_CHECK(imu_i2c_init(GPIO_SDA, GPIO_SCL, I2C_FREQ_HZ));

    bmi270_i2c_ctx_t ctx = {
        .port = I2C_PORT,
        .addr = BMI270_ADDR
    };

    struct bmi2_dev dev = {0};

    int8_t rslt = bmi270_esp_init(&ctx, &dev);

    // ESP_LOGI(TAG, "bmi270_esp_init result = %d", rslt);

    // if (rslt != BMI2_OK) {
    //     ESP_LOGE(TAG, "BMI270 init failed!");
    //     while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // ESP_LOGI(TAG, "BMI270 initialized successfully");


    while (1){

        struct bmi2_sens_axes_data acc, gyr;
        rslt = bmi270_esp_read_ag(&dev, &acc, &gyr);

        if (rslt == BMI2_OK) {
            ESP_LOGI(TAG, "ACC: x=%d y=%d z=%d | GYR: x=%d y=%d z=%d", acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z);
        }
        else {
            ESP_LOGE(TAG, "Read failed: %d", rslt);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/*

    // // initializing esp spi
    // esp_err_t err;
    // err = imu_i2c_init(GPIO_SDA, GPIO_SCL, clk);
    // ESP_ERROR_CHECK(err);   // <-- critical

    // err = bmi270_basic_init();
    // ESP_ERROR_CHECK(err);

    // uint8_t err_reg = 0, status = 0;
    // read_regs(0x02, &err_reg, 1);   // often "ERR_REG"
    // read_regs(0x03, &status, 1);    // often "STATUS"
    // ESP_LOGI("IMU", "ERR_REG(0x02)=0x%02X STATUS(0x03)=0x%02X", err_reg, status);


    // i2c_scan(I2C_NUM_0);
    // ESP_LOGI(TAG, "Just scanned");

    // vTaskDelay(pdMS_TO_TICKS(1000));


    // declaring the variables to store the values inside of
    // int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;


    while(1){
    
        esp_err_t err = read_accel_gyr(&ax, &ay, &az, &gx, &gy, &gz);
        if (err != ESP_OK){
            ESP_LOGE(TAG, "There was an error in read_accel_gyr: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(TAG, "Here are the xyz values for the accelerometer: ax:%d ay:%d az:%d and gyroscope: gx:%d gy:%d gz:%d\n", ax,ay,az,gx,gy,gz); 
        vTaskDelay(pdMS_TO_TICKS(1000));
    
    }

*/



