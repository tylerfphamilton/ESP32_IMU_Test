// #include "read_imu_regs.h"


// #define BMI270_ADDR 0x68     
// #define BMI270_DATA_8 0x0C 

// esp_err_t imu_i2c_init(int sda_gpio, int scl_gpio, uint32_t clk_hz){
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = sda_gpio,
//         .scl_io_num = scl_gpio,
//         .sda_pullup_en = GPIO_PULLUP_DISABLE,  // still use external pullups ideally
//         .scl_pullup_en = GPIO_PULLUP_DISABLE,
//         .master.clk_speed = clk_hz,
//         .clk_flags = 0,
//     };

//     esp_err_t err = i2c_param_config(I2C_PORT, &conf);
//     if (err != ESP_OK) return err;

//     return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
// }


// static esp_err_t i2c_ping(i2c_port_t port, uint8_t addr)
// {
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
//     i2c_master_stop(cmd);
//     esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(50));
//     i2c_cmd_link_delete(cmd);
//     return ret;
// }


// // 
// void i2c_scan(i2c_port_t port)
// {
//     ESP_LOGI("IMU", "Scanning I2C bus...");
//     for (uint8_t addr = 1; addr < 127; addr++) {
//         if (i2c_ping(port, addr) == ESP_OK) {
//             ESP_LOGI("IMU", "I2C device found at 0x%02X", addr);
//         }
//     }
// }




// // only parameter is regs for now. Going to read from an MCU for now and then 
// esp_err_t read_regs(uint8_t start_reg, uint8_t *out, size_t len){

//     // get the register values and store it in regs
//     // check for errors
//     return i2c_master_write_read_device(
//         I2C_PORT,
//         BMI270_ADDR,
//         &start_reg,
//         1,
//         out,
//         len,
//         pdMS_TO_TICKS(100)
//     );
// }

// // concatinates the msb and lsb into one value
// int16_t concatinate(uint8_t msb, uint8_t lsb){

//     return (int16_t)((((uint16_t) msb) << 8) | (uint16_t)lsb);
// }

// esp_err_t read_accel_gyr(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz){

//     uint8_t regs[12];

//     // call some function to read the values into regs variable and check return type
//     esp_err_t err = read_regs(BMI270_DATA_8, regs, sizeof(regs));
//     ESP_LOGI("IMU", "RAW: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
//          regs[0], regs[1], regs[2], regs[3], regs[4], regs[5],
//          regs[6], regs[7], regs[8], regs[9], regs[10], regs[11]);

//     uint8_t id = 0;
//     read_regs(0x00, &id, 1);
//     ESP_LOGI("IMU", "CHIP_ID reg0x00 = 0x%02X", id);

//     if (err != ESP_OK) return err;

//     // calling the helper funciton to concatinate the msb and lsb together
//     *ax = concatinate(regs[1],regs[0]);
//     *ay = concatinate(regs[3],regs[2]);
//     *az = concatinate(regs[5],regs[4]);
//     *gx = concatinate(regs[7],regs[6]);
//     *gy = concatinate(regs[9],regs[8]);
//     *gz = concatinate(regs[11],regs[10]);

//     ESP_LOGI("IMU", "finsihed being called in read_accel_gyr");

//     return ESP_OK;
// }



// esp_err_t write_reg(uint8_t reg, uint8_t val){
//     uint8_t buf[2] = { reg, val };
//     return i2c_master_write_to_device(I2C_PORT, BMI270_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(100));
// }


// esp_err_t bmi270_basic_init(void)
// {
//     esp_err_t err;

//     // 1) Soft reset (Bosch command register pattern)
//     err = write_reg(0x7E, 0xB6);
//     if (err != ESP_OK) return err;
//     vTaskDelay(pdMS_TO_TICKS(50));

//     // 2) Try enabling accel + gyro via power control
//     // NOTE: On many Bosch IMUs, PWR_CTRL is near 0x7D.
//     // We'll read-modify-write to be safe.
//     uint8_t pwr = 0;
//     err = read_regs(0x7D, &pwr, 1);
//     if (err != ESP_OK) return err;

//     // Common pattern: set accel_en and gyro_en bits
//     pwr |= 0x06; // (bit1 accel, bit2 gyro) — if wrong, we'll adjust after readback
//     err = write_reg(0x7D, pwr);
//     if (err != ESP_OK) return err;

//     vTaskDelay(pdMS_TO_TICKS(20));

//     // Read back to confirm it stuck
//     uint8_t pwr2 = 0;
//     err = read_regs(0x7D, &pwr2, 1);
//     if (err != ESP_OK) return err;
//     ESP_LOGI("IMU", "PWR_CTRL (0x7D) was 0x%02X now 0x%02X", pwr, pwr2);

//     return ESP_OK;
// }


// // int main(){

// //     // declaring the variables to store the values inside of
// //     int16_t ax, ay, az, gx, gy, gz;

// //     while (flag){

// //         read_accel_gyr(&ax, &ax, &ax, &ax, &ax, &ax);
// //         printf("Here are the xyz values for the accelerometer: ax:%d ay:%d az:%d and gyroscope: gx:%d gy:%d gz:%d\n", ax,ay,az,gx,gy,gz); 

// //         // if (){ flag = false;}
// //     }

// //     printf("exited the main loop\n");
// //     return 0;
// // }