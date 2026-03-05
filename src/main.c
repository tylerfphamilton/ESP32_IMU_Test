#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>


#include "bmi270_port.h"  
#include "bmi2.h"
#include "bmi270.h"
#include "kalman_wrapper.h"

// I2C Config
#define GPIO_SDA 8
#define GPIO_SCL 9
#define I2C_PORT I2C_NUM_0
#define BMI270_ADDR 0x68
#define I2C_FREQ_HZ 100000   

static const char *IMU_TAG = "IMU";
static const char *FILTERED_TAG = "FILTERED";
static const char *COMMAND_TAG = "COMMAND";


// Kalman Movement Implementation (these are the upper bounds other than fast, will act as a lower bound instead) - vals are in degrees
typedef enum {
    DEADZONE,
    SLOW,
    MED,
    FAST
} SPEED;

typedef enum {
    PITCH_STOPPED,
    FORWARDS,
    BACKWARDS
} PITCH_DIRECTION;

typedef enum {
    ROLL_STOPPED,
    LEFT,
    RIGHT
} ROLL_DIRECTION;

typedef struct {
    SPEED speed;
    PITCH_DIRECTION pitch_dir;
    ROLL_DIRECTION roll_dir;
    float rotational_movement;
} mapping_t;


static void get_direction_and_speed(float roll, float pitch, mapping_t *map){

    // checking for direction and setting type in mapping_t struct
    // TODO: emaill UCLA
    
    // converting to degrees
    float roll_deg = roll * (180 / M_PI);
    float pitch_deg = pitch * (180 / M_PI);

    if (pitch_deg < 0){
        map->pitch_dir = FORWARDS;
    }
    else if (pitch_deg > 0){
        map->pitch_dir = BACKWARDS;
    }
    else {
        map->pitch_dir = PITCH_STOPPED;
    }

    if (roll_deg < 0){
        map->roll_dir = LEFT;
    }
    else if (roll_deg > 0){
        map->roll_dir = RIGHT;
    }
    else {
        map->roll_dir = ROLL_STOPPED;
    }
    

    // getting absolute value
    float abs_roll_deg = fabsf(roll_deg);
    float abs_pitch_deg = fabsf(pitch_deg);

    float rotational_movement = ((abs_pitch_deg > abs_roll_deg) ? abs_pitch_deg : abs_roll_deg);
    map->rotational_movement = rotational_movement;

    // checking for speed and setting type in mapping_t struct
    if (rotational_movement < 10.00){
        map->speed = DEADZONE;
    }
    else if (rotational_movement < 20.00){
        map->speed = SLOW;
    }
    else if (rotational_movement < 35.00){
        map->speed = MED;
    }
    else {
        map->speed = FAST;
    }
}

static void direction_and_speed_to_string(char **pitch_dir, char **roll_dir, char **speed, mapping_t map){

    // for writing to dir char**
    if (map.roll_dir == ROLL_STOPPED){
        *roll_dir = "Stopped";
    }
    else if (map.roll_dir == LEFT){
        *roll_dir = "Left";
    }
    else if (map.roll_dir == RIGHT){
        *roll_dir = "Right";
    }

    if (map.pitch_dir == PITCH_STOPPED){
        *pitch_dir = "Stopped";
    }
    else if (map.pitch_dir == FORWARDS){
        *pitch_dir = "Forwards";
    }
    else {
        *pitch_dir = "Backwards";
    }
    
    
    // for writing to speed char**
    if (map.speed == DEADZONE){
        *speed = "Deadzone";
    }
    else if (map.speed == SLOW){
        *speed = "Slow";
    }
    else if (map.speed == MED){
        *speed = "Medium";
    }   
    else {
        *speed = "Fast";
    }
}


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

// // function for calculating distance moved based on ax, ay, az, and dt
// double magnitude_distance(float ax, float ay, float az, float dt){

//     double a = sqrt((double)((ax*ax) + (ay*ay) + (az*az)));


// }


void app_main(void) {

    // Initialize I2C
    ESP_ERROR_CHECK(imu_i2c_init(GPIO_SDA, GPIO_SCL, I2C_FREQ_HZ));

    bmi270_i2c_ctx_t ctx = {
        .port = I2C_PORT,
        .addr = BMI270_ADDR
    };

    struct bmi2_dev dev = {0};

    int8_t rslt = bmi270_esp_init(&ctx, &dev);

    ESP_LOGI(IMU_TAG, "BMI270 init result: %d", rslt);



    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    int64_t start_dt = esp_timer_get_time();
    int64_t end_dt;
    float delta_dt;

    bool init_flag_acc = true;

    while (1){

        end_dt = esp_timer_get_time();
        struct bmi2_sens_axes_data acc, gyr;
        rslt = bmi270_esp_read_ag(&dev, &acc, &gyr);

        ax = (float) (acc.x / (32768.0f / 4.0f));
        ay = (float) (acc.y / (32768.0f / 4.0f));
        az = (float) (acc.z / (32768.0f / 4.0f));

        gx = (float) (gyr.x / (32768.0f / 2000.0f));
        gy = (float) (gyr.y / (32768.0f / 2000.0f));
        gz = (float) (gyr.z / (32768.0f / 2000.0f));

        // g = m/s^2 and dps = degrees per second
        if (rslt == BMI2_OK) {

            // printing out raw values
            ESP_LOGI(IMU_TAG, "ACC: x=%.2fg y=%.2fg z=%.2fg | GYR: x=%.2fdps y=%.2fdps z=%.2fdps", ax, ay, az, gx, gy, gz);

            if (init_flag_acc && fabsf(az) > 0.1f){
                kalman_init(ax, ay, az);
                init_flag_acc = false;
            }
            else if (!init_flag_acc){

                //
                delta_dt = (float) (end_dt - start_dt) / 1000000.0f;
                kalman_update(ax, ay, az, gx, gy, gz, delta_dt);
                kalman_get_attitude(&roll, &pitch, &yaw);

                //
                mapping_t map = {0};
                get_direction_and_speed(roll, pitch, &map);

                // 
                char* pitch_dir;
                char* roll_dir;
                char* speed;
                direction_and_speed_to_string(&pitch_dir, &roll_dir, &speed, map);

                // printing out filtered values (might be innacurate based on how the board is on the rover)
                ESP_LOGI(FILTERED_TAG, "PITCH: %f deg | ROLL: %f deg", pitch * (180.0f / M_PI), roll * (180.0f / M_PI));
                ESP_LOGI(COMMAND_TAG,"PITCH: %s | ROLL: %s | SPEED: %s\n", pitch_dir, roll_dir, speed);
            }
        }
        else {
            ESP_LOGE(IMU_TAG, "Read failed: %d", rslt);
        }

        start_dt = end_dt;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/*
TODO for demo:

    1. Angle threshold logic in main.c using roll and pitch from kalman_get_attitude
    2. A mapping function that converts angles to movement commands
    3. Terminal output showing raw vs filtered values side by side
    4. Optionally the stuck simulation using a simple timer or keypress to trigger fake encoder activity

*/

                // TODO: need to figure out if what I am doing makes sense, why would I pass in these values and not get anything in return to compare against
                // I might need to pass in empty variables and have the kalman filter right to them? Not sure

/*
TODO for overall project:

    1. Read encoder values — wheel velocity or tick count from your robot's encoders, this is your "commanded/expected motion" source
    2. Calculate IMU motion magnitude — combining ax, ay, az into a single value representing how much total movement the IMU detects
    3. Comparison logic — a threshold check between what encoders expect and what IMU reports
    4. Fault flag — a boolean or state that gets set when mismatch exceeds your threshold, triggering the recovery behavior

*/

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



