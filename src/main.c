// built in headers
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>

// headers that I made
#include "bmi270_port.h"  
#include "bmi2.h"
#include "bmi270.h"
#include "kalman_wrapper.h"

// I2C Config for the ESP
#define GPIO_SDA 8
#define GPIO_SCL 9
#define BOOT_BUTTON GPIO_NUM_0
#define I2C_PORT I2C_NUM_0
#define BMI270_ADDR 0x68
#define I2C_FREQ_HZ 100000   

// defines for changing the frequency on a button press
static volatile int64_t last_pressed_time = 0;
#define DEBOUNCE_MS 100
static const int freq[] = {100,500};
static const int freq_count = 2;
static volatile int freq_index = 0;

// TAGs for print statements
static const char *IMU_TAG = "IMU";
static const char *FILTERED_TAG = "FILTERED";
static const char *COMMAND_TAG = "COMMAND";


// ------------------------ Kalman Movement Implementation ---------------------------
// (these are the upper bounds other than fast, will act as a lower bound instead) - vals are in degrees
typedef enum {
    STOPPED,
    SLOW,
    MED,
    FAST
} SPEED;

typedef enum {
    PITCH_DEADZONE,
    FORWARDS,
    BACKWARDS
} PITCH_DIRECTION;

typedef enum {
    ROLL_DEADZONE,
    LEFT,
    RIGHT
} ROLL_DIRECTION;

typedef struct {
    SPEED speed;
    PITCH_DIRECTION pitch_dir;
    ROLL_DIRECTION roll_dir;
    float rotational_movement;
} mapping_t;
// ----------------------------------------------------------------------------------------------------------

// code for the ISR (button press)
static void IRAM_ATTR button_isr_handler(void *arg) {
    int64_t now = esp_timer_get_time() / 1000; // convert µs to ms
    if ((now - last_pressed_time) > DEBOUNCE_MS) {
        freq_index = (freq_index + 1) % freq_count;
        last_pressed_time = now;
    }
}

// Functionality for getting the pitch and roll
static void get_direction_and_speed(float roll, float pitch, mapping_t *map){
    
    // converting to degrees
    float roll_deg = roll * (180 / M_PI);
    float pitch_deg = pitch * (180 / M_PI);

    if (pitch_deg < 0.0f){
        map->pitch_dir = FORWARDS;
    }
    else if (pitch_deg > 0.0f){
        map->pitch_dir = BACKWARDS;
    }
    else {
        map->pitch_dir = PITCH_DEADZONE;
    }

    if (roll_deg < 0.f){
        map->roll_dir = LEFT;
    }
    else if (roll_deg > 0.0f){
        map->roll_dir = RIGHT;
    }
    else {
        map->roll_dir = ROLL_DEADZONE;
    }
    

    // getting absolute value
    float abs_roll_deg = fabsf(roll_deg);
    float abs_pitch_deg = fabsf(pitch_deg);

    float rotational_movement = ((abs_pitch_deg > abs_roll_deg) ? abs_pitch_deg : abs_roll_deg);
    map->rotational_movement = rotational_movement;

    // checking for speed and setting type in mapping_t struct
    if (rotational_movement < 10.00){
        map->speed = STOPPED;
        // map->pitch_dir = PITCH_DEADZONE;
        // map->roll_dir = ROLL_DEADZONE;
        return;   
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

// for printing to the terminal
static void direction_and_speed_to_string(char **pitch_dir, char **roll_dir, char **speed, mapping_t map){

    // for writing to dir char**
    if (map.roll_dir == ROLL_DEADZONE){
        *roll_dir = "Deadzone";
    }
    else if (map.roll_dir == LEFT){
        *roll_dir = "Left";
    }
    else if (map.roll_dir == RIGHT){
        *roll_dir = "Right";
    }

    if (map.pitch_dir == PITCH_DEADZONE){
        *pitch_dir = "Deadzone";
    }
    else if (map.pitch_dir == FORWARDS){
        *pitch_dir = "Forwards";
    }
    else {
        *pitch_dir = "Backwards";
    }
    
    
    // for writing to speed char**
    if (map.speed == STOPPED){
        *speed = "Stopped";
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

// I think I will need this later
// // function for calculating distance moved based on ax, ay, az, and dt
// double magnitude_distance(float ax, float ay, float az, float dt){
//     double a = sqrt((double)((ax*ax) + (ay*ay) + (az*az)));
// }


// main function in ESP32
void app_main(void) {

    // Initialize I2C
    ESP_ERROR_CHECK(imu_i2c_init(GPIO_SDA, GPIO_SCL, I2C_FREQ_HZ));

    // Configure button GPIO
    gpio_config_t btn_cfg = {
        .pin_bit_mask = (1ULL << BOOT_BUTTON),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,   // Button pulls LOW when pressed
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,    // Trigger on falling edge (press)
    };
    gpio_config(&btn_cfg);

    // Install ISR service and attach handler
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BOOT_BUTTON, button_isr_handler, NULL);

    // Initializing the IMU
    bmi270_i2c_ctx_t ctx = {
        .port = I2C_PORT,
        .addr = BMI270_ADDR
    };
    struct bmi2_dev dev = {0};
    int8_t rslt = bmi270_esp_init(&ctx, &dev);
    ESP_LOGI(IMU_TAG, "BMI270 init result: %d", rslt);

    // variables that are going to be updated in the main loop
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, yaw;
    int64_t start_dt = esp_timer_get_time();
    int64_t end_dt;
    float delta_dt;
    bool init_flag_acc = true;

    while (1){

        // getting the changes in time
        end_dt = esp_timer_get_time();

        // reading the IMU into a struct
        struct bmi2_sens_axes_data acc, gyr;
        rslt = bmi270_esp_read_ag(&dev, &acc, &gyr);

        // converting acceleration to g's
        ax = (float) (acc.x / (32768.0f / 4.0f));
        ay = (float) (acc.y / (32768.0f / 4.0f));
        az = (float) (acc.z / (32768.0f / 4.0f));

        // converting gyr data to degrees per second (dps)
        gx = (float) (gyr.x / (32768.0f / 2000.0f));
        gy = (float) (gyr.y / (32768.0f / 2000.0f));
        gz = (float) (gyr.z / (32768.0f / 2000.0f));

        // g = m/s^2 and dps = degrees per second
        if (rslt == BMI2_OK) {

            // printing out raw values
            ESP_LOGI(IMU_TAG, "ACC: x=%.2fg y=%.2fg z=%.2fg | GYR: x=%.2fdps y=%.2fdps z=%.2fdps", ax, ay, az, gx, gy, gz);

            // if this is the first occurance, initialize the kalman filter
            if (init_flag_acc && fabsf(az) > 0.1f){
                kalman_init(ax, ay, -az);       // negating az (for some reason the filter reads upside down - can chnage later)
                init_flag_acc = false;
            }
            else if (!init_flag_acc){

                // getting the change in time and using the kalman filter wrapper functions to filter the IMU readings
                delta_dt = (float) (end_dt - start_dt) / 1000000.0f;
                kalman_update(ax, ay, -az, gx, gy, gz, delta_dt);       // negating az
                kalman_get_attitude(&roll, &pitch, &yaw);

                // getting the mapping of roll and pitch
                mapping_t map = {0};
                get_direction_and_speed(roll, pitch, &map);

                // converting the roll, pitch, and speed to something printable
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

        // changes in loop timing for kalman function
        start_dt = end_dt;
        
        // for setting the freq
        int curr_freq = freq[freq_index];
        vTaskDelay(pdMS_TO_TICKS(curr_freq));
    }
}
/*
TODO for demo:

    1. Angle threshold logic in main.c using roll and pitch from kalman_get_attitude
    2. A mapping function that converts angles to movement commands
    3. Terminal output showing raw vs filtered values side by side

*/

/*
TODO for overall project:

    1. Read encoder values — wheel velocity or tick count from your robot's encoders, this is your "commanded/expected motion" source
    2. Calculate IMU motion magnitude — combining ax, ay, az into a single value representing how much total movement the IMU detects
    3. Comparison logic — a threshold check between what encoders expect and what IMU reports
    4. Fault flag — a boolean or state that gets set when mismatch exceeds your threshold, triggering the recovery behavior

*/
