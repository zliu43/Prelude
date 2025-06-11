#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include "spi_task_manager.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "dsps_mul.h"


#define DEG2RAD 0.01745329251f
#define G_CONST 9.81f
#define DT 0.0048f 
#define ACC_SENSITIVITY (LSM6DSO_ACC_SENSITIVITY_FS_2G / 1000.0f * G_CONST) // ≈ 0.000598 m/s²
#define GYRO_SENSITIVITY (LSM6DSO_GYRO_SENSITIVITY_FS_250DPS / 1000.0f)  
#define LED1 3

volatile uint8_t IMU_INT_FLAG = 0;

static char* TAG_IMU = "IMU";
static QueueHandle_t spi_queue;
static spi_device_handle_t IMU_handle;
static SemaphoreHandle_t IMU_done;
static LSM6DSO_IO_t io_ctx;
static LSM6DSO_Object_t obj_ctx;    

/*
 * Initializes the spi communication structure.
 * Retrieves the SPI queue and the IMU handle, creates a binary semaphore for synchronization.

 * @brief Sets up all necessary components for SPI-based communication with the IMU.
 * @return 0 on success, -1 on failure.
*/
int32_t imu_spi_platform_init(void)
{

    spi_queue = get_spi_queue();
    IMU_handle = get_imu_handle();
    IMU_done = xSemaphoreCreateBinary();
    if (spi_queue != NULL && IMU_handle != NULL){
        return 0;
    }
    ESP_LOGE(TAG_IMU, "FAILED TO INITIALIZE IMU_SPI");
    return -1;
}

/*
 * @brief Writes data to the LSM6DSO IMU via SPI.
 * 
 * This function constructs and sends an SPI write command to the IMU. It allocates memory 
 * for the transmit and receive buffers, constructs the command, and sends it using a 
 * task-based SPI queue. Synchronization is handled using a binary semaphore.
 * 
 * @param handle Unused in this implementation, but commonly used to pass device-specific context.
 * @param reg    Register address to write to.
 * @param data   Pointer to the data to write.
 * @param len    Length of the data to write.
 * 
 * @return 0 on success, -1 on failure.
 */
int32_t lsm6dso_platform_write(void *handle, uint16_t reg, uint8_t *data, uint16_t len) {

    uint8_t *tx_data = malloc(len + 1);
    if (!tx_data) {
        return -1;
    }
    tx_data[0] = reg & 0x7F;
    memcpy(&tx_data[1], data, len);

    uint8_t *rx_data = malloc(1);  
    if (!rx_data) { 
        free(tx_data); return -1; 
    }

    spi_task_t *task = malloc(sizeof(spi_task_t)); 
    if (!task) { 
        free(tx_data); free(rx_data); return -1; 
    }

    task->device = IMU_handle;
    task->tx_buffer = tx_data;
    task->rx_buffer = rx_data;
    task->length = (len + 1) * 8;
    task->done = IMU_done;
    task->result = ESP_OK;

    xSemaphoreTake(task->done, 0);
    xQueueSend(spi_queue, &task, portMAX_DELAY);
    xSemaphoreTake(task->done, portMAX_DELAY);

    int res = (task->result == ESP_OK) ? 0 : -1;

    free((void *)task->tx_buffer);
    free((void *)task->rx_buffer);
    free(task);
    return res;
}

/*
 * @brief Reads data from the LSM6DSO IMU via SPI.
 * 
 * This function constructs and sends an SPI read command to the IMU. It allocates memory 
 * for the transmit and receive buffers, constructs the command, and sends it using a 
 * task-based SPI queue. Synchronization is handled using a binary semaphore.
 * 
 * @param handle Unused in this implementation, but commonly used to pass device-specific context.
 * @param reg    Register address to write to.
 * @param data   Pointer to the data to write.
 * @param len    Length of the data to write.
 * 
 * @return 0 on success, -1 on failure.
 */
int32_t lsm6dso_platform_read(void *handle, uint16_t reg, uint8_t *data, uint16_t len) {
    spi_device_handle_t dev = get_imu_handle();

    uint8_t *tx_data = malloc(len + 1);
    uint8_t *rx_data = malloc(len + 1);
    if (!tx_data || !rx_data) {
        free(tx_data);
        free(rx_data);
        return -1;
    }

    tx_data[0] = reg | 0x80;  

    spi_task_t *read_task = malloc(sizeof(spi_task_t));
    if (!read_task) {
        free(tx_data);
        free(rx_data);
        return -1;
    }

    read_task->device = dev;
    read_task->tx_buffer = tx_data;
    read_task->rx_buffer = rx_data;
    read_task->length = (len + 1) * 8;
    read_task->done = IMU_done;
    read_task->result = ESP_OK;

    xSemaphoreTake(read_task->done, 0);  
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);  
    xSemaphoreTake(read_task->done, portMAX_DELAY);  

    int result = (read_task->result == ESP_OK) ? 0 : -1;
 
    if (result == 0) {
        memcpy(data, &rx_data[1], len);
    }
 
    free((void *)read_task->tx_buffer);
    free((void *)read_task->rx_buffer);
    free(read_task);

    return result;
}

/*
 * @brief Platform-specific delay function for the LSM6DSO driver.
 *
 * This function introduces a blocking delay for a specified number of milliseconds.
 * It uses the FreeRTOS `vTaskDelay` function, which suspends the current task
 * for the given duration, allowing other tasks to run.
 *
 * @param milisec_delay Delay duration in milliseconds.
 */
void lsm6dso_platform_delay(uint32_t milisec_delay){
    vTaskDelay(pdMS_TO_TICKS(milisec_delay));
}

/*
 * @brief Platform-specific function to get the current system tick count for the LSM6DSO driver.
 *
 * This function returns the current RTOS tick count, which can be used for timestamping or 
 * measuring time intervals. It relies on FreeRTOS's `xTaskGetTickCount()`, which provides 
 * the number of ticks since the scheduler started.
 *
 * @return Current tick count as a 32-bit integer.
 */
int32_t lsm6dso_platform_get_tick(void) {
    return xTaskGetTickCount();  
}

/*
@brief Interrupt Service Routine (ISR) handler for the IMU.
*/
void IRAM_ATTR imu_isr_handler(){
    IMU_INT_FLAG = 1;
}

/**
 * @brief Task that handles the IMU interrupt and toggles an LED.
 *
 * Initializes the GPIO3 pin as output, registers an ISR 
 * for the IMU interrupt pin, and continuously monitors the interrupt flag.
 * When an interrupt is detected (IMU_INT_FLAG is set), it logs the event,
 * toggles the LED state, and clears the interrupt flag.
 *
 * @param pvParameters Pointer to task parameters (unused).
 *
 * @note The task runs at 50Hz. 
 */
void imu_int_task(void *pvParameters){
    bool led_1_on = false;
    gpio_set_direction((gpio_num_t)LED1, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(gpio_isr_handler_add(IMU_INT_PIN, imu_isr_handler, (void*)IMU_INT_PIN));
    while(1){
        if (IMU_INT_FLAG){
            ESP_LOGI("MAIN", "IMU_INT_DETECTED");
            led_1_on = !led_1_on;
            gpio_set_level(LED1,led_1_on);
            IMU_INT_FLAG = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/**
 * @brief Initializes the IMU (LSM6DSO) and configures its settings.
 *
 * This function performs the following steps to set up the IMU:
 * - Initializes the SPI platform for communication with the IMU.
 * - Prepares and assigns the device and IO context structures.
 * - Registers the bus IO functions with the IMU driver.
 * - Initializes the LSM6DSO sensor.
 * - Enables the accelerometer and gyroscope.
 * - Configures the accelerometer to output data at 104 Hz in high-performance mode.
 * - Enables single-tap detection on INT1 pin.
 * - Configures the IMU interrupt GPIO pin as an input with rising edge interrupt and internal pull-up.
 *
 * Logging messages are generated if any of the initialization steps fail.
 */
void imu_init(){
    imu_spi_platform_init();
    stmdev_ctx_t dev_ctx = {
        .write_reg = lsm6dso_platform_write,
        .read_reg  = lsm6dso_platform_read,
        .mdelay    = lsm6dso_platform_delay,
        .handle    = get_imu_handle()
    };

    io_ctx = (LSM6DSO_IO_t){
        .Init     = imu_spi_platform_init,
        .DeInit   = NULL, 
        .BusType  = 1, 
        .Address  = NULL,    
        .WriteReg = lsm6dso_platform_write,
        .ReadReg  = lsm6dso_platform_read,
        .GetTick  = lsm6dso_platform_get_tick, 
        .Delay    = lsm6dso_platform_delay
    }; 
    obj_ctx = (LSM6DSO_Object_t){
        .IO             = io_ctx,
        .Ctx            = dev_ctx,
    };
    if (LSM6DSO_RegisterBusIO(&obj_ctx, &io_ctx) != LSM6DSO_OK){
        ESP_LOGI(TAG_IMU, "Failed to register BUS IO");
        return;
    }
    if (LSM6DSO_Init(&obj_ctx) != LSM6DSO_OK){
        ESP_LOGI(TAG_IMU, "Failed to Init LSM6DSO");
        return;
    }
    if (LSM6DSO_ACC_Enable(&obj_ctx) != LSM6DSO_OK){
        ESP_LOGI(TAG_IMU, "Failed to enable Accelerometer");
        return;
    }
    if (LSM6DSO_GYRO_Enable(&obj_ctx) != LSM6DSO_OK){
        ESP_LOGI(TAG_IMU, "Failed to enable Gyroscope");
        return;
    }

    LSM6DSO_ACC_SetOutputDataRate_With_Mode(&obj_ctx, 104.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
    LSM6DSO_ACC_Enable_Single_Tap_Detection(&obj_ctx, LSM6DSO_INT1_PIN); 

    gpio_config_t imu_int_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = 1ULL << IMU_INT_PIN,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&imu_int_conf));
}

/*
 * @brief Task that continuously reads accelerometer data from the IMU.
 * 
 * This task runs in an infinite loop, reading accelerometer axes data every 500 milliseconds.
 * If reading the data fails, it logs an error message.
 * 
 * @param pvParameters Pointer to task parameters (unused).
 * 
 * @note UNUSED FUNCTION, USED FOR TESTING PURPOSES ONLY.
 */
void imu_task(void *pvParameters){
    while(1){
        LSM6DSO_Axes_t accel_data;
        if(LSM6DSO_ACC_GetAxes(&obj_ctx, &accel_data) != LSM6DSO_OK){
            ESP_LOGI("ACC", "FAILED TO READ ACCEL DATA");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int16_t accel_x_data[1024] __attribute__((aligned(16)));
int16_t accel_y_data[1024] __attribute__((aligned(16)));
int16_t accel_z_data[1024] __attribute__((aligned(16)));

int16_t gyro_x_data[1024] __attribute__((aligned(16)));
int16_t gyro_y_data[1024] __attribute__((aligned(16)));
int16_t gyro_z_data[1024] __attribute__((aligned(16)));

unsigned int sample_idx;
SampleBuffer pending_sample = {0};

/**
 * @brief Structure representing a single IMU data sample.
 *
 * Stores raw accelerometer and gyroscope readings and flags indicating
 * whether the data is ready to be processed.
 */
typedef struct {
    int16_t ax, ay, az;
    bool accel_ready;
    int16_t gx, gy, gz;
    bool gyro_ready;
} SampleBuffer;

/**
 * @brief Computes the mean of the first 256 elements in an array.
 *
 * @param data Pointer to the input data array.
 * @return int16_t The average value of the first 256 elements.
 */
int16_t feature_mean(const int16_t* data){
    int32_t sum = 0;
    for (int i = 0; i < 256; ++i){
        sum += data[i];
    }
    return (int16_t)(sum >> 8);
}

/*
int32_t feature_power(const int16_t* data){
    a5 - counter
    a2 - data pointer
    
    __asm__ __volatile__{
        movi.n      a5, 32       
        movi.n      a2, %1    
        ee.zero.accx               
        ee.vld.128.ip     a2, a5, 0
            ee.vld.128.ip     q0, a2, 16        
            ee.vmulas.s16.accx q0, q0         
        end_loop:

        e.st.accx.ip   a3, 8                 
    }
}*/


int counter = 0;

/**
 * @brief Computes the mean of the first 256 elements in an array.
 *
 * @param data Pointer to the input data array.
 * @return int16_t The average value of the first 256 elements.
 */
void calculate_features(const int index){
    int16_t x_acc_mean = feature_mean(&accel_x_data[index]) >> 4;
    int16_t y_acc_mean = feature_mean(&accel_y_data[index]) >> 4;
    int16_t z_acc_mean = feature_mean(&accel_z_data[index]) >> 4;
    int16_t x_gyro_mean = (feature_mean(&gyro_x_data[index]) * 9);
    int16_t y_gyro_mean = (feature_mean(&gyro_y_data[index]) * 9);
    int16_t z_gyro_mean = (feature_mean(&gyro_z_data[index]) * 9);

    if (counter > 10){
        counter = 0;
        ESP_LOGI("IMU",
                "Accel mean: x=%d , y=%d , z=%d | Gyro mean: x=%d , y=%d , z=%d",
                x_acc_mean, y_acc_mean, z_acc_mean, 
                x_gyro_mean, y_gyro_mean, z_gyro_mean);
    }
    
    counter++;
}

/**
 * @brief FreeRTOS task that triggers feature extraction from a data segment.
 *
 * Takes an index (start position in the data buffer), calculates features, 
 * and then self-deletes.
 *
 * @param pvParameters Pointer to the index as a dynamically allocated int.
 */
void feature_task(void* pvParameters) {
    int index = *(int*)pvParameters;
    calculate_features(index);
    free(pvParameters);
    vTaskDelete(NULL); 
}

/**
 * @brief Configures the IMU for polling mode with 208 Hz data rate.
 *
 * Sets up both the accelerometer and gyroscope for high-performance,
 * 208 Hz data output.
 */
void imu_init_polling_mode() {
    LSM6DSO_ACC_SetOutputDataRate_With_Mode(&obj_ctx, 208.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
    LSM6DSO_GYRO_SetOutputDataRate(&obj_ctx, 208.0f);
}

/**
 * @brief Reads IMU data using polling and stores it in aligned buffers.
 *
 * Reads raw accelerometer and gyroscope data into a temporary sample buffer.
 * Once both are available, stores them into circular data buffers and 
 * triggers a feature extraction task every 256 samples.
 */
void imu_poll_read() {
    LSM6DSO_AxesRaw_t raw_accel;
    LSM6DSO_AxesRaw_t raw_gyro;

    if (LSM6DSO_ACC_GetAxesRaw(&obj_ctx, &raw_accel) == LSM6DSO_OK) {
        pending_sample.ax = raw_accel.x;
        pending_sample.ay = raw_accel.y;
        pending_sample.az = raw_accel.z;
        pending_sample.accel_ready = true;
    }

    if (LSM6DSO_GYRO_GetAxesRaw(&obj_ctx, &raw_gyro) == LSM6DSO_OK) {
        pending_sample.gx = raw_gyro.x;
        pending_sample.gy = raw_gyro.y;
        pending_sample.gz = raw_gyro.z;
        pending_sample.gyro_ready = true;
    }

    if (pending_sample.accel_ready && pending_sample.gyro_ready) {

        accel_x_data[sample_idx] = pending_sample.ax;
        accel_y_data[sample_idx] = pending_sample.ay;
        accel_z_data[sample_idx] = pending_sample.az;

        gyro_x_data[sample_idx] = pending_sample.gx;
        gyro_y_data[sample_idx] = pending_sample.gy;
        gyro_z_data[sample_idx] = pending_sample.gz;

        sample_idx++;

        if (sample_idx % 256 == 0) {
            int* index_ptr = malloc(sizeof(int));
            switch (sample_idx) {
            case 256:
                *index_ptr = 0;
                break;
            case 512:
                *index_ptr = 256;
                break;
            case 768:
                *index_ptr = 512;
                break;
            case 1024:
                *index_ptr = 768;
                break;
            default:
                break;
            }
            xTaskCreatePinnedToCore(feature_task, "FeatureTask", 16384, index_ptr, 5, NULL, 1);
            sample_idx = sample_idx % 1024;
        }

        pending_sample.accel_ready = false;
        pending_sample.gyro_ready = false;
    }
}

/**
 * @brief FreeRTOS task that continuously polls IMU data.
 *
 * Initializes the IMU in polling mode and repeatedly calls `imu_poll_read()`
 * with a 5 ms delay between reads.
 *
 * @param pvParameters Unused.
 */
void tracking_task(void* pvParameters) {
    imu_init_polling_mode();
    while (1) {
        imu_poll_read();
        vTaskDelay(pdMS_TO_TICKS(5));  
    }
}
