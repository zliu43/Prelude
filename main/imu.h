#include "lsm6dso.h"
#include "lsm6dso_reg.h"
#include "spi_task_manager.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static char* TAG_IMU = "IMU";
static QueueHandle_t spi_queue;
static spi_device_handle_t IMU_handle;
static SemaphoreHandle_t IMU_done;
static LSM6DSO_IO_t io_ctx;
static LSM6DSO_Object_t obj_ctx;    

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

int32_t lsm6dso_platform_write(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {

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

int32_t lsm6dso_platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    spi_device_handle_t dev = get_imu_handle();

    // Allocate TX and RX buffers on the heap
    uint8_t *tx_data = malloc(len + 1);
    uint8_t *rx_data = malloc(len + 1);
    if (!tx_data || !rx_data) {
        free(tx_data);
        free(rx_data);
        return -1;
    }

    tx_data[0] = reg | 0x80;  // Set read bit

    // Allocate the task itself on the heap
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

    xSemaphoreTake(read_task->done, 0);  // reset semaphore
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);  // send pointer
    xSemaphoreTake(read_task->done, portMAX_DELAY);  // wait for it to finish

    int result = (read_task->result == ESP_OK) ? 0 : -1;
 
    if (result == 0) {
        memcpy(data, &rx_data[1], len);
    }
 
    free((void *)read_task->tx_buffer);
    free((void *)read_task->rx_buffer);
    free(read_task);

    return result;
}

void lsm6dso_platform_delay(uint32_t milisec_delay){
    vTaskDelay(pdMS_TO_TICKS(milisec_delay));
}

int32_t lsm6dso_platform_get_tick(void) {
    return xTaskGetTickCount();  
}

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
    LSM6DSO_ACC_SetOutputDataRate_With_Mode(&obj_ctx, 104.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
    LSM6DSO_ACC_Enable_Single_Tap_Detection(&obj_ctx, LSM6DSO_INT1_PIN);

}

void imu_task(void *pvParameters){
    while(1){
        LSM6DSO_Axes_t accel_data;
        if(LSM6DSO_ACC_GetAxes(&obj_ctx, &accel_data) != LSM6DSO_OK){
            ESP_LOGI("ACC", "FAILED TO READ ACCEL DATA");
        }
        ESP_LOGI("ACC", "X: %d\tY: %d\tZ: %d", (int)accel_data.x, (int)accel_data.y, (int)accel_data.z);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
