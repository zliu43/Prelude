#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"

#include "imu.h"
#include "EVE.h"
#include "MCU.h"

#include "hardware_config.h"
#include "esp32_init.h" 
#include "i2c_task_manager.h"
#include "spi_task_manager.h"
#include "driver/gpio.h"

#define LED1 3
#define LED2 46

volatile uint8_t IMU_INT_FLAG = 0;

void app_main(void)
{   
    init_gpios();
    i2c_init();
    spi_init();
    imu_init();
    TaskHandle_t imu_task_handle;
    xTaskCreate(imu_task, "IMU_TASK", 4096, NULL, 10, &imu_task_handle);
    xTaskCreate(imu_int_task, "imu_int_task", 4096, NULL, 10, NULL);
    //EVE_Init();
}


void imu_int_task(void *pvParameters){
    while(1){
        if (IMU_INT_FLAG){
            ESP_LOGI("MAIN", "IMU_INT_DETECTED");
            IMU_INT_FLAG = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void IRAM_ATTR imu_isr_handler(){
    IMU_INT_FLAG = 1;
}