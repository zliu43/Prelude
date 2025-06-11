#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_flash.h"

#include "imu.h"
#include "EVE.h"
#include "RTC.h"
#include "MCU.h"

#include "hardware_config.h"
#include "esp32_init.h" 
#include "i2c_task_manager.h"
#include "spi_task_manager.h"
#include "driver/gpio.h"

#define LED1 3
#define LED2 46

/**
 * @brief Entry point of the application.
 *
 * Initializes all required peripherals (GPIOs, I2C, SPI, IMU, and RTC),
 * and creates various FreeRTOS tasks that manage sensor data acquisition
 * and processing.
 *
 * Tasks launched:
 * - `RTC_task`: Handles Real-Time Clock functionality.
 * - `tracking_task`: Polls IMU data at a fixed interval and processes it.
 * - `imu_int_task`: Handles IMU interrupts and toggles an LED indicator.
 *
 * @note Some optional tasks (e.g., `imu_task`, `EVE_Init`) are currently omitted.
 * 
 * @return void
 */
void app_main(void)
{   
    init_gpios();
    i2c_init();
    spi_init();
    imu_init();
    RTC_init();
    //TaskHandle_t imu_task_handle;
    //xTaskCreate(imu_task, "IMU_TASK", 4096, NULL, 10, &imu_task_handle);
    xTaskCreate(RTC_task, "rtc_task", 4096, NULL, 1, NULL);
    //EVE_Init();
    xTaskCreatePinnedToCore(tracking_task, "tracking_task",4096, NULL,2,NULL,1);
    xTaskCreate(imu_int_task, "imu_int_task", 4096, NULL, 10, NULL);
}
