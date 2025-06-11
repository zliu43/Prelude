#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "hardware_config.h"

static const char *TAG = "GPIO_INIT";

void init_gpios(){ 
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)DIS_CS_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)FT813Q_CS_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)IMU_CS_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)FT813Q_EN_PIN, GPIO_MODE_OUTPUT));
 
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)IMU_INT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)BAT_INT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)FT813Q_INT_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_direction((gpio_num_t)RTC_INT_PIN, GPIO_MODE_INPUT));
}
