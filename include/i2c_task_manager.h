#ifndef I2C_TASK_MANAGER_H
#define I2C_TASK_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "driver/i2c.h"
#include <stdbool.h>
#include <stdint.h>
#include "esp_log.h"

#include "hardware_config.h"

static char* TAG_i2c_manager = "i2c_task_manager";

#define I2C_QUEUE_LENGTH 16

typedef struct {
    uint8_t     *payload;  
    SemaphoreHandle_t done; 
    uint32_t    length;    
    esp_err_t   result;     
    uint16_t    address;    
    //uint8_t     priority; 
    bool        write;      
} i2c_message_t;

QueueHandle_t i2c_queue;


void i2c_queue_manager_init() {
    i2c_queue = xQueueCreate(I2C_QUEUE_LENGTH, sizeof(i2c_message_t));
    if (i2c_queue == NULL) {
        ESP_LOGI(TAG_i2c_manager, "Failed to create I2C queue, restarting...");
        esp_restart();
    }
}

static void i2c_manager_task(void *arg) {
    i2c_message_t trans;
    while (1) {
        if (xQueueReceive(i2c_queue, &trans, portMAX_DELAY)) {
            if (trans.write) {
                trans.result = i2c_master_write_to_device(I2C_NUM_0, trans.address,
                    trans.payload, trans.length, pdMS_TO_TICKS(100));
            } else {
                trans.result = i2c_master_read_from_device(I2C_NUM_0, trans.address,
                    trans.payload, trans.length, pdMS_TO_TICKS(100));
            }

            if (trans.done) {
                xSemaphoreGive(trans.done);
            }
        }
    }
}

void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);

    i2c_queue_manager_init();
    xTaskCreate(i2c_manager_task, "i2c_mgr", 4096, NULL, 10, NULL);
}

#endif