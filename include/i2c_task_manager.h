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

/**
 * @brief Structure representing a single I2C message transaction.
 */
typedef struct {
    uint8_t     *payload;  
    SemaphoreHandle_t done; 
    uint32_t    length;    
    esp_err_t   result;     
    uint16_t    address;    
    bool        write;      
} i2c_message_t;

/**
 * @brief Handle to the I2C message queue.
 * 
 * @note this should be static and part of a .c file to avoid multiple definitions.
 */
QueueHandle_t i2c_queue;

/**
 * @brief Initializes the I2C message queue.
 * 
 * This function creates the queue used to store I2C transactions.
 * If creation fails, the system will restart.
 */

void i2c_queue_manager_init() {
    i2c_queue = xQueueCreate(I2C_QUEUE_LENGTH, sizeof(i2c_message_t));
    if (i2c_queue == NULL) {
        ESP_LOGI(TAG_i2c_manager, "Failed to create I2C queue, restarting...");
        esp_restart();
    }
}

/**
 * @brief I2C manager task that processes queued I2C transactions.
 * 
 * @param arg Unused parameter.
 * 
 * This FreeRTOS task runs in an infinite loop and processes I2C read/write
 * requests received via the `i2c_queue`. After each transaction, it
 * optionally signals completion using a semaphore.
 */
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


/**
 * @brief Initializes the I2C peripheral and starts the I2C manager task.
 * 
 * This sets up the I2C bus as a master, installs the driver, initializes
 * the I2C transaction queue, and creates the I2C manager task.
 */
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