#ifndef SPI_TASK_MANAGER_H
#define SPI_TASK_MANAGER_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include <stdint.h>
#include <stdbool.h>
#include "hardware_config.h"

// Constants
#define SPI_QUEUE_LENGTH 16

// SPI Task Structure
typedef struct {
    spi_device_handle_t device;
    const uint8_t *tx_buffer;
    uint8_t *rx_buffer;
    uint16_t length;
    SemaphoreHandle_t done;
    esp_err_t result;
} spi_task_t;

// Function declarations
QueueHandle_t get_spi_queue(void);
spi_device_handle_t get_ft813q_handle(void);
spi_device_handle_t get_imu_handle(void);
spi_device_handle_t get_dis_handle(void);

void spi_queue_manager_init(void);
void spi_init(void);

#endif // SPI_TASK_MANAGER_H
