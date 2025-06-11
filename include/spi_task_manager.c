#include "spi_task_manager.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <assert.h>
 
static const char *TAG_SPI = "SPI_MANAGER";

static QueueHandle_t spi_queue;
static spi_device_handle_t FT813Q_handle;
static spi_device_handle_t IMU_handle;
static spi_device_handle_t DIS_handle;
 
/**
 * @brief Returns the handle to the SPI task queue.
 * 
 * @return QueueHandle_t Handle to the SPI queue.
 */
QueueHandle_t get_spi_queue(void) {
    return spi_queue;
}

/**
 * @brief Returns the SPI device handle for the FT813Q.
 * 
 * @return spi_device_handle_t SPI handle for FT813Q.
 */
spi_device_handle_t get_ft813q_handle(void) {
    return FT813Q_handle;
}

/**
 * @brief Returns the SPI device handle for the IMU.
 * 
 * @return spi_device_handle_t SPI handle for IMU.
 */
spi_device_handle_t get_imu_handle(void) {
    return IMU_handle;
}

/**
 * @brief Returns the SPI device handle for the DIS.
 * 
 * @return spi_device_handle_t SPI handle for DIS.
 */
spi_device_handle_t get_dis_handle(void) {
    return DIS_handle;
}

/**
 * @brief SPI manager task that processes messages from the SPI queue.
 * 
 * @param arg Unused parameter.
 * 
 * This task receives `spi_task_t*` messages from the SPI queue, executes the SPI transaction,
 * and signals the result using a semaphore if provided.
 */
static void spi_manager_task(void *arg) {
    spi_task_t *msg;

    while (1) {
        if (xQueueReceive(spi_queue, &msg, portMAX_DELAY)) {
            /*
            ESP_LOGI(TAG_SPI,
                     "Received SPI Task:\n"
                     "  device: 0x%p\n"
                     "  length: %d bits\n",
                     (void *)msg->device,
                     msg->length);
            ESP_LOGI(TAG_SPI, "device %p\n", msg->device);*/

            spi_transaction_t spi_trans = {
                .tx_buffer = msg->tx_buffer,
                .rx_buffer = msg->rx_buffer,
                .length = msg->length,
            };

            msg->result = spi_device_transmit(msg->device, &spi_trans);

            if (msg->done) {
                xSemaphoreGive(msg->done);
            }
        }
    }
}

/**
 * @brief Initializes the SPI queue used to communicate SPI tasks.
 * 
 * @note This must be called before any SPI task management occurs.
 */
void spi_queue_manager_init(void) {
    spi_queue = xQueueCreate(SPI_QUEUE_LENGTH, sizeof(spi_task_t *));
    assert(spi_queue != NULL);
}
/** 
 * @brief Initializes the SPI bus and devices, and starts the SPI manager task.
 * 
 * This function sets up the SPI bus with the given pin configuration and adds
 * devices (FT813Q, IMU, and DIS) to the SPI bus. It also creates the SPI
 * transaction queue and starts the task that processes SPI transactions.
 */
void spi_init(void) {
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };

    spi_device_interface_config_t FT813Qcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = FT813Q_CS_PIN,
        .queue_size = 1,
    };

    spi_device_interface_config_t IMUcfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = IMU_CS_PIN,
        .queue_size = 1,
    };

    spi_device_interface_config_t DIScfg = {
        .clock_speed_hz = 1 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = DIS_CS_PIN,
        .queue_size = 1,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &FT813Qcfg, &FT813Q_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &IMUcfg, &IMU_handle));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &DIScfg, &DIS_handle));

    spi_queue_manager_init();
    xTaskCreate(spi_manager_task, "spi_mgr", 4096, NULL, 10, NULL);
}
