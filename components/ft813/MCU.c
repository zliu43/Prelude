#include "hardware_config.h"
#include "spi_task_manager.h"
#include "driver/gpio.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "MCU.h"
#include "esp_log.h"
#include <string.h>

static spi_device_handle_t FT813Q_handle;
static QueueHandle_t spi_queue;
static SemaphoreHandle_t FT813Q_done;

void MCU_Init(void){
    FT813Q_handle = get_ft813q_handle();
    spi_queue = get_spi_queue();
    FT813Q_done = xSemaphoreCreateBinary();
    if (FT813Q_handle != NULL && spi_queue != NULL){
        return;
    }
    ESP_LOGI("FT813", "FAILED TO INITIALIZE FT813_SPI");
}

void MCU_Setup(void){
    
}   

void MCU_CSlow(void){
    gpio_set_level((gpio_num_t)FT813Q_CS_PIN,0);
}

void MCU_CShigh(void){
    gpio_set_level((gpio_num_t)FT813Q_CS_PIN,1);

}

void MCU_PDlow(void){
    gpio_set_level((gpio_num_t)FT813Q_EN_PIN,0);
}

void MCU_PDhigh(void){
    gpio_set_level((gpio_num_t)FT813Q_EN_PIN,1);

}

void MCU_SPIWrite(const uint8_t *DataToWrite, uint32_t length){
    ESP_LOGI("MCU.h", "SPI Write (%d bytes):", (int)length);
    for (uint32_t i = 0; i < length; i++) {
        ESP_LOGI("MCU.h", "0x%02X", (int)DataToWrite[i]);
    }
    spi_task_t *task = malloc(sizeof(spi_task_t));
    uint8_t *tx_buf = malloc(length);

    if (!task || !tx_buf) {
        free(task); 
        free(tx_buf);
        return;
    }

    memcpy(tx_buf, DataToWrite, length);

    task->device = FT813Q_handle;
    task->tx_buffer = tx_buf;
    task->rx_buffer = NULL;
    task->length = length << 3;
    task->done = FT813Q_done;
    task->result = ESP_OK;

    xSemaphoreTake(task->done, 0);
    xQueueSend(spi_queue, &task, portMAX_DELAY);
    xSemaphoreTake(task->done, portMAX_DELAY);

    free(task->tx_buffer);
    free(task);
}

void MCU_SPIWrite8(uint8_t DataToWrite){
    MCU_SPIWrite(&DataToWrite, 1);
}

void MCU_SPIWrite16(uint16_t DataToWrite){
    uint16_t be_data = DataToWrite; //MCU_htobe16(DataToWrite);
    MCU_SPIWrite((uint8_t*)&be_data, 2);
}

void MCU_SPIWrite24(uint32_t DataToWrite){
    uint32_t be_data = DataToWrite; //MCU_htobe32(DataToWrite);
    uint8_t buf[3] = {
        (be_data >> 16) & 0xFF,
        (be_data >> 8) & 0xFF,
        be_data & 0xFF
    };
    MCU_SPIWrite(buf, 3);
}

void MCU_SPIWrite32(uint32_t DataToWrite){
    uint32_t be_data = DataToWrite; //MCU_htobe32(DataToWrite);
    MCU_SPIWrite((uint8_t*)&be_data, 4);
}

uint8_t MCU_SPIRead8(void) {
    spi_task_t *read_task = malloc(sizeof(spi_task_t));
    uint8_t *tx_buffer = calloc(1, 1);
    uint8_t *rx_buffer = calloc(1, 1);

    if (!read_task || !tx_buffer || !rx_buffer) {
        free(read_task);
        free(tx_buffer);
        free(rx_buffer);
        return 0;
    }

    *read_task = (spi_task_t){
        .device = FT813Q_handle,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = 8,
        .done = FT813Q_done,
        .result = ESP_OK
    };

    xSemaphoreTake(read_task->done, 0);
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);
    xSemaphoreTake(read_task->done, portMAX_DELAY);

    uint8_t result = rx_buffer[0];
    ESP_LOGI("MCU.c","RESULT: %d", (int)result);

    free(tx_buffer);
    free(rx_buffer);
    free(read_task);

    return result;
}


uint16_t MCU_SPIRead16(void) {
    spi_task_t *read_task = malloc(sizeof(spi_task_t));
    uint8_t *tx_buffer = calloc(2, 1);
    uint8_t *rx_buffer = calloc(2, 1);

    if (!read_task || !tx_buffer || !rx_buffer) {
        free(read_task);
        free(tx_buffer);
        free(rx_buffer);
        return 0;
    }

    *read_task = (spi_task_t){
        .device = FT813Q_handle,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = 16,
        .done = FT813Q_done,
        .result = ESP_OK
    };

    xSemaphoreTake(read_task->done, 0);
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);
    xSemaphoreTake(read_task->done, portMAX_DELAY);

    uint16_t result = ((uint16_t)rx_buffer[1] << 8) | rx_buffer[0];

    free(tx_buffer);
    free(rx_buffer);
    free(read_task);

    return result;
}


uint32_t MCU_SPIRead24(void) {
    spi_task_t *read_task = malloc(sizeof(spi_task_t));
    uint8_t *tx_buffer = calloc(3, 1);
    uint8_t *rx_buffer = calloc(3, 1);

    if (!read_task || !tx_buffer || !rx_buffer) {
        free(read_task);
        free(tx_buffer);
        free(rx_buffer);
        return 0;
    }

    *read_task = (spi_task_t){
        .device = FT813Q_handle,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = 24,
        .done = FT813Q_done,
        .result = ESP_OK
    };

    xSemaphoreTake(read_task->done, 0);
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);
    xSemaphoreTake(read_task->done, portMAX_DELAY);

    uint32_t result = ((uint32_t)rx_buffer[2] << 16) |
                      ((uint32_t)rx_buffer[1] << 8) |
                      rx_buffer[0];

    free(tx_buffer);
    free(rx_buffer);
    free(read_task);

    return result;
}


uint32_t MCU_SPIRead32(void){
    uint8_t *tx_buffer = calloc(4, 1);  
    uint8_t *rx_buffer = calloc(4, 1);
    spi_task_t *read_task = malloc(sizeof(spi_task_t));

    if (!tx_buffer || !rx_buffer || !read_task) {
        free(tx_buffer);
        free(rx_buffer);
        free(read_task);
        return ESP_ERR_NO_MEM; 
    }

    *read_task = (spi_task_t){
        .device = FT813Q_handle,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
        .length = 32,
        .done = FT813Q_done,
        .result = ESP_OK
    };

    xSemaphoreTake(read_task->done, 0);
    xQueueSend(spi_queue, &read_task, portMAX_DELAY);
    xSemaphoreTake(read_task->done, portMAX_DELAY);

    uint32_t result = ((uint32_t)rx_buffer[3] << 24 |
                       (uint32_t)rx_buffer[2] << 16 |
                       (uint32_t)rx_buffer[1] << 8  |
                       rx_buffer[0]);

    free(tx_buffer);
    free(rx_buffer);
    free(read_task);
    return result;
}

void MCU_Delay_20ms(void){
    vTaskDelay(pdMS_TO_TICKS(20));
}

void MCU_Delay_500ms(void){
    vTaskDelay(pdMS_TO_TICKS(500));
}


#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    #define HOST_IS_LITTLE_ENDIAN 1
#else
    #define HOST_IS_LITTLE_ENDIAN 0
#endif
 
uint16_t MCU_htobe16(uint16_t h) {
    return HOST_IS_LITTLE_ENDIAN ? __builtin_bswap16(h) : h;
}

uint16_t MCU_htole16(uint16_t h) {
    return HOST_IS_LITTLE_ENDIAN ? h : __builtin_bswap16(h);
}

uint16_t MCU_be16toh(uint16_t h) {
    return HOST_IS_LITTLE_ENDIAN ? __builtin_bswap16(h) : h;
}

uint16_t MCU_le16toh(uint16_t h) {
    return HOST_IS_LITTLE_ENDIAN ? h : __builtin_bswap16(h);
}
 
uint32_t MCU_htobe32(uint32_t h) {
    return HOST_IS_LITTLE_ENDIAN ? __builtin_bswap32(h) : h;
}

uint32_t MCU_htole32(uint32_t h) {
    return HOST_IS_LITTLE_ENDIAN ? h : __builtin_bswap32(h);
}

uint32_t MCU_be32toh(uint32_t h) {
    return HOST_IS_LITTLE_ENDIAN ? __builtin_bswap32(h) : h;
}

uint32_t MCU_le32toh(uint32_t h) {
    return HOST_IS_LITTLE_ENDIAN ? h : __builtin_bswap32(h);
}