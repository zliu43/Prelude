#include "freertos/semphr.h"
#include "freertos/FreeRTOS.h"
#include "esp_mac.h"
#include "freertos/task.h"
#include "i2c_task_manager.h"
#include "esp_log.h"
#include <string.h>
#include <esp_timer.h>

#define RV3028_I2C_ADDR       0x52  // 7-bit address
#define RV3028_REG_EEADDR     0x25
#define RV3028_REG_EEDATA     0x26
#define RV3028_REG_EECMD      0x27
#define RV3028_REG_STATUS     0x0E
#define RV3028_CMD_EEPROM_WRITE 0x21

/**
 * @brief Structure representing the RTC time and date.
 */
typedef struct {
    uint16_t milliseconds;
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t weekday;
    uint8_t days;
    uint8_t months;
    uint16_t years; 
} RTC_time_t;

/**
 * @brief Global RTC time structure instance.
 */
RTC_time_t RTC_time;

/**
 * @brief Flag to signal RTC time synchronization.
 */
bool sync_time_flag = false;

/**
 * @brief Semaphore to indicate completion of I2C operation.
 */
SemaphoreHandle_t i2c_op_done;

/**
 * @brief Writes bytes to the specified I2C register.
 * 
 * @param reg  The register address to write to.
 * @param data Pointer to the data buffer.
 * @param len  Number of bytes to write.
 * 
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t i2c_write_bytes(uint8_t reg, uint8_t *data, uint8_t len) {
    uint8_t buffer[1 + len];
    buffer[0] = reg;
    memcpy(&buffer[1], data, len);

    i2c_message_t msg = {
        .payload = buffer,
        .done = i2c_op_done,
        .length = len + 1,
        .address = RV3028_I2C_ADDR,
        .write = true
    };

    if (xQueueSend(i2c_queue, &msg, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    xSemaphoreTake(i2c_op_done, portMAX_DELAY);
    return msg.result;
}

/**
 * @brief Reads bytes from the specified I2C register.
 * 
 * @param reg  The register address to read from.
 * @param dest Pointer to destination buffer.
 * @param len  Number of bytes to read.
 * 
 * @return ESP_OK on success, or an error code from the I2C read operation.
 */
esp_err_t i2c_read_bytes(uint8_t reg, uint8_t *dest, uint8_t len) { 
    i2c_message_t reg_msg = {
        .payload = &reg,
        .done = i2c_op_done,
        .length = 1,
        .address = RV3028_I2C_ADDR,
        .write = true
    };

    if (xQueueSend(i2c_queue, &reg_msg, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    xSemaphoreTake(i2c_op_done, portMAX_DELAY);
    if (reg_msg.result != ESP_OK) return reg_msg.result;
 
    i2c_message_t msg = {
        .payload = dest,
        .done = i2c_op_done,
        .length = len,
        .address = RV3028_I2C_ADDR,
        .write = false
    };

    if (xQueueSend(i2c_queue, &msg, portMAX_DELAY) != pdTRUE) return ESP_FAIL;
    xSemaphoreTake(i2c_op_done, portMAX_DELAY);
    return msg.result;
}

/**
 * @brief Writes data to RV3028 RAM.
 * 
 * @param reg    RAM register address.
 * @param data   Data to write.
 * @param length Number of bytes to write.
 * 
 * @return ESP_OK on success, or error code.
 */
esp_err_t rv3028_write_ram(uint8_t reg, uint8_t data, uint8_t length) {
    return i2c_write_bytes(reg, &data, length);
}

/**
 * @brief Reads data from RV3028 RAM.
 * 
 * @param reg    RAM register address.
 * @param data   Pointer to buffer to store data.
 * @param length Number of bytes to read.
 * 
 * @return ESP_OK on success, or error code.
 */
esp_err_t rv3028_read_ram(uint8_t reg, uint8_t *data, uint8_t length) {
    return i2c_read_bytes(reg, data, length);
}

/**
 * @brief Writes a byte to the RV3028 EEPROM.
 * 
 * @param ee_addr EEPROM address to write to.
 * @param data    Byte to write.
 * 
 * @return ESP_OK on success, or error code.
 */
esp_err_t rv3028_write_eeprom(uint8_t ee_addr, uint8_t data) {
    esp_err_t err;

    err = rv3028_write_ram(RV3028_REG_EEADDR, ee_addr, 1);
    if (err != ESP_OK) return err;

    err = rv3028_write_ram(RV3028_REG_EEDATA, data, 1);
    if (err != ESP_OK) return err;

    uint8_t cmd = RV3028_CMD_EEPROM_WRITE;
    err = rv3028_write_ram(RV3028_REG_EECMD, cmd, 1);
    if (err != ESP_OK) return err;

    uint8_t status = 0x80;
    do {
        err = rv3028_read_ram(RV3028_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        vTaskDelay(pdMS_TO_TICKS(2));
    } while (status & 0x80);

    return ESP_OK;
}

/**
 * @brief Timer callback function to trigger RTC synchronization.
 * 
 * @param arg Unused callback argument.
 */
void IRAM_ATTR clock_timer_callback(void* arg) {
    sync_time_flag = true;
}

/**
 * @brief Initializes the RTC and related synchronization primitives.
 */
void RTC_init(){
    RTC_time = (RTC_time_t){0};
    if (!i2c_op_done) {
        i2c_op_done = xSemaphoreCreateBinary();
        assert(i2c_op_done != NULL);
    }
}

/**
 * @brief Reads the current time from the RV3028 into the global RTC_time structure.
 */
void RTC_read_time() {
    uint8_t raw[7];
    if (rv3028_read_ram(0x00, raw, 7) != ESP_OK) {
        ESP_LOGE("RTC", "Failed to read time registers");
        return;
    }

    RTC_time.seconds = ((raw[0] >> 4) * 10) + (raw[0] & 0x0F);
    RTC_time.minutes = ((raw[1] >> 4) * 10) + (raw[1] & 0x0F);
    RTC_time.hours   = ((raw[2] >> 4) * 10) + (raw[2] & 0x0F);
    RTC_time.weekday = raw[3] & 0x07; 
    RTC_time.days    = ((raw[4] >> 4) * 10) + (raw[4] & 0x0F);
    RTC_time.months  = ((raw[5] >> 4) * 10) + (raw[5] & 0x0F);
    RTC_time.years   = 2000 + ((raw[6] >> 4) * 10) + (raw[6] & 0x0F);
}

/**
 * @brief Sets the RTC time by writing to the RV3028 time registers.
 * 
 * @param t Pointer to the RTC_time_t structure containing the new time.
 */
void RTC_set_time(const RTC_time_t *t) {
    uint8_t raw[7];
    raw[0] = ((t->seconds / 10) << 4) | (t->seconds % 10);
    raw[1] = ((t->minutes / 10) << 4) | (t->minutes % 10);
    raw[2] = ((t->hours / 10) << 4) | (t->hours % 10);
    raw[3] = t->weekday & 0x07;
    raw[4] = ((t->days / 10) << 4) | (t->days % 10);
    raw[5] = ((t->months / 10) << 4) | (t->months % 10);
    raw[6] = ((t->years % 100) / 10 << 4) | (t->years % 10);

    for (int i = 0; i < 7; ++i) {
        rv3028_write_ram(0x00 + i, raw[i], 1);
    }
}

/**
 * @brief Initializes a periodic timer to synchronize time every 10 ms.
 */
void init_clock_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &clock_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK, 
        .name = "clock_timer"
    };

    esp_timer_handle_t clock_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &clock_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(clock_timer, 10000)); 
}

/**
 * @brief RTOS task to periodically synchronize and track the RTC time in software.
 * 
 * @param pvParameters Unused parameter.
 */
void RTC_task(void* pvParameters){
    init_clock_timer();
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(25);
    TickType_t prev_tick = 0;  
    while(1){
        if (sync_time_flag){
            RTC_read_time();
            ESP_LOGI("RTC", "Time: %02u:%02u:%02u Date: %04u-%02u-%02u (Weekday: %u)",
                RTC_time.hours,
                RTC_time.minutes,
                RTC_time.seconds,
                RTC_time.years,
                RTC_time.months,
                RTC_time.days,
                RTC_time.weekday);
            sync_time_flag = false;
        }

        vTaskDelayUntil(&last_wake_time, interval);

        TickType_t now_tick = xTaskGetTickCount();

        if (prev_tick == 0) {
            prev_tick = now_tick;
            continue;
        }

        TickType_t elapsed_ticks = now_tick - prev_tick;
        prev_tick = now_tick;

        uint32_t elapsed_ms = elapsed_ticks * portTICK_PERIOD_MS;

        RTC_time.milliseconds += elapsed_ms;

        if (RTC_time.milliseconds >= 1000) {
            RTC_time.milliseconds -= 1000;
            RTC_time.seconds++;

            if (RTC_time.seconds >= 60) {
                RTC_time.seconds = 0;
                RTC_time.minutes++;

                if (RTC_time.minutes >= 60) {
                    RTC_time.minutes = 0;
                    RTC_time.hours++;

                    if (RTC_time.hours >= 24) {
                        RTC_time.hours = 0;
                        RTC_time.weekday = (RTC_time.weekday + 1) % 7;
                        RTC_time.months++;
                        if (RTC_time.months > 11) {
                            RTC_time.months = 0; 
                            RTC_time.years++;
                        }
                    }
                }
            }
        }
       
    }
}
