#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_mac.h"
#include "driver/gpio.h"

#include "spi_task_manager.h"
#include "FT813_cmds.h"

static QueueHandle_t spi_queue;
static spi_device_handle_t FT813Q_handle;

/* Taken from FT813Q datasheet:
During boot up, the following steps are required:
1. Send host command “CLKEXT” if the PLL input is from external crystal oscillator or external
clock.
2. Send host command “CLKSEL” to select system clock frequency if the non-default system
clock is to be used. By default, the system clock is set to 60MHz.
3. Send host command “RST_PULSE” to reset the core of EVE.
4. Send host command “ACTIVE” to enable the clock. The self-diagnosis process might take a
maximum of 300 milliseconds to finish, so delay is recommended, e.g. 40 ~ 300 ms.
5. Read REG_ID until 0x7C is returned.
6. Read REG_CPURESET till EVE goes into the working status, i.e., zero is returned.
7. Configure display control timing registers, except REG_PCLK
8. Write first display list to RAM_DL.
9. Write REG_DLSWAP to start graphics engine rendering process with first display list
10. Enable backlight control for display panel
11. Write REG_PCLK to configure the PCLK frequency of display panel, which leads to the output
of the first display list */
void FT813Q_boot_seq(){
    spi_queue = get_spi_queue();
    
}

void init_from_PD(){
    /*Initialization Sequence from Power Down using PD_N pin:
    1. Drive the PD_N pin high
    2. Wait for at least 20ms
    3. Execute” Initialization Sequence during the Boot UP”*/
    gpio_set_level((gpio_num_t)FT813Q_EN_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(25));
    FT813Q_boot_seq();
}