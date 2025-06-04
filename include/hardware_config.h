#ifndef HARDWARE_CONFIG_H
#define HARDWARE_CONFIG_H
/**
 * Pin Configurations:
 * 
 * */

//I2C config
#define I2C_PORT        I2C_NUM_0
#define I2C_SCL_PIN     6
#define I2C_SDA_PIN     7
#define I2C_FREQ_HZ     100000      //100kHz

//SPI Config
#define SPI_MOSI_PIN    41
#define SPI_MISO_PIN    40
#define SPI_SCLK_PIN    39
#define DIS_CS_PIN      38
#define FT813Q_CS_PIN   13
#define IMU_CS_PIN      5

//INT Config
#define IMU_INT_PIN     4
#define BAT_INT_PIN     10
#define FT813Q_INT_PIN  12
#define RTC_INT_PIN     15

//Other GPIOs
#define FT813Q_EN_PIN   11


#endif
