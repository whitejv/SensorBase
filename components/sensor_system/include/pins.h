/*
 * pins.h - ESP32 GPIO Pin Definitions
 *
 * This file contains all pin definitions for the ESP32 board.
 * Pin mappings are based on ESP32 GPIO numbers (not Arduino pins).
 */

#ifndef PINS_H
#define PINS_H

#include <driver/gpio.h>

// Analog pins (ADC channels - ESP32 GPIO numbers)
#define PIN_A0   GPIO_NUM_1    // GPIO1  - ADC1_CH0
#define PIN_A1   GPIO_NUM_2    // GPIO2  - ADC1_CH1
#define PIN_A2   GPIO_NUM_3    // GPIO3  - ADC1_CH2
#define PIN_A3   GPIO_NUM_4    // GPIO4  - ADC1_CH3
#define PIN_A4   GPIO_NUM_5    // GPIO5  - ADC1_CH4
#define PIN_A5   GPIO_NUM_6    // GPIO6  - ADC1_CH5
#define PIN_A6   GPIO_NUM_7    // GPIO7  - ADC1_CH6
#define PIN_A7   GPIO_NUM_8    // GPIO8  - ADC1_CH7

// Digital pins (ESP32 GPIO numbers)
#define PIN_D0   GPIO_NUM_44   // GPIO44 - UART RX
#define PIN_D1   GPIO_NUM_43   // GPIO43 - UART TX
#define PIN_D2   GPIO_NUM_5    // GPIO5  - Digital pin 2
#define PIN_D3   GPIO_NUM_6    // GPIO6  - Digital pin 3
#define PIN_D4   GPIO_NUM_7    // GPIO7  - Digital pin 4
#define PIN_D5   GPIO_NUM_8    // GPIO8  - Digital pin 5
#define PIN_D6   GPIO_NUM_9    // GPIO9  - Digital pin 6
#define PIN_D7   GPIO_NUM_10   // GPIO10 - Digital pin 7
#define PIN_D8   GPIO_NUM_17   // GPIO17 - Digital pin 8
#define PIN_D9   GPIO_NUM_18   // GPIO18 - Digital pin 9
#define PIN_D10  GPIO_NUM_21   // GPIO21 - SPI CS
#define PIN_D11  GPIO_NUM_33   // GPIO33 - SPI COPI (MOSI)
#define PIN_D12  GPIO_NUM_34   // GPIO34 - SPI CIPO (MISO)
#define PIN_D13  GPIO_NUM_35   // GPIO35 - SPI SCK

// Power and control pins
#define PIN_VIN      GPIO_NUM_39   // VIN input
#define PIN_VBUS     GPIO_NUM_39   // USB VBUS (same as VIN)
#define PIN_3V3      GPIO_NUM_40   // 3.3V output
#define PIN_GND      GPIO_NUM_0    // Ground (dummy)
#define PIN_RESET    GPIO_NUM_41   // Reset button (internal)
#define PIN_BOOT0    GPIO_NUM_42   // Boot button
#define PIN_BOOT1    GPIO_NUM_2    // GPIO2 - Boot mode selection

// Onboard LED pin (Adafruit Feather ESP32)
#define PIN_ONBOARD_LED  GPIO_NUM_15  // GPIO15 - Onboard red LED

// I2C bus pins (ESP32 default)
#define PIN_I2C_SDA  GPIO_NUM_21   // GPIO21 - Default I2C SDA
#define PIN_I2C_SCL  GPIO_NUM_22   // GPIO22 - Default I2C SCL

// SPI bus pins (ESP32 default)
#define PIN_SPI_MOSI GPIO_NUM_23   // GPIO23 - Default SPI MOSI
#define PIN_SPI_MISO GPIO_NUM_19   // GPIO19 - Default SPI MISO
#define PIN_SPI_SCK  GPIO_NUM_18   // GPIO18 - Default SPI SCK
#define PIN_SPI_CS   GPIO_NUM_5    // GPIO5  - Default SPI CS

// UART pins (ESP32 default)
#define PIN_SERIAL_RX GPIO_NUM_3   // GPIO3  - Default UART RX
#define PIN_SERIAL_TX GPIO_NUM_1   // GPIO1  - Default UART TX

// Legacy sensor pin definitions (for compatibility - mapped to ESP32 GPIOs)
#define FLOWSENSOR    PIN_D2    // Flow sensor interrupt pin
#define TEMPSENSOR    PIN_D3    // One-wire temperature sensor
#define DISCINPUT1    PIN_D4    // Discrete input 1
#define DISCINPUT2    PIN_D5    // Discrete input 2
#define CONFIGPIN1    PIN_D6    // Configuration pin 1
#define CONFIGPIN2    PIN_D7    // Configuration pin 2
#define CONFIGPIN3    PIN_D8    // Configuration pin 3

// GPIO expander I2C addresses (from original design)
#define GPIO_EXPANDER_ADDR  0x20

// ADC I2C addresses (from original design)
#define ADS1115_ADDR        0x48  // 16-bit ADC
#define ADS1015_ADDR        0x49  // 12-bit ADC

// Environmental sensor I2C addresses
#define BME280_ADDR         0x77  // Atmospheric sensor

// RTC I2C address
#define RTC_ADDR            0x69  // RV1805 RTC

// OpenLog I2C address
#define OPENLOG_ADDR        0x2A  // OpenLog datalogger

// Buzzer I2C address
#define BUZZER_ADDR         0x34  // Qwiic Buzzer

// Pin function validation macros
#define IS_ANALOG_PIN(pin)  ((pin) >= GPIO_NUM_1 && (pin) <= GPIO_NUM_8)
#define IS_DIGITAL_PIN(pin) ((pin) >= GPIO_NUM_0 && (pin) <= GPIO_NUM_39 && (pin) != GPIO_NUM_24 && (pin) != GPIO_NUM_28 && (pin) != GPIO_NUM_29 && (pin) != GPIO_NUM_30 && (pin) != GPIO_NUM_31)
#define IS_I2C_PIN(pin)     ((pin) == PIN_I2C_SDA || (pin) == PIN_I2C_SCL)
#define IS_SPI_PIN(pin)     ((pin) == PIN_SPI_MOSI || (pin) == PIN_SPI_MISO || \
                             (pin) == PIN_SPI_SCK || (pin) == PIN_SPI_CS)
#define IS_UART_PIN(pin)    ((pin) == PIN_SERIAL_RX || (pin) == PIN_SERIAL_TX)

#endif /* PINS_H */
