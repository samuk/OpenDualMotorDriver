#pragma once

#include <Arduino.h>

namespace BoardConfig {

constexpr uint8_t BRIDGE_COUNT = 2;

// Pin map from the custom Pico 2 carrier.
constexpr uint8_t PIN_UART0_TX = 0;
constexpr uint8_t PIN_UART0_RX = 1;
constexpr uint8_t PIN_I2C1_SDA = 2;
constexpr uint8_t PIN_I2C1_SCL = 3;
constexpr uint8_t PIN_I2C0_PULLUP_EN = 4;
constexpr uint8_t PIN_UNUSED_GPIO5 = 5;
constexpr uint8_t PIN_UNUSED_GPIO6 = 6;
constexpr uint8_t PIN_I2C1_PULLUP_EN = 7;
constexpr uint8_t PIN_DRIVER_PWM_C = 8;
constexpr uint8_t PIN_DRIVER_RESET_CD = 9;
constexpr uint8_t PIN_DRIVER_PWM_D = 10;
constexpr uint8_t PIN_DRIVER_PWM_A = 11;
constexpr uint8_t PIN_DRIVER_RESET_AB = 12;
constexpr uint8_t PIN_DRIVER_PWM_B = 13;
constexpr uint8_t PIN_DRIVER_FAULT = 14;
constexpr uint8_t PIN_DRIVER_OTW = 15;
constexpr uint8_t PIN_SPI_MISO = 16;  // Routed only to the connector, unused in firmware.
constexpr uint8_t PIN_SPI_CS = 17;    // Routed only to the connector, unused in firmware.
constexpr uint8_t PIN_SPI_SCK = 18;   // Routed only to the connector, unused in firmware.
constexpr uint8_t PIN_SPI_MOSI = 19;  // Routed only to the connector, unused in firmware.
constexpr uint8_t PIN_I2C0_SDA = 20;
constexpr uint8_t PIN_I2C0_SCL = 21;
constexpr uint8_t PIN_ACS722_OUT1 = 26;
constexpr uint8_t PIN_ACS722_OUT2 = 27;
constexpr uint8_t PIN_VIN_ADC = 28;

constexpr bool I2C0_PULLUPS_DEFAULT_ENABLED = true;
constexpr bool I2C1_PULLUPS_DEFAULT_ENABLED = true;

constexpr uint8_t PCA9633_ADDRESS = 0x62;
constexpr uint8_t I2C_SLAVE_ADDRESS = 0x16;
constexpr uint8_t AS5600_ADDRESS = 0x36;

constexpr uint32_t USB_SERIAL_BAUD = 115200;
constexpr uint32_t UART_BAUD = 115200;
constexpr uint32_t I2C0_FREQUENCY_HZ = 400000;
constexpr uint32_t I2C1_FREQUENCY_HZ = 400000;
constexpr uint32_t AS5600_I2C_FREQUENCY_HZ = 400000;

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint16_t PWM_RANGE = 1000;
constexpr uint16_t PWM_ACTIVE_MAX = 950;  // Cap duty to 95% and keep bootstrap refresh margin for the DRV8412.

constexpr uint16_t AS5600_COUNTS_PER_REV = 4096;
constexpr uint8_t AS5600_RAW_BITS = 12;
constexpr uint8_t AS5600_DEFAULT_REDUCED_BITS = 10;
constexpr uint8_t AS5600_MIN_REDUCED_BITS = 10;
constexpr uint8_t AS5600_MAX_REDUCED_BITS = 12;
constexpr uint8_t CLOSED_LOOP_DEFAULT_BRIDGE = 0;
constexpr float AS5600_DEGREES_PER_COUNT = 360.0f / static_cast<float>(AS5600_COUNTS_PER_REV);
// Run the encoder-derived speed estimate at the same cadence as the closed-loop update.
// 4 ms keeps the velocity estimate responsive without the quantization noise seen at 1 ms.
constexpr float AS5600_SPEED_FILTER_ALPHA = 0.40f;
constexpr uint16_t CLOSED_LOOP_CONTROL_PERIOD_MS = 4;
constexpr float CLOSED_LOOP_MAX_OUTPUT = 1000.0f;
constexpr float CLOSED_LOOP_MAX_INTEGRAL = 2000.0f;
constexpr float CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC = 262144.0f;
constexpr float CLOSED_LOOP_MAX_POSITION_REF_COUNTS = 1000000.0f;

constexpr float DEFAULT_SPEED_KP = 0.20f;
constexpr float DEFAULT_SPEED_KI = 0.02f;
constexpr float DEFAULT_SPEED_KD = 0.00f;
constexpr float DEFAULT_POSITION_KP = 0.10f;
constexpr float DEFAULT_POSITION_KI = 0.00f;
constexpr float DEFAULT_POSITION_KD = 0.00f;

constexpr uint16_t STATUS_LED_NORMAL_PERIOD_MS = 1000;
constexpr uint16_t STATUS_LED_ERROR_PERIOD_MS = 200;
constexpr uint8_t STATUS_LED_BRIGHTNESS = 255;
constexpr uint8_t STATUS_LED_STARTUP_LOOPS = 3;
constexpr uint16_t STATUS_LED_STARTUP_HOLD_MS = 140;
constexpr uint16_t STATUS_LED_STARTUP_OFF_MS = 60;

constexpr uint16_t DRV8412_RESET_PULSE_MS = 2;
constexpr uint32_t DRV8412_FAULT_RESET_HOLDOFF_MS = 1000;

constexpr uint8_t ADC_SAMPLE_COUNT = 8;
constexpr uint16_t ADC_REFERENCE_MV = 3300;
constexpr uint16_t ADC_MAX_COUNTS = 4095;

constexpr float VIN_DIVIDER_SCALE = 11.0f;      // 100k over 10k divider.
constexpr float VIN_CALIBRATION_SCALE = 1.0f;   // Leave at 1.0 until board-level calibration is done.

constexpr int16_t ACS722_ZERO_CURRENT_MV = 1650;
constexpr uint16_t ACS722_SENSITIVITY_MV_PER_A = 132;
constexpr int16_t ACS722_CHANNEL1_ZERO_TRIM_MV = 0;
constexpr int16_t ACS722_CHANNEL2_ZERO_TRIM_MV = 0;
constexpr int8_t ACS722_CHANNEL1_POLARITY = 1;
constexpr int8_t ACS722_CHANNEL2_POLARITY = 1;

constexpr uint16_t FAULT_POLL_INTERVAL_MS = 5;
constexpr uint16_t ADC_POLL_INTERVAL_MS = 20;
constexpr uint16_t MAIN_LOOP_DELAY_MS = 1;

constexpr uint16_t TEXT_REPORT_DEFAULT_INTERVAL_MS = 0;
constexpr uint16_t BINARY_REPORT_DEFAULT_INTERVAL_MS = 0;
constexpr bool FAST_BINARY_MODE_DEFAULT = false;

constexpr size_t MAX_TEXT_COMMAND_LENGTH = 120;
constexpr size_t MAX_BINARY_PACKET_BYTES = 128;
constexpr size_t MAX_I2C_RX_BYTES = 128;
constexpr size_t MAX_RESPONSE_BYTES = 384;

}  // namespace BoardConfig
