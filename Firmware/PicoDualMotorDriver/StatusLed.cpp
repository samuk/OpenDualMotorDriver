#include "StatusLed.h"

#include <Wire.h>

#include "BoardConfig.h"

namespace {

constexpr uint8_t PCA9633_REG_MODE1 = 0x00;
constexpr uint8_t PCA9633_REG_MODE2 = 0x01;
constexpr uint8_t PCA9633_REG_PWM0 = 0x02;
constexpr uint8_t PCA9633_REG_LEDOUT = 0x08;
constexpr uint8_t PCA9633_LEDOUT_PWM_RGB = 0x2A;  // LED0-2 in PWM mode, LED3 off.

}  // namespace

bool StatusLed::begin() {
  Wire.setSDA(BoardConfig::PIN_I2C0_SDA);
  Wire.setSCL(BoardConfig::PIN_I2C0_SCL);
  Wire.begin();
  Wire.setClock(BoardConfig::I2C0_FREQUENCY_HZ);

  ready_ = writeRegister(PCA9633_REG_MODE1, 0x00) &&
           writeRegister(PCA9633_REG_MODE2, 0x04) &&
           setColor(0, 0, 0);

  initialized_ = false;
  return ready_;
}

bool StatusLed::runStartupSequence() {
  if (!ready_) {
    return false;
  }

  for (uint8_t i = 0; i < BoardConfig::STATUS_LED_STARTUP_LOOPS; ++i) {
    if (!setColor(BoardConfig::STATUS_LED_BRIGHTNESS, 0, 0)) {
      markNotReady();
      return false;
    }
    delay(BoardConfig::STATUS_LED_STARTUP_HOLD_MS);

    if (!setColor(0, BoardConfig::STATUS_LED_BRIGHTNESS, 0)) {
      markNotReady();
      return false;
    }
    delay(BoardConfig::STATUS_LED_STARTUP_HOLD_MS);

    if (!setColor(0, 0, BoardConfig::STATUS_LED_BRIGHTNESS)) {
      markNotReady();
      return false;
    }
    delay(BoardConfig::STATUS_LED_STARTUP_HOLD_MS);

    if (!setColor(0, 0, 0)) {
      markNotReady();
      return false;
    }
    delay(BoardConfig::STATUS_LED_STARTUP_OFF_MS);
  }

  initialized_ = false;
  return true;
}

void StatusLed::update(bool errorActive, uint8_t enabledBridgeCount) {
  if (!ready_) {
    return;
  }

  const uint16_t blinkPeriodMs = errorActive ? BoardConfig::STATUS_LED_ERROR_PERIOD_MS
                                             : BoardConfig::STATUS_LED_NORMAL_PERIOD_MS;
  const bool blinkOn = (millis() % blinkPeriodMs) < (blinkPeriodMs / 2);

  RgbColor targetColor = {0, 0, 0};
  if (blinkOn) {
    targetColor = errorActive ? RgbColor{BoardConfig::STATUS_LED_BRIGHTNESS, 0, 0}
                              : selectNormalColor(enabledBridgeCount);
  }

  const bool colorChanged = targetColor.red != lastColor_.red ||
                            targetColor.green != lastColor_.green ||
                            targetColor.blue != lastColor_.blue;

  if (initialized_ &&
      blinkOn == lastBlinkOn_ &&
      errorActive == lastErrorState_ &&
      enabledBridgeCount == lastEnabledBridgeCount_ &&
      !colorChanged) {
    return;
  }

  if (!setColor(targetColor.red, targetColor.green, targetColor.blue)) {
    markNotReady();
    return;
  }

  initialized_ = true;
  lastBlinkOn_ = blinkOn;
  lastErrorState_ = errorActive;
  lastEnabledBridgeCount_ = enabledBridgeCount;
  lastColor_ = targetColor;
}

bool StatusLed::isReady() const {
  return ready_;
}

bool StatusLed::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BoardConfig::PCA9633_ADDRESS);
  Wire.write(reg & 0x0F);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool StatusLed::writeRegisters(uint8_t startReg, const uint8_t *values, size_t count) {
  Wire.beginTransmission(BoardConfig::PCA9633_ADDRESS);
  Wire.write(0x80 | (startReg & 0x0F));

  for (size_t i = 0; i < count; ++i) {
    Wire.write(values[i]);
  }

  return Wire.endTransmission() == 0;
}

bool StatusLed::setColor(uint8_t red, uint8_t green, uint8_t blue) {
  const uint8_t pwmValues[] = {blue, red, green, 0};
  return writeRegisters(PCA9633_REG_PWM0, pwmValues, sizeof(pwmValues)) &&
         writeRegister(PCA9633_REG_LEDOUT, PCA9633_LEDOUT_PWM_RGB);
}

RgbColor StatusLed::selectNormalColor(uint8_t enabledBridgeCount) const {
  if (enabledBridgeCount == 0) {
    return {0, BoardConfig::STATUS_LED_BRIGHTNESS, 0};
  }

  if (enabledBridgeCount == 1) {
    return {0, 0, BoardConfig::STATUS_LED_BRIGHTNESS};
  }

  return {BoardConfig::STATUS_LED_BRIGHTNESS, 0, BoardConfig::STATUS_LED_BRIGHTNESS};
}

void StatusLed::markNotReady() {
  ready_ = false;
}
