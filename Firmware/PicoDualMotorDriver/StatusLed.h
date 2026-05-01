#pragma once

#include <Arduino.h>

struct RgbColor {
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

class StatusLed {
 public:
  bool begin();
  bool runStartupSequence();
  void update(bool errorActive, uint8_t enabledBridgeCount);
  bool isReady() const;

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool writeRegisters(uint8_t startReg, const uint8_t *values, size_t count);
  bool setColor(uint8_t red, uint8_t green, uint8_t blue);
  RgbColor selectNormalColor(uint8_t enabledBridgeCount) const;
  void markNotReady();

  bool ready_ = false;
  bool initialized_ = false;
  bool lastBlinkOn_ = false;
  bool lastErrorState_ = false;
  uint8_t lastEnabledBridgeCount_ = 0xFF;
  RgbColor lastColor_ = {0, 0, 0};
};
