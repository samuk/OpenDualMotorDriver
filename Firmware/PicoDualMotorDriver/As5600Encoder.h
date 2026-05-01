#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "BoardConfig.h"

class As5600Encoder {
 public:
  bool begin(TwoWire &bus = Wire, uint8_t address = BoardConfig::AS5600_ADDRESS);

  void setResolutionBits(uint8_t bits);
  uint8_t resolutionBits() const;

  void setZeroOffsetCounts(uint16_t counts);
  uint16_t zeroOffsetCounts() const;

  void setInverted(bool inverted);
  bool inverted() const;

  bool sample(uint32_t nowMs = millis());
  bool zeroCurrentPosition(uint32_t nowMs = millis());

  uint16_t nativeCounts() const;
  uint16_t reducedCounts() const;
  uint16_t adjustedCounts() const;
  int32_t positionCounts() const;
  float positionTurns() const;
  float positionDegrees() const;
  float speedCountsPerSec() const;
  float speedTurnsPerSec() const;
  float speedRpm() const;

  bool healthy() const;
  uint32_t lastUpdateMs() const;

 private:
  static constexpr uint8_t kRawAngleRegister = 0x0C;
  static constexpr uint16_t kCountsPerRev = BoardConfig::AS5600_COUNTS_PER_REV;

  static int16_t wrapDelta(int16_t delta);
  static uint16_t normalizeCounts(uint16_t counts);

  bool readNativeCounts(uint16_t &counts) const;
  uint16_t applyOffsetAndDirection(uint16_t counts) const;

  TwoWire *bus_ = nullptr;
  uint8_t address_ = BoardConfig::AS5600_ADDRESS;
  uint8_t resolutionBits_ = BoardConfig::AS5600_DEFAULT_REDUCED_BITS;
  uint16_t zeroOffsetCounts_ = 0;
  bool inverted_ = false;
  bool healthy_ = false;
  bool haveSample_ = false;
  uint16_t nativeCounts_ = 0;
  uint16_t adjustedCounts_ = 0;
  uint16_t reducedCounts_ = 0;
  int32_t positionCounts_ = 0;
  uint16_t previousAdjustedCounts_ = 0;
  float speedCountsPerSec_ = 0.0f;
  uint32_t lastUpdateMs_ = 0;
};
