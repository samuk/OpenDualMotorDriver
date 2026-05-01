#include "As5600Encoder.h"

bool As5600Encoder::begin(TwoWire &bus, uint8_t address) {
  bus_ = &bus;
  address_ = address;
  bus_->setClock(BoardConfig::AS5600_I2C_FREQUENCY_HZ);
  healthy_ = false;
  haveSample_ = false;
  nativeCounts_ = 0;
  adjustedCounts_ = 0;
  reducedCounts_ = 0;
  positionCounts_ = 0;
  previousAdjustedCounts_ = 0;
  speedCountsPerSec_ = 0.0f;
  lastUpdateMs_ = millis();
  return true;
}

void As5600Encoder::setResolutionBits(uint8_t bits) {
  if (bits < BoardConfig::AS5600_MIN_REDUCED_BITS) {
    bits = BoardConfig::AS5600_MIN_REDUCED_BITS;
  } else if (bits > BoardConfig::AS5600_MAX_REDUCED_BITS) {
    bits = BoardConfig::AS5600_MAX_REDUCED_BITS;
  }
  resolutionBits_ = bits;
}

uint8_t As5600Encoder::resolutionBits() const {
  return resolutionBits_;
}

void As5600Encoder::setZeroOffsetCounts(uint16_t counts) {
  zeroOffsetCounts_ = normalizeCounts(counts);
}

uint16_t As5600Encoder::zeroOffsetCounts() const {
  return zeroOffsetCounts_;
}

void As5600Encoder::setInverted(bool inverted) {
  inverted_ = inverted;
}

bool As5600Encoder::inverted() const {
  return inverted_;
}

bool As5600Encoder::sample(uint32_t nowMs) {
  if (bus_ == nullptr) {
    healthy_ = false;
    return false;
  }

  uint16_t nativeCounts = 0;
  if (!readNativeCounts(nativeCounts)) {
    healthy_ = false;
    return false;
  }

  nativeCounts_ = normalizeCounts(nativeCounts);
  adjustedCounts_ = applyOffsetAndDirection(nativeCounts_);
  reducedCounts_ = static_cast<uint16_t>(adjustedCounts_ >> (BoardConfig::AS5600_RAW_BITS - resolutionBits_));

  if (!haveSample_) {
    positionCounts_ = adjustedCounts_;
    previousAdjustedCounts_ = adjustedCounts_;
    speedCountsPerSec_ = 0.0f;
    haveSample_ = true;
    healthy_ = true;
    lastUpdateMs_ = nowMs;
    return true;
  }

  const uint32_t elapsedMs = nowMs - lastUpdateMs_;
  const int16_t delta = wrapDelta(static_cast<int16_t>(adjustedCounts_ - previousAdjustedCounts_));
  positionCounts_ += delta;
  previousAdjustedCounts_ = adjustedCounts_;

  if (elapsedMs > 0) {
    const float rawSpeed = static_cast<float>(delta) * 1000.0f / static_cast<float>(elapsedMs);
    speedCountsPerSec_ = (BoardConfig::AS5600_SPEED_FILTER_ALPHA * rawSpeed) +
                         ((1.0f - BoardConfig::AS5600_SPEED_FILTER_ALPHA) * speedCountsPerSec_);
  }

  lastUpdateMs_ = nowMs;
  healthy_ = true;
  return true;
}

bool As5600Encoder::zeroCurrentPosition(uint32_t nowMs) {
  if (bus_ == nullptr) {
    healthy_ = false;
    return false;
  }

  uint16_t nativeCounts = 0;
  if (!readNativeCounts(nativeCounts)) {
    healthy_ = false;
    return false;
  }

  nativeCounts_ = normalizeCounts(nativeCounts);
  zeroOffsetCounts_ = nativeCounts_;
  adjustedCounts_ = 0;
  reducedCounts_ = 0;
  positionCounts_ = 0;
  previousAdjustedCounts_ = 0;
  speedCountsPerSec_ = 0.0f;
  haveSample_ = true;
  healthy_ = true;
  lastUpdateMs_ = nowMs;
  return true;
}

uint16_t As5600Encoder::nativeCounts() const {
  return nativeCounts_;
}

uint16_t As5600Encoder::reducedCounts() const {
  return reducedCounts_;
}

uint16_t As5600Encoder::adjustedCounts() const {
  return adjustedCounts_;
}

int32_t As5600Encoder::positionCounts() const {
  return positionCounts_;
}

float As5600Encoder::positionTurns() const {
  return static_cast<float>(positionCounts_) / static_cast<float>(kCountsPerRev);
}

float As5600Encoder::positionDegrees() const {
  return static_cast<float>(positionCounts_) * BoardConfig::AS5600_DEGREES_PER_COUNT;
}

float As5600Encoder::speedCountsPerSec() const {
  return speedCountsPerSec_;
}

float As5600Encoder::speedTurnsPerSec() const {
  return speedCountsPerSec_ / static_cast<float>(kCountsPerRev);
}

float As5600Encoder::speedRpm() const {
  return speedTurnsPerSec() * 60.0f;
}

bool As5600Encoder::healthy() const {
  return healthy_;
}

uint32_t As5600Encoder::lastUpdateMs() const {
  return lastUpdateMs_;
}

int16_t As5600Encoder::wrapDelta(int16_t delta) {
  if (delta > static_cast<int16_t>(kCountsPerRev / 2U)) {
    delta -= static_cast<int16_t>(kCountsPerRev);
  } else if (delta < -static_cast<int16_t>(kCountsPerRev / 2U)) {
    delta += static_cast<int16_t>(kCountsPerRev);
  }
  return delta;
}

uint16_t As5600Encoder::normalizeCounts(uint16_t counts) {
  return static_cast<uint16_t>(counts & (kCountsPerRev - 1U));
}

bool As5600Encoder::readNativeCounts(uint16_t &counts) const {
  if (bus_ == nullptr) {
    return false;
  }

  bus_->beginTransmission(address_);
  bus_->write(kRawAngleRegister);
  if (bus_->endTransmission(false) != 0) {
    return false;
  }

  const size_t requested = bus_->requestFrom(address_, static_cast<size_t>(2));
  if (requested != 2) {
    return false;
  }

  const int highByte = bus_->read();
  const int lowByte = bus_->read();
  if (highByte < 0 || lowByte < 0) {
    return false;
  }

  counts = static_cast<uint16_t>(((static_cast<uint16_t>(highByte) << 8U) |
                                  static_cast<uint16_t>(lowByte)) & 0x0FFFU);
  return true;
}

uint16_t As5600Encoder::applyOffsetAndDirection(uint16_t counts) const {
  uint16_t adjusted = static_cast<uint16_t>((counts + kCountsPerRev - zeroOffsetCounts_) & (kCountsPerRev - 1U));
  if (inverted_) {
    adjusted = static_cast<uint16_t>((kCountsPerRev - adjusted) & (kCountsPerRev - 1U));
  }
  return adjusted;
}
