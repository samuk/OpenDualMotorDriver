#include "ClosedLoopController.h"

bool ClosedLoopController::begin(As5600Encoder *encoder, uint8_t bridgeIndex) {
  encoder_ = encoder;
  bridgeIndex_ = bridgeIndex;
  enabled_ = false;
  mode_ = Mode::Manual;
  manualCommandPermille_ = 0;
  speedReferenceCountsPerSec_ = 0.0f;
  positionReferenceCounts_ = 0;
  positionSpeedLimitCountsPerSec_ = BoardConfig::CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC;

  speedPid_.setGains(BoardConfig::DEFAULT_SPEED_KP,
                     BoardConfig::DEFAULT_SPEED_KI,
                     BoardConfig::DEFAULT_SPEED_KD);
  speedPid_.setOutputLimits(-BoardConfig::CLOSED_LOOP_MAX_OUTPUT, BoardConfig::CLOSED_LOOP_MAX_OUTPUT);
  speedPid_.setIntegralLimit(BoardConfig::CLOSED_LOOP_MAX_INTEGRAL);
  speedPid_.reset();

  positionPid_.setGains(BoardConfig::DEFAULT_POSITION_KP,
                        BoardConfig::DEFAULT_POSITION_KI,
                        BoardConfig::DEFAULT_POSITION_KD);
  positionPid_.setOutputLimits(-positionSpeedLimitCountsPerSec_, positionSpeedLimitCountsPerSec_);
  positionPid_.setIntegralLimit(BoardConfig::CLOSED_LOOP_MAX_INTEGRAL);
  positionPid_.reset();

  reset();
  return true;
}

void ClosedLoopController::attachEncoder(As5600Encoder *encoder) {
  encoder_ = encoder;
}

void ClosedLoopController::setBridgeIndex(uint8_t bridgeIndex) {
  bridgeIndex_ = bridgeIndex;
}

uint8_t ClosedLoopController::bridgeIndex() const {
  return bridgeIndex_;
}

void ClosedLoopController::setEnabled(bool enabled) {
  enabled_ = enabled;
}

bool ClosedLoopController::enabled() const {
  return enabled_;
}

void ClosedLoopController::setMode(Mode mode) {
  mode_ = mode;
}

ClosedLoopController::Mode ClosedLoopController::mode() const {
  return mode_;
}

void ClosedLoopController::setManualCommand(int16_t commandPermille) {
  manualCommandPermille_ = clampPermille(commandPermille);
}

int16_t ClosedLoopController::manualCommand() const {
  return manualCommandPermille_;
}

void ClosedLoopController::setSpeedReferenceCountsPerSec(float countsPerSec) {
  speedReferenceCountsPerSec_ = clampFloat(countsPerSec,
                                           -BoardConfig::CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC,
                                           BoardConfig::CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC);
}

void ClosedLoopController::setSpeedReferenceRpm(float rpm) {
  setSpeedReferenceCountsPerSec((rpm * static_cast<float>(BoardConfig::AS5600_COUNTS_PER_REV)) / 60.0f);
}

float ClosedLoopController::speedReferenceCountsPerSec() const {
  return speedReferenceCountsPerSec_;
}

void ClosedLoopController::setPositionReferenceCounts(int32_t counts) {
  const float clamped = clampFloat(static_cast<float>(counts),
                                   -BoardConfig::CLOSED_LOOP_MAX_POSITION_REF_COUNTS,
                                   BoardConfig::CLOSED_LOOP_MAX_POSITION_REF_COUNTS);
  positionReferenceCounts_ = static_cast<int32_t>(clamped);
}

void ClosedLoopController::setPositionReferenceDegrees(float degrees) {
  const float counts = (degrees / 360.0f) * static_cast<float>(BoardConfig::AS5600_COUNTS_PER_REV);
  setPositionReferenceCounts(static_cast<int32_t>(counts >= 0.0f ? (counts + 0.5f) : (counts - 0.5f)));
}

void ClosedLoopController::setPositionReferenceTurns(float turns) {
  const float counts = turns * static_cast<float>(BoardConfig::AS5600_COUNTS_PER_REV);
  setPositionReferenceCounts(static_cast<int32_t>(counts >= 0.0f ? (counts + 0.5f) : (counts - 0.5f)));
}

int32_t ClosedLoopController::positionReferenceCounts() const {
  return positionReferenceCounts_;
}

void ClosedLoopController::setSpeedPidGains(float kp, float ki, float kd) {
  speedPid_.setGains(kp, ki, kd);
}

void ClosedLoopController::setPositionPidGains(float kp, float ki, float kd) {
  positionPid_.setGains(kp, ki, kd);
}

PidController::Gains ClosedLoopController::speedPidGains() const {
  return speedPid_.gains();
}

PidController::Gains ClosedLoopController::positionPidGains() const {
  return positionPid_.gains();
}

void ClosedLoopController::setSpeedPidOutputLimits(float minOutputPermille, float maxOutputPermille) {
  speedPid_.setOutputLimits(minOutputPermille, maxOutputPermille);
}

void ClosedLoopController::setPositionPidSpeedLimits(float minCountsPerSec, float maxCountsPerSec) {
  if (minCountsPerSec > maxCountsPerSec) {
    const float swap = minCountsPerSec;
    minCountsPerSec = maxCountsPerSec;
    maxCountsPerSec = swap;
  }

  const float absMin = minCountsPerSec < 0.0f ? -minCountsPerSec : minCountsPerSec;
  const float absMax = maxCountsPerSec < 0.0f ? -maxCountsPerSec : maxCountsPerSec;
  const float symmetricLimit = absMin > absMax ? absMin : absMax;

  positionSpeedLimitCountsPerSec_ = clampFloat(symmetricLimit,
                                               0.0f,
                                               BoardConfig::CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC);
  positionPid_.setOutputLimits(-positionSpeedLimitCountsPerSec_, positionSpeedLimitCountsPerSec_);
}

void ClosedLoopController::setSpeedPidIntegralLimit(float limit) {
  speedPid_.setIntegralLimit(limit);
}

void ClosedLoopController::setPositionPidIntegralLimit(float limit) {
  positionPid_.setIntegralLimit(limit);
}

void ClosedLoopController::reset() {
  speedPid_.reset();
  positionPid_.reset();
  snapshot_ = Snapshot{};
  snapshot_.mode = mode_;
  snapshot_.enabled = enabled_;
  snapshot_.bridgeIndex = bridgeIndex_;
  snapshot_.manualCommandPermille = manualCommandPermille_;
  snapshot_.referencePositionCounts = positionReferenceCounts_;
  snapshot_.referenceSpeedCountsPerSec = speedReferenceCountsPerSec_;
  snapshot_.positionGains = positionPid_.gains();
  snapshot_.speedGains = speedPid_.gains();
  lastUpdateMs_ = millis();
  initialized_ = false;
}

bool ClosedLoopController::update(uint32_t nowMs) {
  if (initialized_ && nowMs - lastUpdateMs_ < BoardConfig::CLOSED_LOOP_CONTROL_PERIOD_MS) {
    return false;
  }

  if (encoder_ != nullptr) {
    encoder_->sample(nowMs);
  }

  const uint32_t elapsedMs =
      initialized_ ? (nowMs - lastUpdateMs_) : static_cast<uint32_t>(BoardConfig::CLOSED_LOOP_CONTROL_PERIOD_MS);
  const float dtSeconds = static_cast<float>(elapsedMs) / 1000.0f;

  snapshot_.mode = mode_;
  snapshot_.enabled = enabled_;
  snapshot_.bridgeIndex = bridgeIndex_;
  snapshot_.manualCommandPermille = manualCommandPermille_;
  snapshot_.referencePositionCounts = positionReferenceCounts_;
  snapshot_.referenceSpeedCountsPerSec = speedReferenceCountsPerSec_;
  snapshot_.lastUpdateMs = nowMs;

  if (encoder_ != nullptr) {
    snapshot_.encoderNativeCounts = encoder_->nativeCounts();
    snapshot_.encoderReducedCounts = encoder_->reducedCounts();
    snapshot_.measuredPositionCounts = encoder_->positionCounts();
    snapshot_.measuredSpeedCountsPerSec = encoder_->speedCountsPerSec();
    snapshot_.encoderHealthy = encoder_->healthy();
  } else {
    snapshot_.encoderNativeCounts = 0;
    snapshot_.encoderReducedCounts = 0;
    snapshot_.measuredPositionCounts = 0;
    snapshot_.measuredSpeedCountsPerSec = 0.0f;
    snapshot_.encoderHealthy = false;
  }

  snapshot_.positionGains = positionPid_.gains();
  snapshot_.speedGains = speedPid_.gains();

  float output = 0.0f;
  if (!enabled_) {
    output = 0.0f;
  } else if (mode_ == Mode::Manual) {
    output = static_cast<float>(manualCommandPermille_);
  } else if (encoder_ == nullptr || !snapshot_.encoderHealthy) {
    output = 0.0f;
  } else if (mode_ == Mode::SpeedPid) {
    output = speedPid_.update(speedReferenceCountsPerSec_, snapshot_.measuredSpeedCountsPerSec, dtSeconds);
  } else if (mode_ == Mode::PositionPid) {
    const float speedReference = positionPid_.update(static_cast<float>(positionReferenceCounts_),
                                                     static_cast<float>(snapshot_.measuredPositionCounts),
                                                     dtSeconds);
    const float clampedSpeedReference = clampFloat(speedReference,
                                                   -positionSpeedLimitCountsPerSec_,
                                                   positionSpeedLimitCountsPerSec_);
    output = speedPid_.update(clampedSpeedReference, snapshot_.measuredSpeedCountsPerSec, dtSeconds);
  }

  const int16_t permille = clampPermille(static_cast<int32_t>(output >= 0.0f ? (output + 0.5f) : (output - 0.5f)));
  snapshot_.outputCommandPermille = permille;
  lastUpdateMs_ = nowMs;
  initialized_ = true;
  if (!enabled_ || mode_ == Mode::Manual) {
    return true;
  }

  return encoder_ != nullptr && snapshot_.encoderHealthy;
}

int16_t ClosedLoopController::outputCommandPermille() const {
  return snapshot_.outputCommandPermille;
}

const ClosedLoopController::Snapshot &ClosedLoopController::snapshot() const {
  return snapshot_;
}

int16_t ClosedLoopController::clampPermille(int32_t value) {
  if (value > static_cast<int32_t>(BoardConfig::CLOSED_LOOP_MAX_OUTPUT)) {
    return static_cast<int16_t>(BoardConfig::CLOSED_LOOP_MAX_OUTPUT);
  }
  if (value < -static_cast<int32_t>(BoardConfig::CLOSED_LOOP_MAX_OUTPUT)) {
    return static_cast<int16_t>(-BoardConfig::CLOSED_LOOP_MAX_OUTPUT);
  }
  return static_cast<int16_t>(value);
}

float ClosedLoopController::clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}
