#pragma once

#include <Arduino.h>

#include "As5600Encoder.h"
#include "BoardConfig.h"
#include "PidController.h"

class ClosedLoopController {
 public:
  enum class Mode : uint8_t {
    Manual = 0,
    SpeedPid = 1,
    PositionPid = 2,
  };

  struct Snapshot {
    Mode mode = Mode::Manual;
    bool enabled = false;
    uint8_t bridgeIndex = 0;
    int16_t manualCommandPermille = 0;
    int16_t outputCommandPermille = 0;
    int32_t referencePositionCounts = 0;
    float referenceSpeedCountsPerSec = 0.0f;
    int32_t measuredPositionCounts = 0;
    float measuredSpeedCountsPerSec = 0.0f;
    uint16_t encoderNativeCounts = 0;
    uint16_t encoderReducedCounts = 0;
    bool encoderHealthy = false;
    PidController::Gains positionGains;
    PidController::Gains speedGains;
    uint32_t lastUpdateMs = 0;
  };

  bool begin(As5600Encoder *encoder = nullptr, uint8_t bridgeIndex = 0);

  void attachEncoder(As5600Encoder *encoder);
  void setBridgeIndex(uint8_t bridgeIndex);
  uint8_t bridgeIndex() const;

  void setEnabled(bool enabled);
  bool enabled() const;

  void setMode(Mode mode);
  Mode mode() const;

  void setManualCommand(int16_t commandPermille);
  int16_t manualCommand() const;

  void setSpeedReferenceCountsPerSec(float countsPerSec);
  void setSpeedReferenceRpm(float rpm);
  float speedReferenceCountsPerSec() const;

  void setPositionReferenceCounts(int32_t counts);
  void setPositionReferenceDegrees(float degrees);
  void setPositionReferenceTurns(float turns);
  int32_t positionReferenceCounts() const;

  void setSpeedPidGains(float kp, float ki, float kd);
  void setPositionPidGains(float kp, float ki, float kd);
  PidController::Gains speedPidGains() const;
  PidController::Gains positionPidGains() const;

  void setSpeedPidOutputLimits(float minOutputPermille, float maxOutputPermille);
  void setPositionPidSpeedLimits(float minCountsPerSec, float maxCountsPerSec);
  void setSpeedPidIntegralLimit(float limit);
  void setPositionPidIntegralLimit(float limit);

  void reset();
  bool update(uint32_t nowMs = millis());

  int16_t outputCommandPermille() const;
  const Snapshot &snapshot() const;

 private:
  static int16_t clampPermille(int32_t value);
  static float clampFloat(float value, float minValue, float maxValue);

  As5600Encoder *encoder_ = nullptr;
  uint8_t bridgeIndex_ = 0;
  bool enabled_ = false;
  Mode mode_ = Mode::Manual;
  int16_t manualCommandPermille_ = 0;
  float speedReferenceCountsPerSec_ = 0.0f;
  int32_t positionReferenceCounts_ = 0;
  float positionSpeedLimitCountsPerSec_ = BoardConfig::CLOSED_LOOP_MAX_SPEED_REF_COUNTS_PER_SEC;
  PidController speedPid_;
  PidController positionPid_;
  Snapshot snapshot_;
  bool initialized_ = false;
  uint32_t lastUpdateMs_ = 0;
};
