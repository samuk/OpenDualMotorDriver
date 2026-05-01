#pragma once

#include <Arduino.h>

#include "BoardConfig.h"

struct ControllerSnapshot {
  uint32_t uptimeMs = 0;
  bool faultActive = false;
  bool otwActive = false;
  bool bridgeEnabled[BoardConfig::BRIDGE_COUNT] = {false, false};
  bool bridgeDirectionInverted[BoardConfig::BRIDGE_COUNT] = {false, false};
  int16_t bridgePowerPermille[BoardConfig::BRIDGE_COUNT] = {0, 0};
  uint16_t bridgeDuty[BoardConfig::BRIDGE_COUNT] = {0, 0};
  bool i2cPullupsEnabled[2] = {false, false};
  uint16_t vinAdc = 0;
  uint16_t currentAdc[BoardConfig::BRIDGE_COUNT] = {0, 0};
  uint16_t vinMilliVolts = 0;
  int16_t currentMilliAmps[BoardConfig::BRIDGE_COUNT] = {0, 0};
};

class DualMotorController {
 public:
  DualMotorController();

  void begin();
  void service();

  bool setBridgeEnabled(int bridgeIndex, bool enable);
  bool setMotorPower(int bridgeIndex, int16_t powerPermille);
  void stopMotor(int bridgeIndex);
  void stopAll();
  bool clearFaults(int bridgeIndex = -1);

  void setI2cPullupEnabled(int busIndex, bool enable);
  bool isI2cPullupEnabled(int busIndex) const;
  bool setBridgeDirectionInverted(int bridgeIndex, bool inverted);
  bool isBridgeDirectionInverted(int bridgeIndex) const;

  ControllerSnapshot readSnapshot();
  uint8_t getEnabledBridgeCount() const;

  bool isFaultActive() const;
  bool isOtwActive() const;
  uint32_t getFaultResetHoldoffRemainingMs() const;

 private:
  struct BridgePins {
    uint8_t pwmForwardPin;
    uint8_t pwmReversePin;
    uint8_t resetPin;
  };

  struct BridgeState {
    bool enabled = false;
    bool directionInverted = false;
    int16_t powerPermille = 0;
    uint16_t pwmDuty = 0;
  };

  void configurePins();
  void updateFaultInputs();
  void refreshMeasurements();
  void writeBridgePinsLow(uint8_t bridgeIndex);
  void applyBridgeOutput(uint8_t bridgeIndex);
  uint16_t readAdcAverage(uint8_t pin) const;
  uint16_t adcCountsToMilliVolts(uint16_t counts) const;
  uint16_t adcCountsToVinMilliVolts(uint16_t counts) const;
  int16_t adcCountsToCurrentMilliAmps(uint16_t counts, uint8_t channelIndex) const;
  bool isValidBridgeIndex(int bridgeIndex) const;
  bool canClearFaultsNow() const;
  void clearFaultOnBridge(uint8_t bridgeIndex);

  BridgePins bridgePins_[BoardConfig::BRIDGE_COUNT];
  BridgeState bridgeStates_[BoardConfig::BRIDGE_COUNT];
  bool i2cPullupsEnabled_[2] = {false, false};
  bool faultActive_ = false;
  bool otwActive_ = false;
  uint16_t vinAdc_ = 0;
  uint16_t currentAdc_[BoardConfig::BRIDGE_COUNT] = {0, 0};
  uint16_t vinMilliVolts_ = 0;
  int16_t currentMilliAmps_[BoardConfig::BRIDGE_COUNT] = {0, 0};
  uint32_t lastFaultAssertMs_ = 0;
  uint32_t lastFaultPollMs_ = 0;
  uint32_t lastAdcPollMs_ = 0;
};
