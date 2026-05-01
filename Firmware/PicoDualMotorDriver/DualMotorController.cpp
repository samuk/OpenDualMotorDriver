#include "DualMotorController.h"

DualMotorController::DualMotorController()
    : bridgePins_{
          {BoardConfig::PIN_DRIVER_PWM_A, BoardConfig::PIN_DRIVER_PWM_B, BoardConfig::PIN_DRIVER_RESET_AB},
          {BoardConfig::PIN_DRIVER_PWM_C, BoardConfig::PIN_DRIVER_PWM_D, BoardConfig::PIN_DRIVER_RESET_CD},
      } {}

void DualMotorController::begin() {
  configurePins();
  analogReadResolution(12);
  analogWriteFreq(BoardConfig::PWM_FREQUENCY_HZ);
  analogWriteRange(BoardConfig::PWM_RANGE);

  for (uint8_t bridgeIndex = 0; bridgeIndex < BoardConfig::BRIDGE_COUNT; ++bridgeIndex) {
    writeBridgePinsLow(bridgeIndex);
    digitalWrite(bridgePins_[bridgeIndex].resetPin, LOW);
  }

  setI2cPullupEnabled(0, BoardConfig::I2C0_PULLUPS_DEFAULT_ENABLED);
  setI2cPullupEnabled(1, BoardConfig::I2C1_PULLUPS_DEFAULT_ENABLED);

  updateFaultInputs();
  refreshMeasurements();
}

void DualMotorController::service() {
  const uint32_t nowMs = millis();

  if (nowMs - lastFaultPollMs_ >= BoardConfig::FAULT_POLL_INTERVAL_MS) {
    updateFaultInputs();
    lastFaultPollMs_ = nowMs;
  }

  if (nowMs - lastAdcPollMs_ >= BoardConfig::ADC_POLL_INTERVAL_MS) {
    refreshMeasurements();
    lastAdcPollMs_ = nowMs;
  }
}

bool DualMotorController::setBridgeEnabled(int bridgeIndex, bool enable) {
  if (!isValidBridgeIndex(bridgeIndex)) {
    return false;
  }

  updateFaultInputs();
  if (enable && !canClearFaultsNow()) {
    return false;
  }

  BridgeState &bridgeState = bridgeStates_[bridgeIndex];
  const BridgePins &bridgePins = bridgePins_[bridgeIndex];

  if (!enable) {
    bridgeState.enabled = false;
    bridgeState.powerPermille = 0;
    bridgeState.pwmDuty = 0;
    writeBridgePinsLow(bridgeIndex);
    digitalWrite(bridgePins.resetPin, LOW);
    return true;
  }

  bridgeState.enabled = true;
  writeBridgePinsLow(bridgeIndex);
  digitalWrite(bridgePins.resetPin, LOW);
  delay(BoardConfig::DRV8412_RESET_PULSE_MS);
  digitalWrite(bridgePins.resetPin, HIGH);
  applyBridgeOutput(bridgeIndex);
  return true;
}

bool DualMotorController::setMotorPower(int bridgeIndex, int16_t powerPermille) {
  if (!isValidBridgeIndex(bridgeIndex)) {
    return false;
  }

  BridgeState &bridgeState = bridgeStates_[bridgeIndex];
  if (!bridgeState.enabled && powerPermille != 0) {
    return false;
  }

  if (powerPermille > 1000) {
    powerPermille = 1000;
  } else if (powerPermille < -1000) {
    powerPermille = -1000;
  }

  const int16_t absolutePower = powerPermille >= 0 ? powerPermille : static_cast<int16_t>(-powerPermille);

  uint16_t duty = 0;
  if (absolutePower > 0) {
    duty = static_cast<uint16_t>((static_cast<uint32_t>(absolutePower) * BoardConfig::PWM_ACTIVE_MAX + 500U) / 1000U);
    if (duty == 0) {
      duty = 1;
    }
  }

  bridgeState.powerPermille = powerPermille;
  bridgeState.pwmDuty = duty;
  applyBridgeOutput(bridgeIndex);
  return true;
}

void DualMotorController::stopMotor(int bridgeIndex) {
  if (!isValidBridgeIndex(bridgeIndex)) {
    return;
  }

  bridgeStates_[bridgeIndex].powerPermille = 0;
  bridgeStates_[bridgeIndex].pwmDuty = 0;
  applyBridgeOutput(bridgeIndex);
}

void DualMotorController::stopAll() {
  for (uint8_t bridgeIndex = 0; bridgeIndex < BoardConfig::BRIDGE_COUNT; ++bridgeIndex) {
    stopMotor(bridgeIndex);
  }
}

bool DualMotorController::clearFaults(int bridgeIndex) {
  updateFaultInputs();
  if (!canClearFaultsNow()) {
    return false;
  }

  if (bridgeIndex < 0) {
    for (uint8_t index = 0; index < BoardConfig::BRIDGE_COUNT; ++index) {
      clearFaultOnBridge(index);
    }
  } else if (isValidBridgeIndex(bridgeIndex)) {
    clearFaultOnBridge(static_cast<uint8_t>(bridgeIndex));
  } else {
    return false;
  }

  updateFaultInputs();
  return true;
}

bool DualMotorController::setBridgeDirectionInverted(int bridgeIndex, bool inverted) {
  if (!isValidBridgeIndex(bridgeIndex)) {
    return false;
  }

  bridgeStates_[bridgeIndex].directionInverted = inverted;
  applyBridgeOutput(static_cast<uint8_t>(bridgeIndex));
  return true;
}

bool DualMotorController::isBridgeDirectionInverted(int bridgeIndex) const {
  if (!isValidBridgeIndex(bridgeIndex)) {
    return false;
  }

  return bridgeStates_[bridgeIndex].directionInverted;
}

void DualMotorController::setI2cPullupEnabled(int busIndex, bool enable) {
  if (busIndex == 0) {
    digitalWrite(BoardConfig::PIN_I2C0_PULLUP_EN, enable ? HIGH : LOW);
    i2cPullupsEnabled_[0] = enable;
  } else if (busIndex == 1) {
    digitalWrite(BoardConfig::PIN_I2C1_PULLUP_EN, enable ? HIGH : LOW);
    i2cPullupsEnabled_[1] = enable;
  }
}

bool DualMotorController::isI2cPullupEnabled(int busIndex) const {
  if (busIndex == 0) {
    return i2cPullupsEnabled_[0];
  }

  if (busIndex == 1) {
    return i2cPullupsEnabled_[1];
  }

  return false;
}

ControllerSnapshot DualMotorController::readSnapshot() {
  updateFaultInputs();
  refreshMeasurements();

  ControllerSnapshot snapshot;
  snapshot.uptimeMs = millis();
  snapshot.faultActive = faultActive_;
  snapshot.otwActive = otwActive_;
  snapshot.i2cPullupsEnabled[0] = i2cPullupsEnabled_[0];
  snapshot.i2cPullupsEnabled[1] = i2cPullupsEnabled_[1];
  snapshot.vinAdc = vinAdc_;
  snapshot.vinMilliVolts = vinMilliVolts_;

  for (uint8_t bridgeIndex = 0; bridgeIndex < BoardConfig::BRIDGE_COUNT; ++bridgeIndex) {
    snapshot.bridgeEnabled[bridgeIndex] = bridgeStates_[bridgeIndex].enabled;
    snapshot.bridgeDirectionInverted[bridgeIndex] = bridgeStates_[bridgeIndex].directionInverted;
    snapshot.bridgePowerPermille[bridgeIndex] = bridgeStates_[bridgeIndex].powerPermille;
    snapshot.bridgeDuty[bridgeIndex] = bridgeStates_[bridgeIndex].pwmDuty;
    snapshot.currentAdc[bridgeIndex] = currentAdc_[bridgeIndex];
    snapshot.currentMilliAmps[bridgeIndex] = currentMilliAmps_[bridgeIndex];
  }

  return snapshot;
}

uint8_t DualMotorController::getEnabledBridgeCount() const {
  uint8_t enabledCount = 0;
  for (uint8_t bridgeIndex = 0; bridgeIndex < BoardConfig::BRIDGE_COUNT; ++bridgeIndex) {
    if (bridgeStates_[bridgeIndex].enabled) {
      ++enabledCount;
    }
  }
  return enabledCount;
}

bool DualMotorController::isFaultActive() const {
  return faultActive_;
}

bool DualMotorController::isOtwActive() const {
  return otwActive_;
}

uint32_t DualMotorController::getFaultResetHoldoffRemainingMs() const {
  if (!faultActive_) {
    return 0;
  }

  const uint32_t elapsedMs = millis() - lastFaultAssertMs_;
  if (elapsedMs >= BoardConfig::DRV8412_FAULT_RESET_HOLDOFF_MS) {
    return 0;
  }

  return BoardConfig::DRV8412_FAULT_RESET_HOLDOFF_MS - elapsedMs;
}

void DualMotorController::configurePins() {
  pinMode(BoardConfig::PIN_I2C0_PULLUP_EN, OUTPUT);
  pinMode(BoardConfig::PIN_I2C1_PULLUP_EN, OUTPUT);

  pinMode(BoardConfig::PIN_DRIVER_PWM_A, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_PWM_B, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_PWM_C, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_PWM_D, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_RESET_AB, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_RESET_CD, OUTPUT);
  pinMode(BoardConfig::PIN_DRIVER_FAULT, INPUT_PULLUP);
  pinMode(BoardConfig::PIN_DRIVER_OTW, INPUT_PULLUP);

  pinMode(BoardConfig::PIN_ACS722_OUT1, INPUT);
  pinMode(BoardConfig::PIN_ACS722_OUT2, INPUT);
  pinMode(BoardConfig::PIN_VIN_ADC, INPUT);

  pinMode(BoardConfig::PIN_SPI_MISO, INPUT);
  pinMode(BoardConfig::PIN_SPI_CS, INPUT);
  pinMode(BoardConfig::PIN_SPI_SCK, INPUT);
  pinMode(BoardConfig::PIN_SPI_MOSI, INPUT);
}

void DualMotorController::updateFaultInputs() {
  const bool faultNow = digitalRead(BoardConfig::PIN_DRIVER_FAULT) == LOW;
  if (faultNow && !faultActive_) {
    lastFaultAssertMs_ = millis();
  }

  faultActive_ = faultNow;
  otwActive_ = digitalRead(BoardConfig::PIN_DRIVER_OTW) == LOW;
}

void DualMotorController::refreshMeasurements() {
  vinAdc_ = readAdcAverage(BoardConfig::PIN_VIN_ADC);
  currentAdc_[0] = readAdcAverage(BoardConfig::PIN_ACS722_OUT1);
  currentAdc_[1] = readAdcAverage(BoardConfig::PIN_ACS722_OUT2);

  vinMilliVolts_ = adcCountsToVinMilliVolts(vinAdc_);
  currentMilliAmps_[0] = adcCountsToCurrentMilliAmps(currentAdc_[0], 0);
  currentMilliAmps_[1] = adcCountsToCurrentMilliAmps(currentAdc_[1], 1);
}

void DualMotorController::writeBridgePinsLow(uint8_t bridgeIndex) {
  analogWrite(bridgePins_[bridgeIndex].pwmForwardPin, 0);
  analogWrite(bridgePins_[bridgeIndex].pwmReversePin, 0);
}

void DualMotorController::applyBridgeOutput(uint8_t bridgeIndex) {
  const BridgePins &bridgePins = bridgePins_[bridgeIndex];
  const BridgeState &bridgeState = bridgeStates_[bridgeIndex];
  const int16_t appliedPowerPermille =
      bridgeState.directionInverted ? static_cast<int16_t>(-bridgeState.powerPermille) : bridgeState.powerPermille;

  if (!bridgeState.enabled) {
    writeBridgePinsLow(bridgeIndex);
    digitalWrite(bridgePins.resetPin, LOW);
    return;
  }

  digitalWrite(bridgePins.resetPin, HIGH);

  if (appliedPowerPermille > 0) {
    analogWrite(bridgePins.pwmForwardPin, bridgeState.pwmDuty);
    analogWrite(bridgePins.pwmReversePin, 0);
    return;
  }

  if (appliedPowerPermille < 0) {
    analogWrite(bridgePins.pwmForwardPin, 0);
    analogWrite(bridgePins.pwmReversePin, bridgeState.pwmDuty);
    return;
  }

  writeBridgePinsLow(bridgeIndex);
}

uint16_t DualMotorController::readAdcAverage(uint8_t pin) const {
  uint32_t sum = 0;
  for (uint8_t sample = 0; sample < BoardConfig::ADC_SAMPLE_COUNT; ++sample) {
    sum += static_cast<uint16_t>(analogRead(pin));
  }

  return static_cast<uint16_t>((sum + (BoardConfig::ADC_SAMPLE_COUNT / 2U)) / BoardConfig::ADC_SAMPLE_COUNT);
}

uint16_t DualMotorController::adcCountsToMilliVolts(uint16_t counts) const {
  return static_cast<uint16_t>((static_cast<uint32_t>(counts) * BoardConfig::ADC_REFERENCE_MV +
                                (BoardConfig::ADC_MAX_COUNTS / 2U)) /
                               BoardConfig::ADC_MAX_COUNTS);
}

uint16_t DualMotorController::adcCountsToVinMilliVolts(uint16_t counts) const {
  const float sensedMilliVolts = static_cast<float>(adcCountsToMilliVolts(counts));
  const float vinMilliVolts = sensedMilliVolts * BoardConfig::VIN_DIVIDER_SCALE * BoardConfig::VIN_CALIBRATION_SCALE;
  return static_cast<uint16_t>(vinMilliVolts + 0.5f);
}

int16_t DualMotorController::adcCountsToCurrentMilliAmps(uint16_t counts, uint8_t channelIndex) const {
  const int16_t trimMilliVolts = (channelIndex == 0) ? BoardConfig::ACS722_CHANNEL1_ZERO_TRIM_MV
                                                     : BoardConfig::ACS722_CHANNEL2_ZERO_TRIM_MV;
  const int8_t polarity = (channelIndex == 0) ? BoardConfig::ACS722_CHANNEL1_POLARITY
                                              : BoardConfig::ACS722_CHANNEL2_POLARITY;
  const int32_t sensedMilliVolts = adcCountsToMilliVolts(counts);
  const int32_t deltaMilliVolts = sensedMilliVolts - BoardConfig::ACS722_ZERO_CURRENT_MV - trimMilliVolts;
  const int32_t currentMilliAmps = (deltaMilliVolts * 1000L / BoardConfig::ACS722_SENSITIVITY_MV_PER_A) * polarity;
  return static_cast<int16_t>(currentMilliAmps);
}

bool DualMotorController::isValidBridgeIndex(int bridgeIndex) const {
  return bridgeIndex >= 0 && bridgeIndex < static_cast<int>(BoardConfig::BRIDGE_COUNT);
}

bool DualMotorController::canClearFaultsNow() const {
  return getFaultResetHoldoffRemainingMs() == 0;
}

void DualMotorController::clearFaultOnBridge(uint8_t bridgeIndex) {
  const bool restoreEnabled = bridgeStates_[bridgeIndex].enabled;
  writeBridgePinsLow(bridgeIndex);
  digitalWrite(bridgePins_[bridgeIndex].resetPin, LOW);
  delay(BoardConfig::DRV8412_RESET_PULSE_MS);
  digitalWrite(bridgePins_[bridgeIndex].resetPin, HIGH);
  delay(BoardConfig::DRV8412_RESET_PULSE_MS);

  if (restoreEnabled) {
    applyBridgeOutput(bridgeIndex);
  } else {
    digitalWrite(bridgePins_[bridgeIndex].resetPin, LOW);
  }
}
