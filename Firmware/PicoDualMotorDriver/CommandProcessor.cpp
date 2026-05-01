#include "CommandProcessor.h"

#include <Wire.h>
#include <cctype>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

bool isSeparator(char c) {
  return c == ' ' || c == '\t' || c == ',';
}

void extractFirstToken(const char *line, char *token, size_t tokenSize) {
  size_t tokenLength = 0;
  while (*line != '\0' && isSeparator(*line)) {
    ++line;
  }

  while (*line != '\0' && !isSeparator(*line) && tokenLength + 1 < tokenSize) {
    token[tokenLength++] = *line++;
  }

  token[tokenLength] = '\0';
}

bool tokenEquals(const char *left, const char *right) {
  while (*left != '\0' && *right != '\0') {
    const unsigned char leftChar = static_cast<unsigned char>(*left);
    const unsigned char rightChar = static_cast<unsigned char>(*right);
    if (std::toupper(leftChar) != std::toupper(rightChar)) {
      return false;
    }
    ++left;
    ++right;
  }

  return *left == '\0' && *right == '\0';
}

long roundFloatToLong(float value) {
  return static_cast<long>(value >= 0.0f ? (value + 0.5f) : (value - 0.5f));
}

}  // namespace

CommandProcessor *CommandProcessor::instance_ = nullptr;

void CommandProcessor::begin(DualMotorController &controller,
                             StatusLed &statusLed,
                             ClosedLoopController &closedLoop,
                             As5600Encoder &encoder,
                             volatile bool &manualErrorState) {
  controller_ = &controller;
  statusLed_ = &statusLed;
  closedLoop_ = &closedLoop;
  encoder_ = &encoder;
  manualErrorState_ = &manualErrorState;
  fastBinaryMode_ = BoardConfig::FAST_BINARY_MODE_DEFAULT;
  instance_ = this;

  Wire1.setSDA(BoardConfig::PIN_I2C1_SDA);
  Wire1.setSCL(BoardConfig::PIN_I2C1_SCL);
  Wire1.setClock(BoardConfig::I2C1_FREQUENCY_HZ);
  Wire1.setBufferSize(BoardConfig::MAX_I2C_RX_BYTES);
  Wire1.begin(BoardConfig::I2C_SLAVE_ADDRESS);
  Wire1.onReceive(onI2cReceiveThunk);
  Wire1.onRequest(onI2cRequestThunk);

  ResponseBuffer initialResponse;
  buildStatusBinary(initialResponse, static_cast<uint8_t>(kCmdGetStatus | 0x80));
  updateI2cResponse(initialResponse);
}

void CommandProcessor::service() {
  serviceSerialChannel(Serial, Serial, usbState_);
  serviceSerialChannel(Serial1, Serial1, uartState_);
  servicePendingI2cCommand();
  servicePeriodicReports();
}

void CommandProcessor::onI2cReceiveThunk(int count) {
  if (instance_ != nullptr) {
    instance_->onI2cReceive(count);
  }
}

void CommandProcessor::onI2cRequestThunk() {
  if (instance_ != nullptr) {
    instance_->onI2cRequest();
  }
}

void CommandProcessor::onI2cReceive(int count) {
  (void)count;

  size_t length = 0;
  while (Wire1.available() && length < sizeof(i2cRxBuffer_)) {
    const int value = Wire1.read();
    if (value < 0) {
      break;
    }
    i2cRxBuffer_[length++] = static_cast<uint8_t>(value);
  }

  while (Wire1.available()) {
    Wire1.read();
  }

  i2cRxLength_ = length;
  i2cCommandPending_ = true;
}

void CommandProcessor::onI2cRequest() {
  Wire1.write(i2cResponse_, static_cast<size_t>(i2cResponseLength_));
}

void CommandProcessor::serviceSerialChannel(Stream &stream, Print &out, SerialParserState &state) {
  while (stream.available() > 0) {
    const int readValue = stream.read();
    if (readValue < 0) {
      return;
    }

    const uint8_t byteValue = static_cast<uint8_t>(readValue);

    if (state.binaryLength > 0 || byteValue == kBinaryFrameStart) {
      if (state.binaryLength == 0) {
        state.binaryBuffer[0] = byteValue;
        state.binaryLength = 1;
        state.expectedBinaryLength = 0;
        continue;
      }

      if (state.binaryLength < sizeof(state.binaryBuffer)) {
        state.binaryBuffer[state.binaryLength++] = byteValue;
      } else {
        state.binaryLength = 0;
        state.expectedBinaryLength = 0;
        continue;
      }

      if (state.binaryLength == 2) {
        state.expectedBinaryLength = static_cast<size_t>(state.binaryBuffer[1]) + 3U;
        if (state.expectedBinaryLength < 4U || state.expectedBinaryLength > sizeof(state.binaryBuffer)) {
          state.binaryLength = 0;
          state.expectedBinaryLength = 0;
        }
      }

      if (state.expectedBinaryLength > 0 && state.binaryLength == state.expectedBinaryLength) {
        ResponseBuffer response;
        handleBinaryPacket(state.binaryBuffer, state.binaryLength, response);
        dispatchResponse(response, &out);
        state.binaryLength = 0;
        state.expectedBinaryLength = 0;
      }
      continue;
    }

    if (byteValue == '\r') {
      continue;
    }

    if (byteValue == '\n') {
      if (state.droppingLine) {
        ResponseBuffer response;
        setTextResponse(response, "error command too long");
        dispatchResponse(response, &out);
        state.droppingLine = false;
      } else if (state.lineLength > 0) {
        state.lineBuffer[state.lineLength] = '\0';
        ResponseBuffer response;
        handleAsciiCommand(state.lineBuffer, response);
        dispatchResponse(response, &out);
      }
      state.lineLength = 0;
      continue;
    }

    if (state.droppingLine) {
      continue;
    }

    if (state.lineLength + 1 >= sizeof(state.lineBuffer)) {
      state.lineLength = 0;
      state.droppingLine = true;
      continue;
    }

    state.lineBuffer[state.lineLength++] = static_cast<char>(byteValue);
  }
}

void CommandProcessor::servicePendingI2cCommand() {
  if (!i2cCommandPending_) {
    return;
  }

  uint8_t localBuffer[BoardConfig::MAX_I2C_RX_BYTES] = {};
  size_t localLength = 0;

  noInterrupts();
  localLength = i2cRxLength_;
  if (localLength > sizeof(localBuffer)) {
    localLength = sizeof(localBuffer);
  }
  memcpy(localBuffer, i2cRxBuffer_, localLength);
  i2cCommandPending_ = false;
  interrupts();

  if (localLength == 0) {
    return;
  }

  ResponseBuffer response;
  if (localBuffer[0] == kBinaryFrameStart) {
    handleBinaryPacket(localBuffer, localLength, response);
  } else {
    if (localLength >= BoardConfig::MAX_TEXT_COMMAND_LENGTH) {
      localLength = BoardConfig::MAX_TEXT_COMMAND_LENGTH - 1;
    }

    while (localLength > 0 &&
           (localBuffer[localLength - 1] == '\r' || localBuffer[localLength - 1] == '\n')) {
      --localLength;
    }

    char textCommand[BoardConfig::MAX_TEXT_COMMAND_LENGTH] = {};
    memcpy(textCommand, localBuffer, localLength);
    textCommand[localLength] = '\0';
    handleAsciiCommand(textCommand, response);
  }

  updateI2cResponse(response);
}

void CommandProcessor::servicePeriodicReports() {
  const uint32_t nowMs = millis();

  if (textReportIntervalMs_ > 0 && nowMs - lastTextReportMs_ >= textReportIntervalMs_) {
    ResponseBuffer response;
    buildStatusText(response);
    emitResponseToPorts(response, textReportPorts_);
    lastTextReportMs_ = nowMs;
  }

  if (binaryReportIntervalMs_ > 0 && nowMs - lastBinaryReportMs_ >= binaryReportIntervalMs_) {
    ResponseBuffer response;
    buildStatusBinary(response, static_cast<uint8_t>(kCmdGetStatus | 0x80));
    emitResponseToPorts(response, binaryReportPorts_);
    lastBinaryReportMs_ = nowMs;
  }
}

void CommandProcessor::dispatchResponse(const ResponseBuffer &response, Print *out) {
  if (out == nullptr || response.length == 0) {
    return;
  }

  out->write(response.data, response.length);
  if (!response.binary) {
    out->write("\r\n");
  }
}

void CommandProcessor::emitResponseToPorts(const ResponseBuffer &response, uint8_t portMask) {
  if ((portMask & kPortUsb) != 0 && Serial) {
    dispatchResponse(response, &Serial);
  }

  if ((portMask & kPortUart) != 0) {
    dispatchResponse(response, &Serial1);
  }
}

void CommandProcessor::updateI2cResponse(const ResponseBuffer &response) {
  noInterrupts();
  i2cResponseLength_ = response.length;
  if (i2cResponseLength_ > sizeof(i2cResponse_)) {
    i2cResponseLength_ = sizeof(i2cResponse_);
  }
  memcpy(i2cResponse_, response.data, static_cast<size_t>(i2cResponseLength_));
  interrupts();
}

void CommandProcessor::handleAsciiCommand(const char *line, ResponseBuffer &response) {
  char commandToken[16] = {};
  extractFirstToken(line, commandToken, sizeof(commandToken));

  if (commandToken[0] == '\0') {
    setTextResponse(response, "error empty command");
    return;
  }

  if (tokenEquals(commandToken, "M100") || tokenEquals(commandToken, "M115") ||
      tokenEquals(commandToken, "HELP") || tokenEquals(commandToken, "?")) {
    buildHelpText(response);
    return;
  }

  if (tokenEquals(commandToken, "M101") || tokenEquals(commandToken, "M114")) {
    buildStatusText(response);
    return;
  }

  if (tokenEquals(commandToken, "M102") || tokenEquals(commandToken, "M120")) {
    buildStatusBinary(response, static_cast<uint8_t>(kCmdGetStatus | 0x80));
    return;
  }

  if (tokenEquals(commandToken, "M103") || tokenEquals(commandToken, "M154") || tokenEquals(commandToken, "M155")) {
    long reportType = 0;
    long intervalValue = textReportIntervalMs_;
    long portValue = textReportPorts_;

    if (tokenEquals(commandToken, "M155")) {
      reportType = 1;
      intervalValue = binaryReportIntervalMs_;
      portValue = binaryReportPorts_;
    } else if (tokenEquals(commandToken, "M103")) {
      if (!parseLongParam(line, 'T', reportType) || (reportType != 0 && reportType != 1)) {
        setTextResponse(response, "error invalid T parameter");
        return;
      }
      intervalValue = (reportType == 0) ? textReportIntervalMs_ : binaryReportIntervalMs_;
      portValue = (reportType == 0) ? textReportPorts_ : binaryReportPorts_;
    }

    parseLongParam(line, 'S', intervalValue);
    parseLongParam(line, 'P', portValue);

    if (intervalValue < 0) {
      intervalValue = 0;
    }
    if (portValue < 0 || portValue > 3) {
      setTextResponse(response, "error invalid P parameter");
      return;
    }

    if (reportType == 0) {
      textReportIntervalMs_ = static_cast<uint16_t>(intervalValue);
      textReportPorts_ = static_cast<uint8_t>(portValue);
      setTextResponse(response, "ok text report S%u P%u", textReportIntervalMs_, textReportPorts_);
    } else {
      binaryReportIntervalMs_ = static_cast<uint16_t>(intervalValue);
      binaryReportPorts_ = static_cast<uint8_t>(portValue);
      setTextResponse(response, "ok binary report S%u P%u", binaryReportIntervalMs_, binaryReportPorts_);
    }
    return;
  }

  if (tokenEquals(commandToken, "M104") || tokenEquals(commandToken, "M260")) {
    long busIndex = 0;
    if (!parseLongParam(line, 'B', busIndex) || (busIndex != 0 && busIndex != 1)) {
      setTextResponse(response, "error invalid B parameter");
      return;
    }

    long stateValue = controller_->isI2cPullupEnabled(static_cast<int>(busIndex)) ? 1L : 0L;
    if (parseLongParam(line, 'S', stateValue)) {
      controller_->setI2cPullupEnabled(static_cast<int>(busIndex), stateValue != 0);
    }

    setTextResponse(response, "ok I2C%ld pullups=%u", busIndex,
                    controller_->isI2cPullupEnabled(static_cast<int>(busIndex)) ? 1U : 0U);
    return;
  }

  if (tokenEquals(commandToken, "M105") || tokenEquals(commandToken, "M280")) {
    long stateValue = (manualErrorState_ != nullptr && *manualErrorState_) ? 1L : 0L;
    if (parseLongParam(line, 'S', stateValue) && manualErrorState_ != nullptr) {
      *manualErrorState_ = stateValue != 0;
    }

    setTextResponse(response, "ok manual_error=%u",
                    (manualErrorState_ != nullptr && *manualErrorState_) ? 1U : 0U);
    return;
  }

  if (tokenEquals(commandToken, "M106") || tokenEquals(commandToken, "M121")) {
    int bridgeIndex = -1;
    if (!parseBridgeParam(line, bridgeIndex, true)) {
      setTextResponse(response, "error invalid H parameter");
      return;
    }

    if (!controller_->clearFaults(bridgeIndex)) {
      setTextResponse(response, "error fault clear holdoff=%lums",
                      static_cast<unsigned long>(controller_->getFaultResetHoldoffRemainingMs()));
      return;
    }

    if (bridgeIndex < 0) {
      setTextResponse(response, "ok faults cleared on all bridges");
    } else {
      setTextResponse(response, "ok faults cleared on H%d", bridgeIndex);
    }
    return;
  }

  if (tokenEquals(commandToken, "M209")) {
    long stateValue = fastBinaryMode_ ? 1L : 0L;
    if (parseLongParam(line, 'S', stateValue)) {
      fastBinaryMode_ = stateValue != 0;
    }

    setTextResponse(response, "ok fast_binary=%u", fastBinaryMode_ ? 1U : 0U);
    return;
  }

  if (tokenEquals(commandToken, "M210")) {
    long bridgeValue = static_cast<long>(closedLoop_->bridgeIndex());
    if (parseLongParam(line, 'H', bridgeValue)) {
      if (bridgeValue < 0 || bridgeValue >= static_cast<long>(BoardConfig::BRIDGE_COUNT)) {
        setTextResponse(response, "error invalid H parameter");
        return;
      }
      selectClosedLoopBridge(static_cast<uint8_t>(bridgeValue));
    }

    if (encoder_ == nullptr || !encoder_->zeroCurrentPosition()) {
      setTextResponse(response, "error encoder zero failed");
      return;
    }

    closedLoop_->setPositionReferenceCounts(0);
    closedLoop_->reset();
    setTextResponse(response, "ok encoder H%u zeroed zero=%u pos=0", closedLoop_->bridgeIndex(),
                    encoder_->zeroOffsetCounts());
    return;
  }

  if (tokenEquals(commandToken, "M211")) {
    int bridgeIndex = -1;
    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }

    long invertValue = controller_->isBridgeDirectionInverted(bridgeIndex) ? 1L : 0L;
    if (parseLongParam(line, 'S', invertValue)) {
      controller_->setBridgeDirectionInverted(bridgeIndex, invertValue != 0);
    }

    setTextResponse(response, "ok H%d motor_invert=%u", bridgeIndex,
                    controller_->isBridgeDirectionInverted(bridgeIndex) ? 1U : 0U);
    return;
  }

  if (tokenEquals(commandToken, "M200") || tokenEquals(commandToken, "M17") || tokenEquals(commandToken, "M18")) {
    int bridgeIndex = -1;
    if (!parseBridgeParam(line, bridgeIndex, true)) {
      setTextResponse(response, "error invalid H parameter");
      return;
    }

    bool enable = tokenEquals(commandToken, "M17");
    if (tokenEquals(commandToken, "M200")) {
      long stateValue = 0;
      if (!parseLongParam(line, 'S', stateValue)) {
        setTextResponse(response, "error missing S parameter");
        return;
      }
      enable = stateValue != 0;
    } else if (tokenEquals(commandToken, "M18")) {
      enable = false;
    }

    if (bridgeIndex < 0) {
      for (uint8_t index = 0; index < BoardConfig::BRIDGE_COUNT; ++index) {
        if (!controller_->setBridgeEnabled(index, enable)) {
          setTextResponse(response, "error bridge enable blocked, holdoff=%lums",
                          static_cast<unsigned long>(controller_->getFaultResetHoldoffRemainingMs()));
          return;
        }
        syncClosedLoopEnableState(index);
      }
      setTextResponse(response, "ok %s all bridges", enable ? "enabled" : "disabled");
      return;
    }

    if (!controller_->setBridgeEnabled(bridgeIndex, enable)) {
      setTextResponse(response, "error H%d blocked, holdoff=%lums", bridgeIndex,
                      static_cast<unsigned long>(controller_->getFaultResetHoldoffRemainingMs()));
      return;
    }

    syncClosedLoopEnableState(static_cast<uint8_t>(bridgeIndex));
    setTextResponse(response, "ok H%d %s", bridgeIndex, enable ? "enabled" : "disabled");
    return;
  }

  if (tokenEquals(commandToken, "M201") || tokenEquals(commandToken, "M5")) {
    int bridgeIndex = -1;
    if (!parseBridgeParam(line, bridgeIndex, true)) {
      setTextResponse(response, "error invalid H parameter");
      return;
    }

    stopBridgeWithControlState(bridgeIndex);
    if (bridgeIndex < 0) {
      setTextResponse(response, "ok stopped all bridges");
    } else {
      setTextResponse(response, "ok stopped H%d", bridgeIndex);
    }
    return;
  }

  if (tokenEquals(commandToken, "M202")) {
    int bridgeIndex = -1;
    long modeValue = 0;

    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }
    if (!parseLongParam(line, 'C', modeValue) || modeValue < 0 || modeValue > 2) {
      setTextResponse(response, "error invalid C parameter");
      return;
    }

    const ClosedLoopController::Mode mode = static_cast<ClosedLoopController::Mode>(modeValue);
    selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    closedLoop_->setMode(mode);
    syncClosedLoopEnableState(static_cast<uint8_t>(bridgeIndex));
    closedLoop_->reset();

    setTextResponse(response, "ok H%d mode=%s", bridgeIndex, modeName(mode));
    return;
  }

  if (tokenEquals(commandToken, "M203") || tokenEquals(commandToken, "G1") || tokenEquals(commandToken, "M3") ||
      tokenEquals(commandToken, "M4")) {
    int bridgeIndex = -1;
    long speedValue = 0;

    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }

    if (!parseLongParam(line, 'S', speedValue)) {
      setTextResponse(response, "error missing S parameter");
      return;
    }

    int16_t powerPermille = clampPowerPermille(speedValue);
    if (tokenEquals(commandToken, "M3")) {
      if (powerPermille < 0) {
        powerPermille = static_cast<int16_t>(-powerPermille);
      }
    } else if (tokenEquals(commandToken, "M4")) {
      if (powerPermille > 0) {
        powerPermille = static_cast<int16_t>(-powerPermille);
      }
    }

    if (!applyManualDriveToBridge(bridgeIndex, powerPermille)) {
      setTextResponse(response, "error H%d disabled, use M200 H%d S1 first", bridgeIndex, bridgeIndex);
      return;
    }

    setTextResponse(response, "ok H%d manual=%d", bridgeIndex, powerPermille);
    return;
  }

  if (tokenEquals(commandToken, "M204")) {
    int bridgeIndex = -1;
    float referenceValue = 0.0f;
    long unitValue = 0;

    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }
    if (!parseFloatParam(line, 'R', referenceValue)) {
      setTextResponse(response, "error missing R parameter");
      return;
    }
    parseLongParam(line, 'U', unitValue);
    if (unitValue != 0 && unitValue != 1) {
      setTextResponse(response, "error invalid U parameter");
      return;
    }

    selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    if (unitValue == 0) {
      closedLoop_->setSpeedReferenceCountsPerSec(referenceValue);
    } else {
      closedLoop_->setSpeedReferenceRpm(referenceValue);
    }

    const float appliedCountsPerSec = closedLoop_->speedReferenceCountsPerSec();
    const float appliedUnitValue =
        (unitValue == 0)
            ? appliedCountsPerSec
            : (appliedCountsPerSec * 60.0f) / static_cast<float>(BoardConfig::AS5600_COUNTS_PER_REV);
    setTextResponse(response, "ok H%d speed_ref=%ld unit=%s", bridgeIndex, roundFloatToLong(appliedUnitValue * 1000.0f),
                    (unitValue == 0) ? "mcps" : "mrpm");
    return;
  }

  if (tokenEquals(commandToken, "M205")) {
    int bridgeIndex = -1;
    float referenceValue = 0.0f;
    long unitValue = 0;

    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }
    if (!parseFloatParam(line, 'R', referenceValue)) {
      setTextResponse(response, "error missing R parameter");
      return;
    }
    parseLongParam(line, 'U', unitValue);
    if (unitValue < 0 || unitValue > 2) {
      setTextResponse(response, "error invalid U parameter");
      return;
    }

    selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    if (unitValue == 0) {
      closedLoop_->setPositionReferenceCounts(static_cast<int32_t>(roundFloatToLong(referenceValue)));
    } else if (unitValue == 1) {
      closedLoop_->setPositionReferenceDegrees(referenceValue);
    } else {
      closedLoop_->setPositionReferenceTurns(referenceValue);
    }

    setTextResponse(response, "ok H%d position_ref=%ld unit=%s",
                    bridgeIndex,
                    roundFloatToLong(referenceValue * 1000.0f),
                    (unitValue == 0) ? "mcount" : ((unitValue == 1) ? "mdeg" : "mturn"));
    return;
  }

  if (tokenEquals(commandToken, "M206")) {
    int bridgeIndex = -1;
    long loopSelector = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    if (!parseBridgeParam(line, bridgeIndex, false)) {
      setTextResponse(response, "error missing or invalid H parameter");
      return;
    }
    if (!parseLongParam(line, 'L', loopSelector) || (loopSelector != 0 && loopSelector != 1)) {
      setTextResponse(response, "error invalid L parameter");
      return;
    }
    if (!parseFloatParam(line, 'P', kp) || !parseFloatParam(line, 'I', ki) || !parseFloatParam(line, 'D', kd)) {
      setTextResponse(response, "error missing P/I/D parameter");
      return;
    }

    selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    if (loopSelector == 0) {
      closedLoop_->setSpeedPidGains(kp, ki, kd);
    } else {
      closedLoop_->setPositionPidGains(kp, ki, kd);
    }
    closedLoop_->reset();

    setTextResponse(response, "ok H%d %s_pid=%ld/%ld/%ld",
                    bridgeIndex,
                    (loopSelector == 0) ? "speed" : "position",
                    roundFloatToLong(kp * 1000.0f),
                    roundFloatToLong(ki * 1000.0f),
                    roundFloatToLong(kd * 1000.0f));
    return;
  }

  if (tokenEquals(commandToken, "M207")) {
    int bridgeIndex = static_cast<int>(closedLoop_->bridgeIndex());
    parseBridgeParam(line, bridgeIndex, false);
    selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    closedLoop_->reset();
    syncClosedLoopEnableState(static_cast<uint8_t>(bridgeIndex));
    setTextResponse(response, "ok loop reset H%d", bridgeIndex);
    return;
  }

  if (tokenEquals(commandToken, "M208")) {
    int bridgeIndex = static_cast<int>(closedLoop_->bridgeIndex());
    long bitsValue = encoder_->resolutionBits();
    long zeroOffset = encoder_->zeroOffsetCounts();
    long invertValue = encoder_->inverted() ? 1L : 0L;

    parseBridgeParam(line, bridgeIndex, false);
    if (bridgeIndex >= 0 && bridgeIndex < static_cast<int>(BoardConfig::BRIDGE_COUNT)) {
      selectClosedLoopBridge(static_cast<uint8_t>(bridgeIndex));
    }

    parseLongParam(line, 'B', bitsValue);
    parseLongParam(line, 'Z', zeroOffset);
    parseLongParam(line, 'V', invertValue);

    if (bitsValue < BoardConfig::AS5600_MIN_REDUCED_BITS || bitsValue > BoardConfig::AS5600_MAX_REDUCED_BITS) {
      setTextResponse(response, "error invalid B parameter");
      return;
    }
    if (zeroOffset < 0 || zeroOffset >= BoardConfig::AS5600_COUNTS_PER_REV) {
      setTextResponse(response, "error invalid Z parameter");
      return;
    }

    encoder_->setResolutionBits(static_cast<uint8_t>(bitsValue));
    encoder_->setZeroOffsetCounts(static_cast<uint16_t>(zeroOffset));
    encoder_->setInverted(invertValue != 0);
    closedLoop_->reset();

    setTextResponse(response, "ok encoder H%d bits=%u zero=%u invert=%u", closedLoop_->bridgeIndex(),
                    encoder_->resolutionBits(), encoder_->zeroOffsetCounts(), encoder_->inverted() ? 1U : 0U);
    return;
  }

  setTextResponse(response, "error unknown command");
}

void CommandProcessor::handleBinaryPacket(const uint8_t *data, size_t length, ResponseBuffer &response) {
  if (length < 4 || data[0] != kBinaryFrameStart) {
    buildBinaryError(response, 0x00, kErrBadLength);
    return;
  }

  const uint8_t payloadLengthField = data[1];
  const uint8_t requestCommand = (length > 2) ? data[2] : 0x00;

  if (payloadLengthField < 1 || length != static_cast<size_t>(payloadLengthField) + 3U) {
    buildBinaryError(response, requestCommand, kErrBadLength);
    return;
  }

  uint8_t checksum = 0;
  for (size_t index = 1; index < length - 1; ++index) {
    checksum ^= data[index];
  }

  if (checksum != data[length - 1]) {
    buildBinaryError(response, requestCommand, kErrBadChecksum);
    return;
  }

  const uint8_t *payload = &data[3];
  const size_t payloadLength = payloadLengthField - 1U;

  switch (requestCommand) {
    case kCmdGetStatus:
      if (payloadLength != 0) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }
      buildStatusBinary(response, static_cast<uint8_t>(requestCommand | 0x80));
      return;

    case kCmdSetFastMode:
      if (payloadLength != 1) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }
      fastBinaryMode_ = payload[0] != 0;
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetReport: {
      if (payloadLength != 4) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }

      const uint8_t reportType = payload[0];
      const uint8_t portMask = payload[1] & 0x03;
      const uint16_t intervalMs = static_cast<uint16_t>(payload[2]) |
                                  (static_cast<uint16_t>(payload[3]) << 8U);

      if (reportType == 0) {
        textReportIntervalMs_ = intervalMs;
        textReportPorts_ = portMask;
      } else if (reportType == 1) {
        binaryReportIntervalMs_ = intervalMs;
        binaryReportPorts_ = portMask;
      } else {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }

      buildBinaryAckOrStatus(response, requestCommand);
      return;
    }

    case kCmdSetPullups:
      if (payloadLength != 2 || payload[0] > 1) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      controller_->setI2cPullupEnabled(payload[0], payload[1] != 0);
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetManualError:
      if (payloadLength != 1 || manualErrorState_ == nullptr) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      *manualErrorState_ = payload[0] != 0;
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdClearFaults: {
      int bridgeIndex = -1;
      if (payloadLength == 1) {
        if (payload[0] == 0xFF) {
          bridgeIndex = -1;
        } else if (payload[0] < BoardConfig::BRIDGE_COUNT) {
          bridgeIndex = payload[0];
        } else {
          buildBinaryError(response, requestCommand, kErrInvalidArgument);
          return;
        }
      } else if (payloadLength != 0) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }

      if (!controller_->clearFaults(bridgeIndex)) {
        buildBinaryError(response, requestCommand, kErrBusy);
        return;
      }

      buildBinaryAckOrStatus(response, requestCommand);
      return;
    }

    case kCmdSetBridgeEnable: {
      if (payloadLength != 2) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }

      const uint8_t bridge = payload[0];
      const bool enable = payload[1] != 0;

      if (bridge == 0xFF) {
        for (uint8_t index = 0; index < BoardConfig::BRIDGE_COUNT; ++index) {
          if (!controller_->setBridgeEnabled(index, enable)) {
            buildBinaryError(response, requestCommand, kErrBusy);
            return;
          }
          syncClosedLoopEnableState(index);
        }
      } else if (bridge < BoardConfig::BRIDGE_COUNT) {
        if (!controller_->setBridgeEnabled(bridge, enable)) {
          buildBinaryError(response, requestCommand, kErrBusy);
          return;
        }
        syncClosedLoopEnableState(bridge);
      } else {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }

      buildBinaryAckOrStatus(response, requestCommand);
      return;
    }

    case kCmdStop:
      if (payloadLength > 1) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }
      if (payloadLength == 0 || payload[0] == 0xFF) {
        stopBridgeWithControlState(-1);
      } else if (payload[0] < BoardConfig::BRIDGE_COUNT) {
        stopBridgeWithControlState(payload[0]);
      } else {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetMode:
      if (payloadLength != 2 || payload[0] >= BoardConfig::BRIDGE_COUNT || payload[1] > 2) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      selectClosedLoopBridge(payload[0]);
      closedLoop_->setMode(static_cast<ClosedLoopController::Mode>(payload[1]));
      syncClosedLoopEnableState(payload[0]);
      closedLoop_->reset();
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetManualDrive:
      if (payloadLength != 3 || payload[0] >= BoardConfig::BRIDGE_COUNT) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      if (!applyManualDriveToBridge(payload[0], clampPowerPermille(readInt16LE(&payload[1])))) {
        buildBinaryError(response, requestCommand, kErrBridgeDisabled);
        return;
      }
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetSpeedReference:
      if (payloadLength != 5 || payload[0] >= BoardConfig::BRIDGE_COUNT) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      selectClosedLoopBridge(payload[0]);
      closedLoop_->setSpeedReferenceCountsPerSec(static_cast<float>(readInt32LE(&payload[1])));
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetPositionReference:
      if (payloadLength != 5 || payload[0] >= BoardConfig::BRIDGE_COUNT) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      selectClosedLoopBridge(payload[0]);
      closedLoop_->setPositionReferenceCounts(readInt32LE(&payload[1]));
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetPidGains:
      if (payloadLength != 14 || payload[0] >= BoardConfig::BRIDGE_COUNT || payload[1] > 1) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      selectClosedLoopBridge(payload[0]);
      if (payload[1] == 0) {
        closedLoop_->setSpeedPidGains(readFloatLE(&payload[2]), readFloatLE(&payload[6]), readFloatLE(&payload[10]));
      } else {
        closedLoop_->setPositionPidGains(readFloatLE(&payload[2]), readFloatLE(&payload[6]),
                                         readFloatLE(&payload[10]));
      }
      closedLoop_->reset();
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdResetLoop:
      if (payloadLength > 1) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }
      if (payloadLength == 1) {
        if (payload[0] >= BoardConfig::BRIDGE_COUNT) {
          buildBinaryError(response, requestCommand, kErrInvalidArgument);
          return;
        }
        selectClosedLoopBridge(payload[0]);
      }
      closedLoop_->reset();
      syncClosedLoopEnableState(closedLoop_->bridgeIndex());
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdSetEncoderConfig: {
      if (payloadLength != 4) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }

      const uint8_t bitsValue = payload[0];
      const uint16_t zeroOffset = static_cast<uint16_t>(payload[1]) | (static_cast<uint16_t>(payload[2]) << 8U);
      const uint8_t invert = payload[3];

      if (bitsValue != 0xFF) {
        if (bitsValue < BoardConfig::AS5600_MIN_REDUCED_BITS || bitsValue > BoardConfig::AS5600_MAX_REDUCED_BITS) {
          buildBinaryError(response, requestCommand, kErrInvalidArgument);
          return;
        }
        encoder_->setResolutionBits(bitsValue);
      }

      if (zeroOffset != 0xFFFFU) {
        encoder_->setZeroOffsetCounts(static_cast<uint16_t>(zeroOffset % BoardConfig::AS5600_COUNTS_PER_REV));
      }

      if (invert != 0xFFU) {
        encoder_->setInverted(invert != 0);
      }

      closedLoop_->reset();
      buildBinaryAckOrStatus(response, requestCommand);
      return;
    }

    case kCmdSetBridgeDirection:
      if (payloadLength != 2 || payload[0] >= BoardConfig::BRIDGE_COUNT || payload[1] > 1) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      controller_->setBridgeDirectionInverted(payload[0], payload[1] != 0);
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdFastSetpoint:
      if (payloadLength != 7 || payload[0] >= BoardConfig::BRIDGE_COUNT || payload[1] > 2 || payload[2] > 2) {
        buildBinaryError(response, requestCommand, kErrInvalidArgument);
        return;
      }
      selectClosedLoopBridge(payload[0]);
      if (payload[2] == 1) {
        if (!controller_->setBridgeEnabled(payload[0], false)) {
          buildBinaryError(response, requestCommand, kErrBusy);
          return;
        }
      } else if (payload[2] == 2) {
        if (!controller_->setBridgeEnabled(payload[0], true)) {
          buildBinaryError(response, requestCommand, kErrBusy);
          return;
        }
      }
      syncClosedLoopEnableState(payload[0]);
      if (payload[1] == static_cast<uint8_t>(ClosedLoopController::Mode::Manual)) {
        if (!applyManualDriveToBridge(payload[0], clampPowerPermille(readInt32LE(&payload[3])))) {
          buildBinaryError(response, requestCommand, kErrBridgeDisabled);
          return;
        }
      } else if (payload[1] == static_cast<uint8_t>(ClosedLoopController::Mode::SpeedPid)) {
        closedLoop_->setMode(ClosedLoopController::Mode::SpeedPid);
        closedLoop_->setSpeedReferenceCountsPerSec(static_cast<float>(readInt32LE(&payload[3])));
        syncClosedLoopEnableState(payload[0]);
      } else {
        closedLoop_->setMode(ClosedLoopController::Mode::PositionPid);
        closedLoop_->setPositionReferenceCounts(readInt32LE(&payload[3]));
        syncClosedLoopEnableState(payload[0]);
      }
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    case kCmdFastManualDual:
      if (payloadLength != 4) {
        buildBinaryError(response, requestCommand, kErrBadLength);
        return;
      }
      if (!applyManualDriveToBridge(0, clampPowerPermille(readInt16LE(&payload[0]))) ||
          !applyManualDriveToBridge(1, clampPowerPermille(readInt16LE(&payload[2])))) {
        buildBinaryError(response, requestCommand, kErrBridgeDisabled);
        return;
      }
      buildBinaryAckOrStatus(response, requestCommand);
      return;

    default:
      buildBinaryError(response, requestCommand, kErrUnknownCommand);
      return;
  }
}

void CommandProcessor::buildHelpText(ResponseBuffer &response) {
  setTextResponse(response,
                  "ok M100 help, M101 text, M102 binary, M103 T0|1 Sms P1..3, M104 B0|1 S0|1, "
                  "M105 S0|1, M106 [H0|H1], M200 [H0|H1] S0|1, M201 [H0|H1], M202 Hn C0|1|2, "
                  "M203 Hn S-1000..1000, M204 Hn Rval U0:cps U1:rpm, M205 Hn Rval U0:cnt U1:deg U2:turn, "
                  "M206 Hn L0:speed|1:pos P I D, M207 [Hn], M208 [Hn] [B10..12] [Z0..4095] [V0|1], "
                  "M209 S0|1, M210 [Hn], M211 Hn [S0|1]");
}

void CommandProcessor::setTextResponse(ResponseBuffer &response, const char *fmt, ...) {
  response.binary = false;

  va_list args;
  va_start(args, fmt);
  const int written = vsnprintf(reinterpret_cast<char *>(response.data), sizeof(response.data), fmt, args);
  va_end(args);

  if (written < 0) {
    response.length = 0;
    return;
  }

  response.length = static_cast<size_t>(written);
  if (response.length >= sizeof(response.data)) {
    response.length = sizeof(response.data) - 1;
  }
}

void CommandProcessor::beginBinaryResponse(ResponseBuffer &response, uint8_t responseCommand) {
  response.binary = true;
  response.length = 0;
  response.data[response.length++] = kBinaryFrameStart;
  response.data[response.length++] = 1;
  response.data[response.length++] = responseCommand;
}

bool CommandProcessor::appendBinaryByte(ResponseBuffer &response, uint8_t value) {
  if (response.length + 1 >= sizeof(response.data)) {
    return false;
  }

  response.data[response.length++] = value;
  ++response.data[1];
  return true;
}

bool CommandProcessor::appendBinaryUInt16(ResponseBuffer &response, uint16_t value) {
  return appendBinaryByte(response, static_cast<uint8_t>(value & 0xFFU)) &&
         appendBinaryByte(response, static_cast<uint8_t>((value >> 8U) & 0xFFU));
}

bool CommandProcessor::appendBinaryInt16(ResponseBuffer &response, int16_t value) {
  return appendBinaryUInt16(response, static_cast<uint16_t>(value));
}

bool CommandProcessor::appendBinaryUInt32(ResponseBuffer &response, uint32_t value) {
  return appendBinaryByte(response, static_cast<uint8_t>(value & 0xFFUL)) &&
         appendBinaryByte(response, static_cast<uint8_t>((value >> 8U) & 0xFFUL)) &&
         appendBinaryByte(response, static_cast<uint8_t>((value >> 16U) & 0xFFUL)) &&
         appendBinaryByte(response, static_cast<uint8_t>((value >> 24U) & 0xFFUL));
}

bool CommandProcessor::appendBinaryInt32(ResponseBuffer &response, int32_t value) {
  return appendBinaryUInt32(response, static_cast<uint32_t>(value));
}

bool CommandProcessor::appendBinaryFloat(ResponseBuffer &response, float value) {
  uint32_t rawValue = 0;
  memcpy(&rawValue, &value, sizeof(rawValue));
  return appendBinaryUInt32(response, rawValue);
}

void CommandProcessor::finalizeBinaryResponse(ResponseBuffer &response) {
  uint8_t checksum = 0;
  for (size_t index = 1; index < response.length; ++index) {
    checksum ^= response.data[index];
  }

  if (response.length < sizeof(response.data)) {
    response.data[response.length++] = checksum;
  }
}

void CommandProcessor::buildBinaryAck(ResponseBuffer &response, uint8_t responseCommand, uint8_t status) {
  beginBinaryResponse(response, responseCommand);
  appendBinaryByte(response, status);
  finalizeBinaryResponse(response);
}

void CommandProcessor::buildBinaryAckOrStatus(ResponseBuffer &response, uint8_t requestCommand) {
  if (fastBinaryMode_) {
    buildBinaryAck(response, static_cast<uint8_t>(requestCommand | 0x80));
  } else {
    buildStatusBinary(response, static_cast<uint8_t>(requestCommand | 0x80));
  }
}

void CommandProcessor::buildBinaryError(ResponseBuffer &response, uint8_t requestCommand, uint8_t errorCode) {
  beginBinaryResponse(response, 0x7F);
  appendBinaryByte(response, requestCommand);
  appendBinaryByte(response, errorCode);
  finalizeBinaryResponse(response);
}

void CommandProcessor::buildStatusBinary(ResponseBuffer &response, uint8_t responseCommand) {
  const ControllerSnapshot snapshot = controller_->readSnapshot();
  const ClosedLoopController::Snapshot loopSnapshot = closedLoop_->snapshot();
  const bool ledReady = statusLed_ != nullptr && statusLed_->isReady();
  const bool errorActive = isErrorActive(snapshot, loopSnapshot);

  beginBinaryResponse(response, responseCommand);
  appendBinaryByte(response, buildStatusFlags(snapshot, errorActive, ledReady));
  appendBinaryUInt32(response, snapshot.uptimeMs);
  appendBinaryInt16(response, snapshot.bridgePowerPermille[0]);
  appendBinaryInt16(response, snapshot.bridgePowerPermille[1]);
  appendBinaryUInt16(response, snapshot.bridgeDuty[0]);
  appendBinaryUInt16(response, snapshot.bridgeDuty[1]);
  appendBinaryUInt16(response, snapshot.vinMilliVolts);
  appendBinaryInt16(response, snapshot.currentMilliAmps[0]);
  appendBinaryInt16(response, snapshot.currentMilliAmps[1]);
  appendBinaryUInt16(response,
                     static_cast<uint16_t>(controller_->getFaultResetHoldoffRemainingMs() > 0xFFFFU
                                               ? 0xFFFFU
                                               : controller_->getFaultResetHoldoffRemainingMs()));
  appendBinaryByte(response, loopSnapshot.bridgeIndex);
  appendBinaryByte(response, static_cast<uint8_t>(loopSnapshot.mode));
  appendBinaryByte(response, buildLoopFlags(loopSnapshot));
  appendBinaryByte(response, encoder_ != nullptr ? encoder_->resolutionBits() : 0U);
  appendBinaryUInt16(response, loopSnapshot.encoderNativeCounts);
  appendBinaryUInt16(response, loopSnapshot.encoderReducedCounts);
  appendBinaryInt32(response, loopSnapshot.measuredPositionCounts);
  appendBinaryInt16(response, static_cast<int16_t>(roundFloatToLong(loopSnapshot.measuredSpeedCountsPerSec)));
  appendBinaryInt16(response, loopSnapshot.manualCommandPermille);
  appendBinaryInt16(response, loopSnapshot.outputCommandPermille);
  appendBinaryInt32(response, loopSnapshot.referencePositionCounts);
  appendBinaryInt16(response, static_cast<int16_t>(roundFloatToLong(loopSnapshot.referenceSpeedCountsPerSec)));
  appendBinaryByte(response,
                   static_cast<uint8_t>((snapshot.bridgeDirectionInverted[0] ? 0x01U : 0x00U) |
                                        (snapshot.bridgeDirectionInverted[1] ? 0x02U : 0x00U)));
  finalizeBinaryResponse(response);
}

void CommandProcessor::buildStatusText(ResponseBuffer &response) {
  const ControllerSnapshot snapshot = controller_->readSnapshot();
  const ClosedLoopController::Snapshot loopSnapshot = closedLoop_->snapshot();
  const bool ledReady = statusLed_ != nullptr && statusLed_->isReady();
  const bool errorActive = isErrorActive(snapshot, loopSnapshot);

  const uint16_t vinWhole = snapshot.vinMilliVolts / 1000U;
  const uint16_t vinFrac = snapshot.vinMilliVolts % 1000U;

  const int32_t current1Abs = snapshot.currentMilliAmps[0] >= 0 ? snapshot.currentMilliAmps[0]
                                                                 : -snapshot.currentMilliAmps[0];
  const int32_t current2Abs = snapshot.currentMilliAmps[1] >= 0 ? snapshot.currentMilliAmps[1]
                                                                 : -snapshot.currentMilliAmps[1];

  const long speedKp = roundFloatToLong(loopSnapshot.speedGains.kp * 1000.0f);
  const long speedKi = roundFloatToLong(loopSnapshot.speedGains.ki * 1000.0f);
  const long speedKd = roundFloatToLong(loopSnapshot.speedGains.kd * 1000.0f);
  const long positionKp = roundFloatToLong(loopSnapshot.positionGains.kp * 1000.0f);
  const long positionKi = roundFloatToLong(loopSnapshot.positionGains.ki * 1000.0f);
  const long positionKd = roundFloatToLong(loopSnapshot.positionGains.kd * 1000.0f);

  setTextResponse(response,
                  "ok up=%lums err=%u fault=%u otw=%u holdoff=%lums led=%u fast=%u en=[%u,%u] "
                  "pwr=[%d,%d] duty=[%u,%u] dir=[%u,%u] vin=%u.%03uV i1=%s%ld.%03ldA i2=%s%ld.%03ldA "
                  "pullups=[%u,%u] loop={H%u mode=%s en=%u enc=%u raw=%u red=%u pos=%ld spd=%ld "
                  "refS=%ld refP=%ld out=%d man=%d pidS=%ld/%ld/%ld pidP=%ld/%ld/%ld}",
                  static_cast<unsigned long>(snapshot.uptimeMs),
                  errorActive ? 1U : 0U,
                  snapshot.faultActive ? 1U : 0U,
                  snapshot.otwActive ? 1U : 0U,
                  static_cast<unsigned long>(controller_->getFaultResetHoldoffRemainingMs()),
                  ledReady ? 1U : 0U,
                  fastBinaryMode_ ? 1U : 0U,
                  snapshot.bridgeEnabled[0] ? 1U : 0U,
                  snapshot.bridgeEnabled[1] ? 1U : 0U,
                  snapshot.bridgePowerPermille[0],
                  snapshot.bridgePowerPermille[1],
                  snapshot.bridgeDuty[0],
                  snapshot.bridgeDuty[1],
                  snapshot.bridgeDirectionInverted[0] ? 1U : 0U,
                  snapshot.bridgeDirectionInverted[1] ? 1U : 0U,
                  vinWhole,
                  vinFrac,
                  snapshot.currentMilliAmps[0] < 0 ? "-" : "",
                  static_cast<long>(current1Abs / 1000L),
                  static_cast<long>(current1Abs % 1000L),
                  snapshot.currentMilliAmps[1] < 0 ? "-" : "",
                  static_cast<long>(current2Abs / 1000L),
                  static_cast<long>(current2Abs % 1000L),
                  snapshot.i2cPullupsEnabled[0] ? 1U : 0U,
                  snapshot.i2cPullupsEnabled[1] ? 1U : 0U,
                  loopSnapshot.bridgeIndex,
                  modeName(loopSnapshot.mode),
                  loopSnapshot.enabled ? 1U : 0U,
                  loopSnapshot.encoderHealthy ? 1U : 0U,
                  loopSnapshot.encoderNativeCounts,
                  loopSnapshot.encoderReducedCounts,
                  static_cast<long>(loopSnapshot.measuredPositionCounts),
                  roundFloatToLong(loopSnapshot.measuredSpeedCountsPerSec),
                  roundFloatToLong(loopSnapshot.referenceSpeedCountsPerSec),
                  static_cast<long>(loopSnapshot.referencePositionCounts),
                  loopSnapshot.outputCommandPermille,
                  loopSnapshot.manualCommandPermille,
                  speedKp,
                  speedKi,
                  speedKd,
                  positionKp,
                  positionKi,
                  positionKd);
}

bool CommandProcessor::parseLongParam(const char *line, char prefix, long &value) const {
  const unsigned char wantedPrefix = static_cast<unsigned char>(std::toupper(static_cast<unsigned char>(prefix)));
  const char *cursor = line;

  while (*cursor != '\0') {
    while (*cursor != '\0' && isSeparator(*cursor)) {
      ++cursor;
    }

    if (*cursor == '\0') {
      return false;
    }

    if (std::toupper(static_cast<unsigned char>(*cursor)) == wantedPrefix) {
      char *endPtr = nullptr;
      const long parsedValue = strtol(cursor + 1, &endPtr, 10);
      if (endPtr != cursor + 1) {
        value = parsedValue;
        return true;
      }
    }

    while (*cursor != '\0' && !isSeparator(*cursor)) {
      ++cursor;
    }
  }

  return false;
}

bool CommandProcessor::parseFloatParam(const char *line, char prefix, float &value) const {
  const unsigned char wantedPrefix = static_cast<unsigned char>(std::toupper(static_cast<unsigned char>(prefix)));
  const char *cursor = line;

  while (*cursor != '\0') {
    while (*cursor != '\0' && isSeparator(*cursor)) {
      ++cursor;
    }

    if (*cursor == '\0') {
      return false;
    }

    if (std::toupper(static_cast<unsigned char>(*cursor)) == wantedPrefix) {
      char *endPtr = nullptr;
      const float parsedValue = strtof(cursor + 1, &endPtr);
      if (endPtr != cursor + 1) {
        value = parsedValue;
        return true;
      }
    }

    while (*cursor != '\0' && !isSeparator(*cursor)) {
      ++cursor;
    }
  }

  return false;
}

bool CommandProcessor::parseBridgeParam(const char *line, int &bridgeIndex, bool allowAll) const {
  long value = 0;
  if (!parseLongParam(line, 'H', value)) {
    if (allowAll) {
      bridgeIndex = -1;
      return true;
    }
    return false;
  }

  if (allowAll && (value == -1 || value == 255)) {
    bridgeIndex = -1;
    return true;
  }

  if (value < 0 || value >= static_cast<long>(BoardConfig::BRIDGE_COUNT)) {
    return false;
  }

  bridgeIndex = static_cast<int>(value);
  return true;
}

int16_t CommandProcessor::clampPowerPermille(long value) const {
  if (value > 1000) {
    return 1000;
  }
  if (value < -1000) {
    return -1000;
  }
  return static_cast<int16_t>(value);
}

bool CommandProcessor::isErrorActive(const ControllerSnapshot &snapshot,
                                     const ClosedLoopController::Snapshot &loopSnapshot) const {
  const bool encoderLoopError = loopSnapshot.enabled &&
                                loopSnapshot.mode != ClosedLoopController::Mode::Manual &&
                                !loopSnapshot.encoderHealthy;
  return (manualErrorState_ != nullptr && *manualErrorState_) || snapshot.faultActive ||
         (statusLed_ != nullptr && !statusLed_->isReady()) || encoderLoopError;
}

uint8_t CommandProcessor::buildStatusFlags(const ControllerSnapshot &snapshot, bool errorActive, bool ledReady) const {
  uint8_t flags = 0;

  if (errorActive) {
    flags |= 0x01;
  }
  if (snapshot.faultActive) {
    flags |= 0x02;
  }
  if (snapshot.otwActive) {
    flags |= 0x04;
  }
  if (ledReady) {
    flags |= 0x08;
  }
  if (snapshot.bridgeEnabled[0]) {
    flags |= 0x10;
  }
  if (snapshot.bridgeEnabled[1]) {
    flags |= 0x20;
  }
  if (snapshot.i2cPullupsEnabled[0]) {
    flags |= 0x40;
  }
  if (snapshot.i2cPullupsEnabled[1]) {
    flags |= 0x80;
  }

  return flags;
}

uint8_t CommandProcessor::buildLoopFlags(const ClosedLoopController::Snapshot &loopSnapshot) const {
  uint8_t flags = 0;
  if (loopSnapshot.enabled) {
    flags |= 0x01;
  }
  if (loopSnapshot.encoderHealthy) {
    flags |= 0x02;
  }
  if (fastBinaryMode_) {
    flags |= 0x04;
  }
  if (loopSnapshot.enabled && loopSnapshot.mode != ClosedLoopController::Mode::Manual &&
      !loopSnapshot.encoderHealthy) {
    flags |= 0x08;
  }
  return flags;
}

const char *CommandProcessor::modeName(ClosedLoopController::Mode mode) const {
  switch (mode) {
    case ClosedLoopController::Mode::Manual:
      return "manual";
    case ClosedLoopController::Mode::SpeedPid:
      return "speed";
    case ClosedLoopController::Mode::PositionPid:
      return "position";
    default:
      return "unknown";
  }
}

bool CommandProcessor::isClosedLoopBridge(int bridgeIndex) const {
  return closedLoop_ != nullptr && bridgeIndex >= 0 && bridgeIndex < static_cast<int>(BoardConfig::BRIDGE_COUNT) &&
         closedLoop_->bridgeIndex() == static_cast<uint8_t>(bridgeIndex);
}

void CommandProcessor::selectClosedLoopBridge(uint8_t bridgeIndex) {
  if (closedLoop_ == nullptr || bridgeIndex >= BoardConfig::BRIDGE_COUNT) {
    return;
  }

  if (closedLoop_->bridgeIndex() == bridgeIndex) {
    return;
  }

  const uint8_t previousBridge = closedLoop_->bridgeIndex();
  if (previousBridge < BoardConfig::BRIDGE_COUNT) {
    controller_->stopMotor(previousBridge);
  }

  closedLoop_->setEnabled(false);
  closedLoop_->setMode(ClosedLoopController::Mode::Manual);
  closedLoop_->setManualCommand(0);
  closedLoop_->setSpeedReferenceCountsPerSec(0.0f);
  closedLoop_->setPositionReferenceCounts(0);
  closedLoop_->setBridgeIndex(bridgeIndex);
  closedLoop_->reset();
  syncClosedLoopEnableState(bridgeIndex);
}

void CommandProcessor::syncClosedLoopEnableState(uint8_t bridgeIndex) {
  if (closedLoop_ == nullptr || bridgeIndex >= BoardConfig::BRIDGE_COUNT || closedLoop_->bridgeIndex() != bridgeIndex) {
    return;
  }

  const ControllerSnapshot snapshot = controller_->readSnapshot();
  const bool previousEnabled = closedLoop_->enabled();
  closedLoop_->setEnabled(snapshot.bridgeEnabled[bridgeIndex]);
  if (closedLoop_->enabled() != previousEnabled) {
    closedLoop_->reset();
  }
}

void CommandProcessor::disableClosedLoop() {
  if (closedLoop_ == nullptr) {
    return;
  }

  const ClosedLoopController::Snapshot loopSnapshot = closedLoop_->snapshot();
  closedLoop_->setMode(ClosedLoopController::Mode::Manual);
  closedLoop_->setManualCommand(0);
  closedLoop_->setSpeedReferenceCountsPerSec(0.0f);
  closedLoop_->setPositionReferenceCounts(loopSnapshot.measuredPositionCounts);
  closedLoop_->setEnabled(false);
  closedLoop_->reset();
}

void CommandProcessor::stopBridgeWithControlState(int bridgeIndex) {
  if (bridgeIndex < 0) {
    controller_->stopAll();
    disableClosedLoop();
    return;
  }

  controller_->stopMotor(bridgeIndex);
  if (isClosedLoopBridge(bridgeIndex)) {
    disableClosedLoop();
  }
}

bool CommandProcessor::applyManualDriveToBridge(int bridgeIndex, int16_t powerPermille) {
  if (bridgeIndex < 0 || bridgeIndex >= static_cast<int>(BoardConfig::BRIDGE_COUNT)) {
    return false;
  }

  if (isClosedLoopBridge(bridgeIndex)) {
    closedLoop_->setMode(ClosedLoopController::Mode::Manual);
    closedLoop_->setManualCommand(powerPermille);
    syncClosedLoopEnableState(static_cast<uint8_t>(bridgeIndex));
    closedLoop_->reset();
  }

  return controller_->setMotorPower(bridgeIndex, powerPermille);
}

int16_t CommandProcessor::readInt16LE(const uint8_t *data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8U));
}

int32_t CommandProcessor::readInt32LE(const uint8_t *data) {
  return static_cast<int32_t>(static_cast<uint32_t>(data[0]) |
                              (static_cast<uint32_t>(data[1]) << 8U) |
                              (static_cast<uint32_t>(data[2]) << 16U) |
                              (static_cast<uint32_t>(data[3]) << 24U));
}

float CommandProcessor::readFloatLE(const uint8_t *data) {
  uint32_t rawValue = static_cast<uint32_t>(data[0]) |
                      (static_cast<uint32_t>(data[1]) << 8U) |
                      (static_cast<uint32_t>(data[2]) << 16U) |
                      (static_cast<uint32_t>(data[3]) << 24U);
  float value = 0.0f;
  memcpy(&value, &rawValue, sizeof(value));
  return value;
}
