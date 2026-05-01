#pragma once

#include <Arduino.h>

#include "As5600Encoder.h"
#include "BoardConfig.h"
#include "ClosedLoopController.h"
#include "DualMotorController.h"
#include "StatusLed.h"

// ASCII command examples:
//   M100                          -> help
//   M101                          -> one-shot human-readable status
//   M102                          -> one-shot compact binary status
//   M103 T1 S20 P3                -> binary reports every 20 ms to USB+UART
//   M200 H0 S1                    -> enable bridge 0
//   M201                          -> stop all bridges
//   M202 H0 C2                    -> assign loop to bridge 0 and select position mode
//   M203 H0 S250                  -> manual drive bridge 0 at +25.0%
//   M204 H0 R1500 U0              -> speed reference 1500 counts/s
//   M205 H0 R90 U1                -> position reference 90 degrees
//   M206 H0 L0 P0.2 I0.02 D0      -> set speed PID gains
//   M208 B10 Z0 V0                -> encoder reduced bits, zero offset, invert flag
//   M209 S1                       -> fast binary mode, compact ACKs for write commands
//   M210 H0                       -> zero the current encoder position on bridge 0
//   M211 H0 S1                    -> invert motor spin direction on bridge 0
//
// Binary frame format for UART / USB serial / I2C write:
//   [0] 0xA5
//   [1] length = command byte + payload bytes
//   [2] command
//   [3..n] payload
//   [last] XOR checksum over bytes [1] through [last-1]
class CommandProcessor {
 public:
  void begin(DualMotorController &controller,
             StatusLed &statusLed,
             ClosedLoopController &closedLoop,
             As5600Encoder &encoder,
             volatile bool &manualErrorState);
  void service();

 private:
  static constexpr uint8_t kPortUsb = 0x01;
  static constexpr uint8_t kPortUart = 0x02;
  static constexpr uint8_t kBinaryFrameStart = 0xA5;

  enum BinaryCommand : uint8_t {
    kCmdGetStatus = 0x01,
    kCmdSetFastMode = 0x02,
    kCmdSetReport = 0x03,
    kCmdSetPullups = 0x04,
    kCmdSetManualError = 0x05,
    kCmdClearFaults = 0x06,
    kCmdSetBridgeEnable = 0x10,
    kCmdStop = 0x11,
    kCmdSetMode = 0x12,
    kCmdSetManualDrive = 0x13,
    kCmdSetSpeedReference = 0x14,
    kCmdSetPositionReference = 0x15,
    kCmdSetPidGains = 0x16,
    kCmdResetLoop = 0x17,
    kCmdSetEncoderConfig = 0x18,
    kCmdSetBridgeDirection = 0x19,
    kCmdFastSetpoint = 0x20,
    kCmdFastManualDual = 0x21,
  };

  enum BinaryError : uint8_t {
    kErrBadLength = 1,
    kErrBadChecksum = 2,
    kErrUnknownCommand = 3,
    kErrInvalidArgument = 4,
    kErrBridgeDisabled = 5,
    kErrBusy = 6,
  };

  struct ResponseBuffer {
    uint8_t data[BoardConfig::MAX_RESPONSE_BYTES] = {};
    size_t length = 0;
    bool binary = false;
  };

  struct SerialParserState {
    char lineBuffer[BoardConfig::MAX_TEXT_COMMAND_LENGTH + 1] = {};
    size_t lineLength = 0;
    bool droppingLine = false;
    uint8_t binaryBuffer[BoardConfig::MAX_BINARY_PACKET_BYTES] = {};
    size_t binaryLength = 0;
    size_t expectedBinaryLength = 0;
  };

  static CommandProcessor *instance_;

  static void onI2cReceiveThunk(int count);
  static void onI2cRequestThunk();

  void onI2cReceive(int count);
  void onI2cRequest();

  void serviceSerialChannel(Stream &stream, Print &out, SerialParserState &state);
  void servicePendingI2cCommand();
  void servicePeriodicReports();
  void dispatchResponse(const ResponseBuffer &response, Print *out);
  void emitResponseToPorts(const ResponseBuffer &response, uint8_t portMask);
  void updateI2cResponse(const ResponseBuffer &response);

  void handleAsciiCommand(const char *line, ResponseBuffer &response);
  void handleBinaryPacket(const uint8_t *data, size_t length, ResponseBuffer &response);

  void buildHelpText(ResponseBuffer &response);
  void setTextResponse(ResponseBuffer &response, const char *fmt, ...);
  void beginBinaryResponse(ResponseBuffer &response, uint8_t responseCommand);
  bool appendBinaryByte(ResponseBuffer &response, uint8_t value);
  bool appendBinaryUInt16(ResponseBuffer &response, uint16_t value);
  bool appendBinaryInt16(ResponseBuffer &response, int16_t value);
  bool appendBinaryUInt32(ResponseBuffer &response, uint32_t value);
  bool appendBinaryInt32(ResponseBuffer &response, int32_t value);
  bool appendBinaryFloat(ResponseBuffer &response, float value);
  void finalizeBinaryResponse(ResponseBuffer &response);
  void buildBinaryAck(ResponseBuffer &response, uint8_t responseCommand, uint8_t status = 0x00);
  void buildBinaryAckOrStatus(ResponseBuffer &response, uint8_t requestCommand);
  void buildBinaryError(ResponseBuffer &response, uint8_t requestCommand, uint8_t errorCode);
  void buildStatusBinary(ResponseBuffer &response, uint8_t responseCommand);
  void buildStatusText(ResponseBuffer &response);

  bool parseLongParam(const char *line, char prefix, long &value) const;
  bool parseFloatParam(const char *line, char prefix, float &value) const;
  bool parseBridgeParam(const char *line, int &bridgeIndex, bool allowAll) const;
  int16_t clampPowerPermille(long value) const;
  bool isErrorActive(const ControllerSnapshot &snapshot, const ClosedLoopController::Snapshot &loopSnapshot) const;
  uint8_t buildStatusFlags(const ControllerSnapshot &snapshot, bool errorActive, bool ledReady) const;
  uint8_t buildLoopFlags(const ClosedLoopController::Snapshot &loopSnapshot) const;
  const char *modeName(ClosedLoopController::Mode mode) const;

  bool isClosedLoopBridge(int bridgeIndex) const;
  void selectClosedLoopBridge(uint8_t bridgeIndex);
  void syncClosedLoopEnableState(uint8_t bridgeIndex);
  void disableClosedLoop();
  void stopBridgeWithControlState(int bridgeIndex);
  bool applyManualDriveToBridge(int bridgeIndex, int16_t powerPermille);

  static int16_t readInt16LE(const uint8_t *data);
  static int32_t readInt32LE(const uint8_t *data);
  static float readFloatLE(const uint8_t *data);

  DualMotorController *controller_ = nullptr;
  StatusLed *statusLed_ = nullptr;
  ClosedLoopController *closedLoop_ = nullptr;
  As5600Encoder *encoder_ = nullptr;
  volatile bool *manualErrorState_ = nullptr;

  SerialParserState usbState_;
  SerialParserState uartState_;

  volatile bool i2cCommandPending_ = false;
  volatile size_t i2cRxLength_ = 0;
  uint8_t i2cRxBuffer_[BoardConfig::MAX_I2C_RX_BYTES] = {};
  uint8_t i2cResponse_[BoardConfig::MAX_RESPONSE_BYTES] = {};
  volatile size_t i2cResponseLength_ = 0;

  uint16_t textReportIntervalMs_ = BoardConfig::TEXT_REPORT_DEFAULT_INTERVAL_MS;
  uint16_t binaryReportIntervalMs_ = BoardConfig::BINARY_REPORT_DEFAULT_INTERVAL_MS;
  uint8_t textReportPorts_ = kPortUsb;
  uint8_t binaryReportPorts_ = kPortUsb;
  uint32_t lastTextReportMs_ = 0;
  uint32_t lastBinaryReportMs_ = 0;
  bool fastBinaryMode_ = BoardConfig::FAST_BINARY_MODE_DEFAULT;
};
