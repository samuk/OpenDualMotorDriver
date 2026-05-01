#include "As5600Encoder.h"
#include "BoardConfig.h"
#include "ClosedLoopController.h"
#include "CommandProcessor.h"
#include "DualMotorController.h"
#include "StatusLed.h"

StatusLed g_statusLed;
DualMotorController g_motorController;
As5600Encoder g_encoder;
ClosedLoopController g_closedLoop;
CommandProcessor g_commandProcessor;
volatile bool g_manualErrorState = false;

void setup() {
  Serial.begin(BoardConfig::USB_SERIAL_BAUD);

  Serial1.setTX(BoardConfig::PIN_UART0_TX);
  Serial1.setRX(BoardConfig::PIN_UART0_RX);
  Serial1.begin(BoardConfig::UART_BAUD);

  g_motorController.begin();

  if (!g_statusLed.begin()) {
    g_manualErrorState = true;
    Serial.println("PCA9633 init failed on I2C0.");
    Serial1.println("PCA9633 init failed on I2C0.");
  } else if (!g_statusLed.runStartupSequence()) {
    g_manualErrorState = true;
    Serial.println("PCA9633 startup sequence failed.");
    Serial1.println("PCA9633 startup sequence failed.");
  }

  g_encoder.begin(Wire);
  g_encoder.setResolutionBits(BoardConfig::AS5600_DEFAULT_REDUCED_BITS);
  g_closedLoop.begin(&g_encoder, BoardConfig::CLOSED_LOOP_DEFAULT_BRIDGE);
  g_closedLoop.setEnabled(false);
  g_closedLoop.setMode(ClosedLoopController::Mode::Manual);

  if (!g_encoder.sample()) {
    Serial.println("AS5600 not responding on I2C0.");
    Serial1.println("AS5600 not responding on I2C0.");
  }

  g_commandProcessor.begin(g_motorController, g_statusLed, g_closedLoop, g_encoder, g_manualErrorState);

  Serial.println("PicoDualMotorDriver ready. Use M100 for help.");
  Serial1.println("PicoDualMotorDriver ready. Use M100 for help.");
}

void loop() {
  g_motorController.service();
  g_closedLoop.update();

  const ClosedLoopController::Snapshot &loopSnapshot = g_closedLoop.snapshot();
  if (loopSnapshot.bridgeIndex < BoardConfig::BRIDGE_COUNT) {
    g_motorController.setMotorPower(loopSnapshot.bridgeIndex, loopSnapshot.outputCommandPermille);
  }

  const bool loopError = loopSnapshot.enabled &&
                         loopSnapshot.mode != ClosedLoopController::Mode::Manual &&
                         !loopSnapshot.encoderHealthy;
  const bool errorActive =
      g_manualErrorState || g_motorController.isFaultActive() || !g_statusLed.isReady() || loopError;
  g_statusLed.update(errorActive, g_motorController.getEnabledBridgeCount());

  g_commandProcessor.service();
  delay(BoardConfig::MAIN_LOOP_DELAY_MS);
}
