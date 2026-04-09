#include "tx_commands.h"
#include "tx_runtime.h"

void setup() {
  initializeTxParameters();
  initializeTxCommandHandlers();
  initializeTxHardware();
}

void ADC_Handler() {
  DueAdcF.adcHandler();
}

void loop() {
  String command = waitForSerialCommand();
  dispatchTxCommand(command);
}
