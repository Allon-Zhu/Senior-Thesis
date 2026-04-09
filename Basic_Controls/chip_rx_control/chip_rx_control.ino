#include "rx_commands.h"
#include "rx_runtime.h"

void setup() {
  initializeRxParameters();
  initializeRxCommandHandlers();
  initializeRxHardware();
}

void ADC_Handler() {
  DueAdcF.adcHandler();
}

void loop() {
  String command = waitForRxCommand();
  dispatchRxCommand(command);
}
