#include "rx_runtime.h"

#include "rx_commands.h"
#include "rx_params.h"

const int kRxBaudRate = 115200;
const int kRxSerialTimeout = 200;
const uint32_t kRxMaxInput = 2048;
const uint8_t kRxLenBytes = calc_len_bytes_u32_rx(kRxMaxInput);

static const int kPd1Pin = A0;
static const int kPd2Pin = A1;
static const int kPd3Pin = A2;
static const int kPd4Pin = A3;

DueAdcFast DueAdcF(1024);

RxRuntimeConfig rxConfig = {
  50,
  500,
  2,
  5,
  5
};

uint8_t calc_len_bytes_u32_rx(uint32_t value) {
  return (value <= 0xFFu) ? 1 :
         (value <= 0xFFFFu) ? 2 :
         (value <= 0xFFFFFFu) ? 3 : 4;
}

String waitForRxCommand() {
  while (Serial.available() == 0) { yield(); }
  String input = Serial.readStringUntil('\n');
  input.trim();
  return input;
}

uint16_t readPdChannel(int pin) {
  uint32_t accumulator = 0;
  for (uint16_t i = 0; i < rxConfig.pd_repeat; ++i) {
    accumulator += (uint32_t)DueAdcF.ReadAnalogPin(pin);
  }
  return (uint16_t)(accumulator / (uint32_t)rxConfig.pd_repeat);
}

void initializeRxParameters() {
  initializeRxParameterRegistry();
}

void initializeRxHardware() {
  Serial.begin(kRxBaudRate);
  Serial.setTimeout(kRxSerialTimeout);
  DueAdcF.EnablePin(kPd1Pin);
  DueAdcF.EnablePin(kPd2Pin);
  DueAdcF.EnablePin(kPd3Pin);
  DueAdcF.EnablePin(kPd4Pin);
  DueAdcF.Start1Mhz();
  Serial.println("RX Ready.");
}
