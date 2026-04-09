#include "tx_runtime.h"

#include <math.h>
#include "modem.h"
#include "tx_commands.h"
#include "tx_params.h"

const int kTxBaudRate = 115200;
const int kTxSerialTimeout = 200;
const uint32_t kTxMaxInput = 2048;
const uint8_t kTxPrefixByte = 'U';
const uint8_t kTxLenBytes = calc_len_bytes_u32(kTxMaxInput);

static const uint16_t kOutMax = 65535;
static const uint8_t kAllRange = 40;

static const uint8_t kDac0Cs = 13;
static const uint8_t kDac1Cs = 24;
static const uint8_t kDac0Rst = 28;
static const uint8_t kDac1Rst = 26;
static const uint8_t kDacLdac = 12;
static const int kSpiSpeed = 21000000;

static const int kPd1Pin = A0;
static const int kPd2Pin = A1;
static const int kPd3Pin = A2;
static const int kPd4Pin = A3;

DACX1416* dac0 = nullptr;
DACX1416* dac1 = nullptr;
DueAdcFast DueAdcF(1024);

TxRuntimeConfig txConfig = {
  12.21f,
  10.21f,
  0.0f,
  30.0f,
  0.31f,
  100,
  100,
  5
};

uint8_t calc_len_bytes_u32(uint32_t value) {
  return (value <= 0xFFu) ? 1 :
         (value <= 0xFFFFu) ? 2 :
         (value <= 0xFFFFFFu) ? 3 : 4;
}

String waitForSerialCommand() {
  while (!Serial.available()) {
    if (SerialUSB.available()) {
      String usbInput = SerialUSB.readStringUntil('\n');
      usbInput.trim();
      return usbInput;
    }
    yield();
  }

  String input = Serial.readStringUntil('\n');
  input.trim();
  return input;
}

float clampVoltage(float value) {
  if (value < txConfig.voltage_min) value = txConfig.voltage_min;
  if (value > txConfig.voltage_max) value = txConfig.voltage_max;
  return value;
}

uint16_t voltageToCode(float value) {
  float ratio = clampVoltage(value) / (float)kAllRange;
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return (uint16_t)roundf(ratio * kOutMax);
}

bool setDacVoltage(uint8_t chipId, uint8_t pin, float voltage) {
  if (chipId == 0 && dac0 != nullptr) {
    dac0->set_out(pin, voltageToCode(voltage));
    return true;
  }
  if (chipId == 1 && dac1 != nullptr) {
    dac1->set_out(pin, voltageToCode(voltage));
    return true;
  }
  return false;
}

void syncTxOutputs() {
  if (dac0 != nullptr) dac0->sync(1);
  if (dac1 != nullptr) dac1->sync(1);
}

void shutdownTxOutputs() {
  for (uint8_t channel = 0; channel < 16; ++channel) {
    if (dac0 != nullptr) dac0->set_out(channel, 0);
    if (dac1 != nullptr) dac1->set_out(channel, 0);
  }
  syncTxOutputs();
}

int pdIndexToPin(uint8_t pdIndex) {
  switch (pdIndex) {
    case 1: return kPd1Pin;
    case 2: return kPd2Pin;
    case 3: return kPd3Pin;
    case 4: return kPd4Pin;
    default: return kPd1Pin;
  }
}

uint16_t readPhotodiode(int pin) {
  return (uint16_t)DueAdcF.ReadAnalogPin(pin);
}

void initializeTxParameters() {
  initializeTxParameterRegistry();
}

void initializeTxHardware() {
  Serial.begin(kTxBaudRate);
  Serial.setTimeout(kTxSerialTimeout);
  SerialUSB.begin(kTxBaudRate);
  SerialUSB.setTimeout(kTxSerialTimeout);

  dac0 = new DACX1416(kDac0Cs, kDac0Rst, kDacLdac, &SPI, kSpiSpeed);
  dac1 = new DACX1416(kDac1Cs, kDac1Rst, kDacLdac, &SPI, kSpiSpeed);

  dac0->read_reg(R_DEVICEID);
  dac1->read_reg(R_DEVICEID);
  dac0->init();
  dac1->init();
  dac0->set_int_reference(false);
  dac1->set_int_reference(false);

  for (uint8_t channel = 0; channel < 16; ++channel) {
    dac0->set_ch_enabled(channel, true);
    dac0->set_range(channel, DACX1416::U_40);
    dac0->set_ch_sync(channel, true);
    dac1->set_ch_enabled(channel, true);
    dac1->set_range(channel, DACX1416::U_40);
    dac1->set_ch_sync(channel, true);
  }

  DueAdcF.EnablePin(kPd1Pin);
  DueAdcF.EnablePin(kPd2Pin);
  DueAdcF.EnablePin(kPd3Pin);
  DueAdcF.EnablePin(kPd4Pin);
  DueAdcF.Start1Mhz();

  Serial.println("TX Ready.");
}
