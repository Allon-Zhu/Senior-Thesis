#ifndef TX_RUNTIME_H
#define TX_RUNTIME_H

#include <Arduino.h>
#include <DueAdcFast.h>
#include "dacx1416.h"

struct TxRuntimeConfig {
  float tx_mux_low;
  float tx_mux_high;
  float voltage_min;
  float voltage_max;
  float pd_dwdm_ratio;
  uint16_t tx_period_us;
  uint16_t scan_settle_us;
  uint8_t prefix_count;
};

extern const int kTxBaudRate;
extern const int kTxSerialTimeout;
extern const uint32_t kTxMaxInput;
extern const uint8_t kTxLenBytes;
extern const uint8_t kTxPrefixByte;

extern DACX1416* dac0;
extern DACX1416* dac1;
extern DueAdcFast DueAdcF;
extern TxRuntimeConfig txConfig;

String waitForSerialCommand();
void initializeTxHardware();
void initializeTxParameters();
void initializeTxCommandHandlers();
void dispatchTxCommand(const String& command);

float clampVoltage(float value);
uint16_t voltageToCode(float value);
bool setDacVoltage(uint8_t chipId, uint8_t pin, float voltage);
void syncTxOutputs();
void shutdownTxOutputs();
int pdIndexToPin(uint8_t pdIndex);
uint16_t readPhotodiode(int pin);

uint8_t calc_len_bytes_u32(uint32_t value);

#endif
