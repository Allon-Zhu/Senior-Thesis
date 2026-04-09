#ifndef RX_RUNTIME_H
#define RX_RUNTIME_H

#include <Arduino.h>
#include <DueAdcFast.h>

struct RxRuntimeConfig {
  uint16_t stream_delay_ms;
  uint16_t decode_threshold;
  uint16_t edge_confirm_us;
  uint16_t pd_repeat;
  uint8_t prefix_count;
};

extern const int kRxBaudRate;
extern const int kRxSerialTimeout;
extern const uint32_t kRxMaxInput;
extern const uint8_t kRxLenBytes;

extern DueAdcFast DueAdcF;
extern RxRuntimeConfig rxConfig;

void initializeRxHardware();
void initializeRxParameters();
void initializeRxCommandHandlers();
void dispatchRxCommand(const String& command);
String waitForRxCommand();
uint8_t calc_len_bytes_u32_rx(uint32_t value);
uint16_t readPdChannel(int pin);

#endif
