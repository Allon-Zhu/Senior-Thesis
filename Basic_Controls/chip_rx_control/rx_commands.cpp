#include "rx_commands.h"

#include <stdlib.h>
#include "rx_params.h"
#include "rx_runtime.h"

namespace {

static const int kPd1Pin = A0;
static const int kPd2Pin = A1;
static const int kPd3Pin = A2;
static const int kPd4Pin = A3;

inline int readBit() {
  return (DueAdcF.ReadAnalogPin(kPd1Pin) > rxConfig.decode_threshold) ? 1 : 0;
}

bool rxOncePacket() {
RESTART_HOLD:
  while (readBit() == 0) { }
  if (rxConfig.edge_confirm_us) {
    delayMicroseconds(rxConfig.edge_confirm_us);
    if (readBit() == 0) goto RESTART_HOLD;
  }

  int last = 1;
  const uint16_t fallsNeeded = (uint16_t)(4u * rxConfig.prefix_count);
  uint32_t fallTs[32];
  if (fallsNeeded > 32) {
    Serial.println("ERR prefix_count too large");
    return false;
  }

  uint16_t fallCount = 0;
  while (fallCount < fallsNeeded) {
    int bit = readBit();
    if (bit == last) continue;

    uint32_t edgeTs = micros();
    if (rxConfig.edge_confirm_us) {
      delayMicroseconds(rxConfig.edge_confirm_us);
      if (readBit() != bit) continue;
    }

    if (last == 1 && bit == 0) {
      fallTs[fallCount++] = edgeTs;
    }
    last = bit;
  }

  const uint16_t denom = (uint16_t)(8u * rxConfig.prefix_count - 2u);
  if (denom == 0) {
    Serial.println("ERR bad prefix_count");
    return false;
  }

  uint32_t bitUs = (uint32_t)(fallTs[fallsNeeded - 1] - fallTs[0]) / (uint32_t)denom;
  uint32_t sampleTs = (uint32_t)fallTs[fallsNeeded - 1] + 7 * bitUs / 4;
  uint32_t payloadLen = 0;
  const uint8_t lenWireBits = (uint8_t)(kRxLenBytes * 8);

  for (uint8_t bit = 0; bit < lenWireBits; ++bit) {
    while ((uint32_t)micros() < sampleTs) { }
    if (readBit()) payloadLen |= (1u << bit);
    sampleTs += bitUs;
  }

  if (payloadLen > kRxMaxInput) {
    Serial.println("ERR payloadLen");
    return false;
  }

  char* payload = (char*)malloc(payloadLen + 1);
  if (payload == nullptr) {
    Serial.println("ERR alloc");
    return false;
  }

  for (uint32_t i = 0; i < payloadLen; ++i) {
    uint8_t value = 0;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      while ((uint32_t)micros() < sampleTs) { }
      if (readBit()) value |= (1u << bit);
      sampleTs += bitUs;
    }
    payload[i] = (char)value;
  }
  payload[payloadLen] = '\0';

  Serial.print("Clock=");
  Serial.print(bitUs);
  Serial.print(" |PayloadLen=");
  Serial.print((unsigned)payloadLen);
  Serial.print(" | Msg: ");
  Serial.println(payload);

  free(payload);
  return true;
}

void streamPhotodiodes() {
  for (;;) {
    if (Serial.available()) {
      String maybeStop = Serial.readStringUntil('\n');
      maybeStop.trim();
      if (maybeStop == "STREAM_STOP") {
        Serial.println("Done");
        return;
      }
    }

    uint16_t p1 = readPdChannel(kPd1Pin);
    uint16_t p2 = readPdChannel(kPd2Pin);
    uint16_t p3 = readPdChannel(kPd3Pin);
    uint16_t p4 = readPdChannel(kPd4Pin);

    char line[32];
    snprintf(line, sizeof(line), "%5u %5u %5u %5u\r\n", p1, p2, p3, p4);
    Serial.print(line);

    if (rxConfig.stream_delay_ms) delay(rxConfig.stream_delay_ms);
    yield();
  }
}

void handleSetParam() {
  String line = waitForRxCommand();
  int split = line.indexOf(' ');
  if (split <= 0) {
    Serial.println("ERR param format");
    return;
  }

  String key = line.substring(0, split);
  float value = line.substring(split + 1).toFloat();
  if (!setRxParameter(key, value)) {
    Serial.println("ERR param full");
    return;
  }
  Serial.println("Done");
}

void handleDeleteParam() {
  String key = waitForRxCommand();
  key.trim();
  if (!deleteRxParameter(key)) {
    Serial.println("ERR missing param");
    return;
  }
  Serial.println("Done");
}

}  // namespace

void initializeRxCommandHandlers() {
}

void dispatchRxCommand(const String& command) {
  if (command == "STREAM_START") {
    Serial.println("ACK");
    Serial.println(" PD1  PD2  PD3  PD4");
    streamPhotodiodes();
  } else if (command == "DECODE_MODE") {
    Serial.println("ACK");
    for (;;) (void)rxOncePacket();
  } else if (command == "MEASURE_PD1") {
    Serial.print("PD1 ");
    Serial.println(readPdChannel(kPd1Pin));
  } else if (command == "MEASURE_PD2") {
    Serial.print("PD2 ");
    Serial.println(readPdChannel(kPd2Pin));
  } else if (command == "MEASURE_PD3") {
    Serial.print("PD3 ");
    Serial.println(readPdChannel(kPd3Pin));
  } else if (command == "MEASURE_PD4") {
    Serial.print("PD4 ");
    Serial.println(readPdChannel(kPd4Pin));
  } else if (command == "SET_PARAM") {
    handleSetParam();
  } else if (command == "DELETE_PARAM") {
    handleDeleteParam();
  } else if (command == "LIST_PARAMS") {
    listRxParameters();
  }
}
