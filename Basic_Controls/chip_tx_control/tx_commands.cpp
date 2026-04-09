#include "tx_commands.h"

#include <math.h>
#include <stdlib.h>
#include "modem.h"
#include "tx_params.h"
#include "tx_runtime.h"

namespace {

struct ScanAxis {
  uint8_t chip;
  uint8_t pin;
  float vInit;
  float vStart;
  float vStop;
  uint16_t count;
};

bool parseAxisLine(const String& line, ScanAxis& axis) {
  int chip;
  int pin;
  float vInit;
  float vStart;
  float vStop;
  int count;
  int parsed = sscanf(line.c_str(), "%d %d %f %f %f %d", &chip, &pin, &vInit, &vStart, &vStop, &count);
  if (parsed != 6 || count < 2) return false;

  axis.chip = (uint8_t)chip;
  axis.pin = (uint8_t)pin;
  axis.vInit = clampVoltage(vInit);
  axis.vStart = clampVoltage(vStart);
  axis.vStop = clampVoltage(vStop);
  axis.count = (uint16_t)count;
  return true;
}

float axisValueVsq(const ScanAxis& axis, uint16_t index) {
  float startSq = axis.vStart * axis.vStart;
  float stopSq = axis.vStop * axis.vStop;
  float t = (axis.count <= 1) ? 0.0f : ((float)index / (float)(axis.count - 1));
  return clampVoltage(sqrtf(startSq + t * (stopSq - startSq)));
}

void handleUpdateVoltages() {
  delay(50);
  Serial.println("ACK");

  while (true) {
    String line = waitForSerialCommand();
    if (line.length() == 0) continue;
    if (line == "END") break;

    uint8_t chipId;
    uint8_t pin;
    float voltage;
    int parsed = sscanf(line.c_str(), "%hhu %hhu %f", &chipId, &pin, &voltage);
    if (parsed != 3 || !setDacVoltage(chipId, pin, voltage)) {
      Serial.println("ERR bad chip");
      continue;
    }
  }

  syncTxOutputs();
  delayMicroseconds(50);
  Serial.println("Done.");
}

void handleSetTxLevels() {
  delay(25);
  float low = clampVoltage(Serial.parseFloat());
  float high = clampVoltage(Serial.parseFloat());
  setTxParameter("tx_mux_low", low);
  setTxParameter("tx_mux_high", high);
  Serial.println("Done");
}

void handleScan2V() {
  delay(25);
  Serial.println("ACK");

  ScanAxis axis1;
  ScanAxis axis2;
  String line = waitForSerialCommand();
  if (!parseAxisLine(line, axis1)) {
    Serial.println("ERR axis1");
    return;
  }

  line = waitForSerialCommand();
  if (!parseAxisLine(line, axis2)) {
    Serial.println("ERR axis2");
    return;
  }

  line = waitForSerialCommand();
  int pd1;
  int pd2;
  if (sscanf(line.c_str(), "%d %d", &pd1, &pd2) != 2) {
    Serial.println("ERR pds");
    return;
  }

  line = waitForSerialCommand();
  if (line != "END") {
    Serial.println("ERR no END");
    return;
  }

  Serial.println("ACK");

  uint32_t totalPoints = (uint32_t)axis1.count * (uint32_t)axis2.count;
  uint16_t* pd1Values = (uint16_t*)malloc(totalPoints * sizeof(uint16_t));
  uint16_t* pd2Values = (uint16_t*)malloc(totalPoints * sizeof(uint16_t));
  if (pd1Values == nullptr || pd2Values == nullptr) {
    if (pd1Values != nullptr) free(pd1Values);
    if (pd2Values != nullptr) free(pd2Values);
    Serial.println("ERR alloc");
    return;
  }

  setDacVoltage(axis1.chip, axis1.pin, axis1.vInit);
  setDacVoltage(axis2.chip, axis2.pin, axis2.vInit);
  syncTxOutputs();
  delayMicroseconds(50);

  uint32_t sampleIndex = 0;
  for (uint16_t i = 0; i < axis1.count; ++i) {
    setDacVoltage(axis1.chip, axis1.pin, axisValueVsq(axis1, i));
    for (uint16_t j = 0; j < axis2.count; ++j) {
      setDacVoltage(axis2.chip, axis2.pin, axisValueVsq(axis2, j));
      syncTxOutputs();
      delayMicroseconds(txConfig.scan_settle_us);
      pd1Values[sampleIndex] = readPhotodiode(pdIndexToPin((uint8_t)pd1));
      pd2Values[sampleIndex] = readPhotodiode(pdIndexToPin((uint8_t)pd2));
      ++sampleIndex;
    }
  }

  setDacVoltage(axis1.chip, axis1.pin, axis1.vInit);
  setDacVoltage(axis2.chip, axis2.pin, axis2.vInit);
  syncTxOutputs();

  Serial.println("Scan2V Finished");
  Serial.print("BEGIN ");
  Serial.print(axis1.count);
  Serial.print(" ");
  Serial.println(axis2.count);
  Serial.write((uint8_t*)pd1Values, totalPoints * sizeof(uint16_t));
  Serial.write((uint8_t*)pd2Values, totalPoints * sizeof(uint16_t));
  Serial.println("DONE");

  free(pd1Values);
  free(pd2Values);
}

size_t readLineFromSerialUsb(char* buffer, size_t maxLen) {
  if (maxLen == 0) return 0;
  size_t index = 0;
  unsigned long start = millis();
  while (millis() - start < 5000) {
    while (SerialUSB.available()) {
      char value = (char)SerialUSB.read();
      if (value == '\r') continue;
      if (value == '\n') {
        buffer[index] = '\0';
        return index;
      }
      if (index < maxLen - 1) {
        buffer[index++] = value;
      }
    }
  }
  buffer[index] = '\0';
  return index;
}

bool readExactSerialUsb(uint8_t* buffer, size_t size, unsigned long timeoutMs) {
  size_t offset = 0;
  unsigned long start = millis();
  while (offset < size && (millis() - start < timeoutMs)) {
    while (SerialUSB.available() && offset < size) {
      buffer[offset++] = (uint8_t)SerialUSB.read();
    }
  }
  return offset == size;
}

void handleQdcpPacket() {
  SerialUSB.println("READY_FOR_QDCP");
  char lenBuffer[24];
  size_t lineLen = readLineFromSerialUsb(lenBuffer, sizeof(lenBuffer));
  if (lineLen == 0) {
    SerialUSB.println("ERR_NO_LENGTH");
    return;
  }

  long packetLen = atol(lenBuffer);
  if (packetLen <= 0 || packetLen > (long)kTxMaxInput) {
    SerialUSB.println("ERR_BAD_LENGTH");
    return;
  }

  SerialUSB.println("SEND_QDCP_BYTES");
  uint8_t* packet = (uint8_t*)malloc((size_t)packetLen);
  if (packet == nullptr) {
    SerialUSB.println("ERR_ALLOC");
    return;
  }

  if (!readExactSerialUsb(packet, (size_t)packetLen, 5000)) {
    free(packet);
    SerialUSB.println("ERR_READ_TIMEOUT");
    return;
  }

  SerialUSB.println("QDCP_PACKET_RECEIVED");
  SerialUSB.println("QDCP_DONE");
  free(packet);
}

void handleSendMessage() {
  while (!Serial.available()) { yield(); }

  char inputBuffer[kTxMaxInput + 1];
  size_t payloadLen = Serial.readBytesUntil('\n', inputBuffer, kTxMaxInput);
  inputBuffer[payloadLen] = '\0';

  size_t totalLen = txConfig.prefix_count + kTxLenBytes + payloadLen;
  char* packet = (char*)malloc(totalLen);
  bool* encoded = (bool*)malloc(totalLen * 8);
  if (packet == nullptr || encoded == nullptr) {
    if (packet != nullptr) free(packet);
    if (encoded != nullptr) free(encoded);
    Serial.println("Done");
    return;
  }

  size_t index = 0;
  for (uint8_t i = 0; i < txConfig.prefix_count; ++i) packet[index++] = kTxPrefixByte;
  for (uint8_t i = 0; i < kTxLenBytes; ++i) packet[index++] = (uint8_t)((payloadLen >> (8 * i)) & 0xFF);
  for (size_t i = 0; i < payloadLen; ++i) packet[index++] = inputBuffer[i];

  Encoder(packet, totalLen, encoded);

  unsigned long start = micros();
  for (size_t bitIndex = 0; bitIndex < totalLen * 8; ++bitIndex) {
    while ((micros() - start) < (unsigned long)(bitIndex * txConfig.tx_period_us)) { }
    float level = encoded[bitIndex] ? txConfig.tx_mux_high : txConfig.tx_mux_low;
    setDacVoltage(1, 10, level);
    syncTxOutputs();
  }

  setDacVoltage(1, 10, txConfig.tx_mux_low);
  syncTxOutputs();

  free(packet);
  free(encoded);
  Serial.println("Done");
}

void handleSetParam() {
  String line = waitForSerialCommand();
  int split = line.indexOf(' ');
  if (split <= 0) {
    Serial.println("ERR param format");
    return;
  }

  String key = line.substring(0, split);
  float value = line.substring(split + 1).toFloat();
  if (!setTxParameter(key, value)) {
    Serial.println("ERR param full");
    return;
  }
  Serial.println("Done");
}

void handleDeleteParam() {
  String key = waitForSerialCommand();
  key.trim();
  if (!deleteTxParameter(key)) {
    Serial.println("ERR missing param");
    return;
  }
  Serial.println("Done");
}

}  // namespace

void initializeTxCommandHandlers() {
}

void dispatchTxCommand(const String& command) {
  if (command == "UPDATE_VOLTAGES") {
    handleUpdateVoltages();
  } else if (command == "SET_TX_LEVELS") {
    handleSetTxLevels();
  } else if (command == "SCAN_2V") {
    handleScan2V();
  } else if (command == "QDCP_PACKET") {
    handleQdcpPacket();
  } else if (command == "SEND_MESSAGE_d") {
    handleSendMessage();
  } else if (command == "TX_SHUT_DOWN") {
    shutdownTxOutputs();
    Serial.println("Done");
  } else if (command == "SET_PARAM") {
    handleSetParam();
  } else if (command == "DELETE_PARAM") {
    handleDeleteParam();
  } else if (command == "LIST_PARAMS") {
    listTxParameters();
  }
}
