#include "rx_params.h"

#include "rx_runtime.h"

namespace {

struct RxParamEntry {
  String key;
  float value;
  float defaultValue;
  bool inUse;
  bool registered;
};

const uint8_t kMaxRxParams = 16;
RxParamEntry gRxParams[kMaxRxParams];

int findParamIndex(const String& key) {
  for (uint8_t i = 0; i < kMaxRxParams; ++i) {
    if (gRxParams[i].inUse && gRxParams[i].key == key) return i;
  }
  return -1;
}

int allocateSlot() {
  for (uint8_t i = 0; i < kMaxRxParams; ++i) {
    if (!gRxParams[i].inUse) return i;
  }
  return -1;
}

void registerParam(const String& key, float defaultValue) {
  int slot = allocateSlot();
  if (slot < 0) return;
  gRxParams[slot].key = key;
  gRxParams[slot].value = defaultValue;
  gRxParams[slot].defaultValue = defaultValue;
  gRxParams[slot].inUse = true;
  gRxParams[slot].registered = true;
}

}  // namespace

void initializeRxParameterRegistry() {
  for (uint8_t i = 0; i < kMaxRxParams; ++i) gRxParams[i].inUse = false;
  registerParam("stream_delay_ms", rxConfig.stream_delay_ms);
  registerParam("decode_threshold", rxConfig.decode_threshold);
  registerParam("edge_confirm_us", rxConfig.edge_confirm_us);
  registerParam("pd_repeat", rxConfig.pd_repeat);
  registerParam("prefix_count", rxConfig.prefix_count);
}

bool setRxParameter(const String& key, float value) {
  int index = findParamIndex(key);
  if (index < 0) {
    index = allocateSlot();
    if (index < 0) return false;
    gRxParams[index].key = key;
    gRxParams[index].defaultValue = value;
    gRxParams[index].registered = false;
    gRxParams[index].inUse = true;
  }

  gRxParams[index].value = value;

  if (key == "stream_delay_ms") rxConfig.stream_delay_ms = (uint16_t)value;
  else if (key == "decode_threshold") rxConfig.decode_threshold = (uint16_t)value;
  else if (key == "edge_confirm_us") rxConfig.edge_confirm_us = (uint16_t)value;
  else if (key == "pd_repeat") rxConfig.pd_repeat = (uint16_t)value;
  else if (key == "prefix_count") rxConfig.prefix_count = (uint8_t)value;

  return true;
}

bool deleteRxParameter(const String& key) {
  int index = findParamIndex(key);
  if (index < 0) return false;
  if (gRxParams[index].registered) return setRxParameter(key, gRxParams[index].defaultValue);
  gRxParams[index].inUse = false;
  gRxParams[index].key = "";
  return true;
}

void listRxParameters() {
  Serial.println("BEGIN_PARAMS");
  for (uint8_t i = 0; i < kMaxRxParams; ++i) {
    if (!gRxParams[i].inUse) continue;
    Serial.print(gRxParams[i].key);
    Serial.print("=");
    Serial.println(gRxParams[i].value, 6);
  }
  Serial.println("END_PARAMS");
}
