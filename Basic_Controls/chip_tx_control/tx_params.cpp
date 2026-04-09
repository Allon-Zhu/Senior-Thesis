#include "tx_params.h"

#include "tx_runtime.h"

namespace {

struct TxParamEntry {
  String key;
  float value;
  float defaultValue;
  bool inUse;
  bool registered;
};

const uint8_t kMaxTxParams = 32;
TxParamEntry gTxParams[kMaxTxParams];

int findParamIndex(const String& key) {
  for (uint8_t i = 0; i < kMaxTxParams; ++i) {
    if (gTxParams[i].inUse && gTxParams[i].key == key) {
      return i;
    }
  }
  return -1;
}

int allocateParamSlot() {
  for (uint8_t i = 0; i < kMaxTxParams; ++i) {
    if (!gTxParams[i].inUse) {
      return i;
    }
  }
  return -1;
}

void registerParam(const String& key, float defaultValue) {
  int slot = allocateParamSlot();
  if (slot < 0) return;

  gTxParams[slot].key = key;
  gTxParams[slot].defaultValue = defaultValue;
  gTxParams[slot].value = defaultValue;
  gTxParams[slot].inUse = true;
  gTxParams[slot].registered = true;
}

}  // namespace

void initializeTxParameterRegistry() {
  for (uint8_t i = 0; i < kMaxTxParams; ++i) {
    gTxParams[i].inUse = false;
  }

  registerParam("tx_mux_low", txConfig.tx_mux_low);
  registerParam("tx_mux_high", txConfig.tx_mux_high);
  registerParam("voltage_min", txConfig.voltage_min);
  registerParam("voltage_max", txConfig.voltage_max);
  registerParam("pd_dwdm_ratio", txConfig.pd_dwdm_ratio);
  registerParam("tx_period_us", (float)txConfig.tx_period_us);
  registerParam("scan_settle_us", (float)txConfig.scan_settle_us);
  registerParam("prefix_count", (float)txConfig.prefix_count);
}

bool setTxParameter(const String& key, float value) {
  int index = findParamIndex(key);
  if (index >= 0) {
    gTxParams[index].value = value;
    if (key == "tx_mux_low") txConfig.tx_mux_low = value;
    else if (key == "tx_mux_high") txConfig.tx_mux_high = value;
    else if (key == "voltage_min") txConfig.voltage_min = value;
    else if (key == "voltage_max") txConfig.voltage_max = value;
    else if (key == "pd_dwdm_ratio") txConfig.pd_dwdm_ratio = value;
    else if (key == "tx_period_us") gTxParams[index].value = (float)(txConfig.tx_period_us = (uint16_t)value);
    else if (key == "scan_settle_us") gTxParams[index].value = (float)(txConfig.scan_settle_us = (uint16_t)value);
    else if (key == "prefix_count") gTxParams[index].value = (float)(txConfig.prefix_count = (uint8_t)value);
    return true;
  }

  int slot = allocateParamSlot();
  if (slot < 0) return false;
  gTxParams[slot].key = key;
  gTxParams[slot].value = value;
  gTxParams[slot].defaultValue = value;
  gTxParams[slot].inUse = true;
  gTxParams[slot].registered = false;
  return true;
}

bool deleteTxParameter(const String& key) {
  int index = findParamIndex(key);
  if (index < 0) return false;

  if (gTxParams[index].registered) {
    return setTxParameter(key, gTxParams[index].defaultValue);
  }

  gTxParams[index].inUse = false;
  gTxParams[index].key = "";
  return true;
}

bool getTxParameter(const String& key, float& value) {
  int index = findParamIndex(key);
  if (index < 0) return false;
  value = gTxParams[index].value;
  return true;
}

void listTxParameters() {
  Serial.println("BEGIN_PARAMS");
  for (uint8_t i = 0; i < kMaxTxParams; ++i) {
    if (!gTxParams[i].inUse) continue;
    Serial.print(gTxParams[i].key);
    Serial.print("=");
    Serial.println(gTxParams[i].value, 6);
  }
  Serial.println("END_PARAMS");
}
