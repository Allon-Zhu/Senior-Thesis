#ifndef TX_PARAMS_H
#define TX_PARAMS_H

#include <Arduino.h>

void initializeTxParameterRegistry();
bool setTxParameter(const String& key, float value);
bool deleteTxParameter(const String& key);
bool getTxParameter(const String& key, float& value);
void listTxParameters();

#endif
