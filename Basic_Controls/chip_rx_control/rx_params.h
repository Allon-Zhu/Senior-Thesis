#ifndef RX_PARAMS_H
#define RX_PARAMS_H

#include <Arduino.h>

void initializeRxParameterRegistry();
bool setRxParameter(const String& key, float value);
bool deleteRxParameter(const String& key);
void listRxParameters();

#endif
