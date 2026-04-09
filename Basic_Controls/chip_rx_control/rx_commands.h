#ifndef RX_COMMANDS_H
#define RX_COMMANDS_H

#include <Arduino.h>

void initializeRxCommandHandlers();
void dispatchRxCommand(const String& command);

#endif
