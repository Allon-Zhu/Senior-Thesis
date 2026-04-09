#ifndef TX_COMMANDS_H
#define TX_COMMANDS_H

#include <Arduino.h>

void initializeTxCommandHandlers();
void dispatchTxCommand(const String& command);

#endif
