#include <math.h>
#include "modem.h"
#include "control.h"
#include "Arduino.h"
#include <stdlib.h>

#pragma region User Parameters
// Serial Communication
int BaudRate = 115200;
int SerialTimeOut = 200;
int Serial_Input = 0;

// TX Clock Period
const uint16_t Tx_period_us = 100;

// TX Clock Recovery
#define PREFIX_COUNT 4
#define PREFIX_BYTE 'U'

// TX Max Size
#define MAX_INPUT 2048


// DAC parameters
#define DAC_CS 13
#define DAC_RST 28
#define DAC_LDAC 12
#define SYNC 10
const int SPI_SPEED = 21000000;
DACX1416* dac;
const int OUT_MAX = 65535;
const uint8_t ALL_RAN = 40;

// Voltage Sanity Bounds
const float V_MIN = 0.0;
const float V_MAX_mzi = 30.0;
const float V_MAX_qf = 30.0;
const float V_MAX_eps = 27.0;

// Voltage Control Pins
const uint8_t PStop2_PIN = 8;
const uint8_t MZMtop2_PIN = 9;
const uint8_t PStop1_PIN = 10;
const uint8_t MZMtop1_PIN = 11;
const uint8_t MZM1_PIN = 12;
const uint8_t MZMC_PIN = 13;
const uint8_t EPStop_PIN = 14;
const uint8_t QF2top_PIN = 15;
const uint8_t QF3top_PIN = 0;
const uint8_t QF4_PIN = 1;
const uint8_t QF3bot_PIN = 2;
const uint8_t QF2bot_PIN = 3;
const uint8_t EPSbot_PIN = 4;
const uint8_t MZMQ_PIN = 5;
const uint8_t PSbot_PIN = 6;
const uint8_t MZMbot_PIN = 7;

// Default Voltages
float V_MZMtop1 = 18.1;
float V_MZMtop2 = 21.0;
float V_MZMbot = 29.5;
float V_PStop1 = 21.0;
float V_PStop2 = 23.0;
float V_PSbot = 21.9;
float V_MZM1 = 24.0;
float V_MZMQ = 22.80;
float V_MZMC = 30.5;
float V_QF4 = 21.0;
float V_EPStop = 11.25;
float V_QF2top = 27.3;
float V_QF3top = 19.3;
float V_EPSbot = 11.20;
float V_QF2bot = 26.28;
float V_QF3bot = 19.60;

// QF4 Low and High Voltages
float Tx_VH = 19.0;
float Tx_VL = 23.0;

// Teensy Control Pin
const int TEENSY_PIN = 53;

#pragma endregion


void update_voltages() {
  delay(200);
  V_EPStop = Serial.parseFloat();
  V_QF2top = Serial.parseFloat();
  V_QF3top = Serial.parseFloat();
  V_EPSbot = Serial.parseFloat();
  V_QF2bot = Serial.parseFloat();
  V_QF3bot = Serial.parseFloat();
  V_QF4 = Serial.parseFloat();
  V_MZMtop1 = Serial.parseFloat();
  V_MZMtop2 = Serial.parseFloat();
  V_MZMbot = Serial.parseFloat();
  V_PStop1 = Serial.parseFloat();
  V_PStop2 = Serial.parseFloat();
  V_PSbot = Serial.parseFloat();
  V_MZM1 = Serial.parseFloat();
  V_MZMQ = Serial.parseFloat();
  V_MZMC = Serial.parseFloat();

  dac -> set_out(MZM1_PIN, int(V_MZM1/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMtop1_PIN, int(V_MZMtop1/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMbot_PIN, int(V_MZMbot/ALL_RAN*OUT_MAX));
  dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
  dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
  dac -> set_out(PSbot_PIN, int(V_PSbot/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMQ_PIN, int(V_MZMQ/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMC_PIN, int(V_MZMC/ALL_RAN*OUT_MAX));
  dac -> set_out(EPStop_PIN, int(V_EPStop/ALL_RAN*OUT_MAX));
  dac -> set_out(QF2top_PIN, int(V_QF2top/ALL_RAN*OUT_MAX));
  dac -> set_out(QF3top_PIN, int(V_QF3top/ALL_RAN*OUT_MAX));
  dac -> set_out(EPSbot_PIN, int(V_EPSbot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF2bot_PIN, int(V_QF2bot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF3bot_PIN, int(V_QF3bot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF4_PIN, int(V_QF4/ALL_RAN*OUT_MAX));
  dac -> sync(1);
  delayMicroseconds(50);
  Serial.println("Done.");
}

String waitForSerialCommand() {
  // Wait until data is available on Serial
  while (Serial.available() == 0) {
    // Do nothing (blocking)
  }

  // Read the incoming string until newline
  String input = Serial.readStringUntil('\n');

  // Remove any carriage return or whitespace
  input.trim();

  // Optional: print it back for confirmation
  Serial.print("Received: ");
  Serial.println(input);

  return input;
}

bool printPacketBits(const bool* bits, size_t n) {
  char* buf = (char*)malloc(n + 1);   // +1 for '\n' (or '\0' if you prefer)
  if (!buf) return false;

  for (size_t i = 0; i < n; ++i) {
    buf[i] = bits[i] ? '1' : '0';
  }
  buf[n] = '\n';

  Serial.write(buf, n + 1);
  free(buf);
  return true;
}

void init_voltages() {
  // Tune everyone to the correct voltage
  dac -> set_out(MZM1_PIN, int(V_MZM1/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMtop1_PIN, int(V_MZMtop1/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMbot_PIN, int(V_MZMbot/ALL_RAN*OUT_MAX));
  dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
  dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
  dac -> set_out(PSbot_PIN, int(V_PSbot/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMQ_PIN, int(V_MZMQ/ALL_RAN*OUT_MAX));
  dac -> set_out(MZMC_PIN, int(V_MZMC/ALL_RAN*OUT_MAX));
  dac -> set_out(EPStop_PIN, int(V_EPStop/ALL_RAN*OUT_MAX));
  dac -> set_out(QF2top_PIN, int(V_QF2top/ALL_RAN*OUT_MAX));
  dac -> set_out(QF3top_PIN, int(V_QF3top/ALL_RAN*OUT_MAX));
  dac -> set_out(EPSbot_PIN, int(V_EPSbot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF2bot_PIN, int(V_QF2bot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF3bot_PIN, int(V_QF3bot/ALL_RAN*OUT_MAX));
  dac -> set_out(QF4_PIN, int(V_QF4/ALL_RAN*OUT_MAX));
  dac -> sync(1);
  delay(2000);
  Serial.println("Ready.");

  // Send voltage data to Tx_Control
  while (Serial.available() == 0) {}
  Serial.parseInt();
  Serial.println(V_EPStop);
  Serial.println(V_QF2top);
  Serial.println(V_QF3top);
  Serial.println(V_EPSbot);
  Serial.println(V_QF2bot);
  Serial.println(V_QF3bot);
  Serial.println(V_QF4);
  Serial.println(V_MZMtop1);
  Serial.println(V_MZMtop2);
  Serial.println(V_MZMbot);
  Serial.println(V_PStop1);
  Serial.println(V_PStop2);
  Serial.println(V_PSbot);
  Serial.println(V_MZM1);
  Serial.println(V_MZMQ);
  Serial.println(V_MZMC);
}


void setup() {

  // Setup Serial
  Serial.begin(BaudRate);
  Serial.setTimeout(SerialTimeOut);

  // Enable Pins
  pinMode(SYNC, OUTPUT);
  pinMode(TEENSY_PIN, OUTPUT);

  // Enable DAC
  dac = new DACX1416(DAC_CS, DAC_RST, DAC_LDAC, &SPI, SPI_SPEED);
  delay(1500);
  dac -> read_reg(R_DEVICEID);
  int res = dac -> init();
  dac -> set_int_reference(false);
  for(int i=0; i<16; i++){
    dac -> set_ch_enabled(i, true);
    dac -> set_range(i, DACX1416::U_40);
    dac -> set_ch_sync(i, true);
  }

  // Initialize Voltages
  init_voltages()
}


void loop() {
  String command = waitForSerialCommand();

  if (command == "UPDATE_VOLTAGES") {
    update_voltages()
    Serial.println("Voltages Updated.")
  } else if(command == "SEND_MESSAGE"){
    
    // Wait for Message to Send
    while(!Serial.available()){}

    // Read message and get length
    char inputBuf[MAX_INPUT + 1];
    size_t n = Serial.readBytesUntil('\n', inputBuf, MAX_INPUT);
    inputBuf[n] = '\0';
    uint8_t payloadLen = static_cast<uint8_t>(n);
    
    size_t totalLen = PREFIX_COUNT + 1 + payloadLen;

    // Create Packet in ASCII
    char packet[totalLen + 1];
    size_t idx = 0;
    for (uint8_t i = 0; i < PREFIX_COUNT; ++i) {
      packet[idx++] = PREFIX_BYTE;
    }

    // Add length byte
    packet[idx++] = payloadLen;

    // Add payload bytes
    for (uint8_t i = 0; i < payloadLen; ++i) {
      packet[idx++] = static_cast<uint8_t>(inputBuf[i]);
    }
    packet[idx] = '\0';

    // Encode Packet in Binary
    bool packetBin[totalLen * 8];
    Encoder(packet, totalLen, packetBin);

    // Send each bit in order
    Tx_t1 = micros();
    tic = Tx_t1;
    Tx_idx = 0;
    Data_idx = 0;

    while(true){
      // Check if packet should continue being sent
      if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
        // If packet isn't done sending, send next bit
        if(Tx_idx < totalLen * 8){
          Tx_V = (packetBin[Data_idx] == 0) ? Tx_VL : Tx_VH;
          Data_idx++;
          dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
          dac -> sync(1);
          Tx_idx++;
          delayMicroseconds(30);
        }
        // Else, set output to 0 and break
        else if(Tx_idx >= totalLen*8){
          dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
          dac -> sync(1);
          break;
        }
      }
    }
    Serial.println("Done");
  } else if (command == "TEST_TEENSY") {
    while(!Serial.available()){}
    char mode = Serial.read();
    if(mode == '0') {
      digitalWrite(TEENSY_PIN, LOW);
      Serial.println("Setting Low");
    } else if(mode == '1') {
      digitalWrite(TEENSY_PIN, HIGH);
      Serial.println("Setting High");
    }
    Serial.println("Done");
  }
}
