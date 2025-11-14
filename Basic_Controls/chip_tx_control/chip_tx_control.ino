#include <math.h>
#include "modem.h"
#include "control.h"
#include "Arduino.h"
#include <stdlib.h>

// ========================== Serial / Timing ==========================
static const int BaudRate       = 115200;
static const int SerialTimeOut  = 200;

static const uint16_t Tx_period_us = 100;

#define PREFIX_COUNT 5
#define PREFIX_BYTE 'U'
#define MAX_INPUT 2048

constexpr uint8_t calc_len_bytes_u32(uint32_t x) {
  return (x <= 0xFFu)       ? 1 :
         (x <= 0xFFFFu)     ? 2 :
         (x <= 0xFFFFFFu)   ? 3 :
                              4;
}
constexpr uint8_t LEN_BYTES = calc_len_bytes_u32(MAX_INPUT);


// ========================== DAC / Hardware ==========================
#define DAC_CS   13
#define DAC_RST  28
#define DAC_LDAC 12
#define SYNC     10
const int SPI_SPEED = 21000000;
DACX1416* dac = nullptr;

const int OUT_MAX = 65535;
const uint8_t ALL_RAN = 40;   // DAC range U_40 (0..40 V logical)

const float V_MIN = 0.0f;
const float V_MAX = 30.0f;    // sanity clamp, even if DAC range is 40 V

// These are DAC channel indices (not MCU GPIO pins)
const uint8_t PStop2_PIN  = 8;
const uint8_t MZMtop2_PIN = 9;
const uint8_t PStop1_PIN  = 10;
const uint8_t MZMtop1_PIN = 11;
const uint8_t MZM1_PIN    = 12;
const uint8_t MZMC_PIN    = 13;
const uint8_t EPStop_PIN  = 14;
const uint8_t QF2top_PIN  = 15;
const uint8_t QF3top_PIN  = 0;
const uint8_t QF4_PIN     = 1;
const uint8_t QF3bot_PIN  = 2;
const uint8_t QF2bot_PIN  = 3;
const uint8_t EPSbot_PIN  = 4;
const uint8_t MZMQ_PIN    = 5;
const uint8_t PSbot_PIN   = 6;
const uint8_t MZMbot_PIN  = 7;

// QF4 signaling levels (unchanged from your code)
float Tx_VH = 19.0f;
float Tx_VL = 23.0f;

unsigned long Tx_t1 = 0;
int Tx_idx = 0;
int Data_idx = 0;
float Tx_V = 0.0;

const int TEENSY_PIN = 53;

// ========================== Current Setpoints ==========================
float V_EPStop   = 0.0f;
float V_QF2top   = 0.0f;
float V_QF3top   = 0.0f;
float V_EPSbot   = 0.0f;
float V_QF2bot   = 0.0f;
float V_QF3bot   = 0.0f;
float V_QF4      = 0.0f;
float V_MZMtop1  = 0.0f;
float V_MZMtop2  = 0.0f;
float V_MZMbot   = 0.0f;
float V_PStop1   = 0.0f;
float V_PStop2   = 0.0f;
float V_PSbot    = 0.0f;
float V_MZM1     = 0.0f;
float V_MZMQ     = 0.0f;
float V_MZMC     = 0.0f;

// Read-but-unused (keeps the stream aligned with Python)
float PD_DWDM    = 0.0f;
float RT_DWDM    = 0.0f;

// ========================== Helpers ==========================
static inline float clampV(float v) {
  if (v < V_MIN) v = V_MIN;
  if (v > V_MAX) v = V_MAX;
  return v;
}

static inline uint16_t v2code(float v) {
  float vv = clampV(v);
  float ratio = vv / (float)ALL_RAN;       // normalize to 0..1 over 40 V full-scale
  if (ratio < 0.0f) ratio = 0.0f;
  if (ratio > 1.0f) ratio = 1.0f;
  return (uint16_t)roundf(ratio * OUT_MAX);
}

String waitForSerialCommand() {
  while (Serial.available() == 0) {
    // blocking wait
  }
  String input = Serial.readStringUntil('\n');
  input.trim();
  return input;
}

void set_tx_levels() {
  // Read two floats: VL then VH
  delay(50);
  float newVL = clampV(Serial.parseFloat());
  float newVH = clampV(Serial.parseFloat());

  // Assign (no ordering enforced; you currently use VL as idle)
  Tx_VL = newVL;
  Tx_VH = newVH;

  // Immediately drive QF4 to the (new) idle/low level and sync
  delayMicroseconds(50);

  Serial.println("Done");
}


bool printPacketBits(const bool* bits, size_t n) {
  char* buf = (char*)malloc(n + 1);
  if (!buf) return false;
  for (size_t i = 0; i < n; ++i) buf[i] = bits[i] ? '1' : '0';
  buf[n] = '\n';
  Serial.write(buf, n + 1);
  free(buf);
  return true;
}

// ========================== UPDATE_VOLTAGES ==========================
void update_voltages() {
  // Read floats in the exact order Python sends them
  delay(200);
  Serial.println("ACK");
  V_EPStop   = clampV(Serial.parseFloat());
  V_QF2top   = clampV(Serial.parseFloat());
  V_QF3top   = clampV(Serial.parseFloat());
  V_EPSbot   = clampV(Serial.parseFloat());
  V_QF2bot   = clampV(Serial.parseFloat());
  V_QF3bot   = clampV(Serial.parseFloat());
  V_QF4      = clampV(Serial.parseFloat());
  V_MZMtop1  = clampV(Serial.parseFloat());
  V_MZMtop2  = clampV(Serial.parseFloat());
  V_MZMbot   = clampV(Serial.parseFloat());
  V_PStop1   = clampV(Serial.parseFloat());
  V_PStop2   = clampV(Serial.parseFloat());
  V_PSbot    = clampV(Serial.parseFloat());
  V_MZM1     = clampV(Serial.parseFloat());
  V_MZMQ     = clampV(Serial.parseFloat());
  V_MZMC     = clampV(Serial.parseFloat());

  // Keep stream aligned (read but not used for DAC)
  PD_DWDM     = Serial.parseFloat();
  RT_DWDM     = Serial.parseFloat();

  // Always set all outputs, then always sync
  dac->set_out(MZM1_PIN,    v2code(V_MZM1));
  dac->set_out(MZMtop1_PIN, v2code(V_MZMtop1));
  dac->set_out(MZMtop2_PIN, v2code(V_MZMtop2));
  dac->set_out(MZMbot_PIN,  v2code(V_MZMbot));
  dac->set_out(PStop1_PIN,  v2code(V_PStop1));
  dac->set_out(PStop2_PIN,  v2code(V_PStop2));
  dac->set_out(PSbot_PIN,   v2code(V_PSbot));
  dac->set_out(MZMQ_PIN,    v2code(V_MZMQ));
  dac->set_out(MZMC_PIN,    v2code(V_MZMC));
  dac->set_out(EPStop_PIN,  v2code(V_EPStop));
  dac->set_out(QF2top_PIN,  v2code(V_QF2top));
  dac->set_out(QF3top_PIN,  v2code(V_QF3top));
  dac->set_out(EPSbot_PIN,  v2code(V_EPSbot));
  dac->set_out(QF2bot_PIN,  v2code(V_QF2bot));
  dac->set_out(QF3bot_PIN,  v2code(V_QF3bot));
  dac->set_out(QF4_PIN,     v2code(V_QF4));

  dac->sync(1);                 // <-- always sync after an update
  delayMicroseconds(50);

  Serial.println("Done.");
}

// ========================== Setup / Loop ==========================
void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(SerialTimeOut);

  pinMode(SYNC, OUTPUT);
  pinMode(TEENSY_PIN, OUTPUT);

  // Initialize DAC
  dac = new DACX1416(DAC_CS, DAC_RST, DAC_LDAC, &SPI, SPI_SPEED);
  delay(1500);
  dac->read_reg(R_DEVICEID);
  (void)dac->init();
  dac->set_int_reference(false);

  // Enable/sync all channels, set to 0..40V logical range
  for (int i = 0; i < 16; i++) {
    dac->set_ch_enabled(i, true);
    dac->set_range(i, DACX1416::U_40);
    dac->set_ch_sync(i, true);  // changes stage until sync()
  }

  Serial.println("Ready.");
}

void loop() {
  String command = waitForSerialCommand();

  if (command == "UPDATE_VOLTAGES") {
    update_voltages();  // prints "Done."
  }

  else if (command == "SEND_MESSAGE") {
    // Wait for message payload line
    while (!Serial.available()) { /* wait */ }

    char inputBuf[MAX_INPUT + 1];
    size_t n = Serial.readBytesUntil('\n', inputBuf, MAX_INPUT);
    inputBuf[n] = '\0';
    uint32_t payloadLen = static_cast<uint32_t>(n);
    if (payloadLen > MAX_INPUT) payloadLen = MAX_INPUT;

    size_t totalLen = PREFIX_COUNT + LEN_BYTES + payloadLen;
    Serial.println(totalLen);

    // Build ASCII packet [ U U U U | len | payload... ]
    char* packet = (char*)malloc(totalLen);
    if (!packet) {
      Serial.println("Done");
      return;
    }
    size_t idx = 0;
    for (uint8_t i = 0; i < PREFIX_COUNT; ++i) packet[idx++] = PREFIX_BYTE;
    for (uint8_t i = 0; i < LEN_BYTES; ++i) packet[idx++] = (uint8_t)((payloadLen >> (8 * i)) & 0xFF);
    for (uint32_t i = 0; i < payloadLen; ++i) packet[idx++] = static_cast<uint8_t>(inputBuf[i]);

    // Encode to bits via your modem
    bool* packetBin = (bool*)malloc(totalLen * 8);
    if (!packetBin) {
      free(packet);
      Serial.println("Done");
      return;
    }
    Encoder(packet, totalLen, packetBin);  // from modem.h

    // Transmit bits using your globals (from control.h/modem.h)
    Tx_t1  = micros();
    Tx_idx = 0;
    Data_idx = 0;

    while (true) {
      if (micros() - Tx_t1 >= Tx_idx * Tx_period_us) {
        if (Tx_idx < (int)(totalLen * 8)) {
          Tx_V = (packetBin[Data_idx] == 0) ? Tx_VL : Tx_VH;
          Data_idx++;
          dac->set_out(QF4_PIN, v2code(Tx_V));
          dac->sync(1);              // latch every bit transition
          Tx_idx++;
          delayMicroseconds(30);
        } else {
          // Done: set to idle level and break
          dac->set_out(QF4_PIN, v2code(Tx_VL));
          dac->sync(1);
          break;
        }
      }
    }

    free(packetBin);
    free(packet);
    Serial.println("Done");
  }

  else if (command == "TEST_TEENSY") {
    while (!Serial.available()) { /* wait */ }
    char mode = Serial.read();
    if (mode == '0') {
      digitalWrite(TEENSY_PIN, LOW);
      Serial.println("Setting Low");
    } else if (mode == '1') {
      digitalWrite(TEENSY_PIN, HIGH);
      Serial.println("Setting High");
    }
    Serial.println("Done");
  }

  else if (command == "SET_TX_LEVELS") {
    set_tx_levels();
  }

  else {
    // Unknown command; ignore or add debug
    // Serial.println("Unknown command");
  }
}
