#include <Arduino.h>
#include <DueAdcFast.h>

// =================== Configuration ===================
static const int BaudRate       = 115200;
static const int SerialTimeOut  = 200;

DueAdcFast DueAdcF(1024);
const int PD1_PIN = A0;
const int PD2_PIN = A1;
const int PD3_PIN = A2;
const int PD4_PIN = A3;

volatile int      PD_Repeat = 5;     // average N samples per channel
volatile uint16_t Delay_ms  = 50;    // delay between lines while streaming

#define THRESH 500
#define READ_BIT()  ((DueAdcF.ReadAnalogPin(PD1_PIN) > THRESH) ? 1 : 0)

// ====== Simple packet decode config (match your TX) ======
#define PREFIX_COUNT 5
#define EDGE_CONFIRM_US 2
#define MAX_INPUT 2048

constexpr uint8_t calc_len_bytes_u32(uint32_t x) {
  return (x <= 0xFFu) ? 1 :
         (x <= 0xFFFFu) ? 2 :
         (x <= 0xFFFFFFu) ? 3 : 4;
}
constexpr uint8_t LEN_BYTES = calc_len_bytes_u32(MAX_INPUT);

// =================== Helpers ===================
static inline String waitForLine() {
  while (Serial.available() == 0) { yield(); }
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

static inline void wait_until_us(uint32_t t) {
  while ((int32_t)(micros() - t) < 0) { /* tight spin; keep fast for timing */ }
}

uint16_t read_pd_channel(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < PD_Repeat; ++i) {
    acc += (uint32_t)DueAdcF.ReadAnalogPin(pin);
  }
  return (uint16_t)(acc / (uint32_t)PD_Repeat);
}

// One-shot packet receiver (your existing approach, trimmed a bit)
bool rx_once_packet() {
  // 1) find first rising edge
  int last = READ_BIT();
  while (last != 0) last = READ_BIT();

  uint32_t t_start = 0, last_edge_ts = 0;
  for (;;) {
    int b = READ_BIT();
    if (b != last && b == 1) {
      uint32_t t_edge = micros();                  // timestamp NOW
      if (EDGE_CONFIRM_US) {
        delayMicroseconds(EDGE_CONFIRM_US);
        if (READ_BIT() != 1) { last = b; continue; }
      }
      last = 1;
      t_start = t_edge;                             // unbiased
      break;
    }
    last = b;
  }

  // === consume the rest of the preamble, keep unbiased time of each confirmed edge ===
  const uint32_t EDGES_TO_MEASURE = (uint32_t)(PREFIX_COUNT * 8u) - 1u;
  last_edge_ts = t_start;
  for (uint16_t i = 0; i < EDGES_TO_MEASURE; ++i) {
    int b;
    do { b = READ_BIT(); } while (b == last);
    uint32_t edge_ts = micros();                   // timestamp NOW
    if (EDGE_CONFIRM_US) {
      delayMicroseconds(EDGE_CONFIRM_US);
      if (READ_BIT() == last) { --i; continue; }   // glitch; retry same edge
    }
    last = b;
    last_edge_ts = edge_ts;                        // unbiased
  }
  uint32_t t_end   = last_edge_ts;
  uint32_t span_us = (t_end - t_start);

  // ===== 3) Recover clock: T = span / 31, sample first data bit at t_end + 1.5T =====
  // Guard against zero/div rounding
  uint32_t bit_us = (span_us + EDGES_TO_MEASURE / 2) / EDGES_TO_MEASURE;
  uint32_t half_us = bit_us / 2u;

  // First sample at center of the first length bit:
  // We ended exactly at start of the last '0' bit of preamble; next bit starts at t_end + T,
  // its midpoint is t_end + 1.5T.
  uint32_t t_sample = t_end + bit_us + half_us;

  // ===== 3) Read TOTAL length byte (LSB-first) =====
  // totalLenBytes = PREFIX_COUNT + 1 + payloadLen
  uint32_t payloadLen = 0;                           // 32-bit to be safe during shifts
  const uint8_t lenWireBits = (uint8_t)(LEN_BYTES * 8);

  for (uint8_t bit = 0; bit < lenWireBits; ++bit) {
    while ((int32_t)(micros() - t_sample) < 0) { /* wait until sample time */ }
    int b = READ_BIT();
    if (b) payloadLen |= (1u << bit);              // LSB-first bit order
    t_sample += bit_us;
  }

  // Sanity check against MAX_INPUT
  if (payloadLen > MAX_INPUT) {
    Serial.print(F("Bad payloadLen: ")); Serial.println((unsigned)payloadLen);
    return false; // resync/drop
  }

  // ===== 4) Read payload bytes (LSB-first); no max, allocate exact =====
  char* payload = (char*)malloc((size_t)payloadLen + 1);
  if (!payload) {
    Serial.println(F("Alloc failed; dropping packet."));
    // Drain bits to keep cadence (optional)
    return false;
  }

  for (int k = 0; k < payloadLen; k++) {
    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; i++) {
      while ((int32_t)(micros() - t_sample) < 0) { /* wait */ }
      int b = READ_BIT();
      if (b) v |= (1u << i);              // LSB-first
      t_sample += bit_us;
    }
    payload[k] = (char)v;
  }
  payload[payloadLen] = '\0';

  // ===== 5) Print recovered payload =====
  Serial.print(F("Clock="));
  Serial.print(bit_us);
  Serial.print(F("  |PayloadLen="));
  Serial.print(payloadLen);
  Serial.print(F(" | Msg: "));
  Serial.println(payload);

  // Optional hex dump
  Serial.print(F("Hex: "));
  for (int i = 0; i < payloadLen; i++) {
    uint8_t v = (uint8_t)payload[i];
    if (v < 16) Serial.print('0');
    Serial.print(v, HEX);
    Serial.print(' ');
  }
  Serial.println();

  free(payload);
  return true;
}

// =================== Setup / Loop ===================
void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(SerialTimeOut);

  DueAdcF.EnablePin(PD1_PIN);
  DueAdcF.EnablePin(PD2_PIN);
  DueAdcF.EnablePin(PD3_PIN);
  DueAdcF.EnablePin(PD4_PIN);
  DueAdcF.Start1Mhz();
  Serial.println("Ready.");
}

void ADC_Handler() {
  DueAdcF.adcHandler();
}

void stream_pd_loop() {
  // Stream until a STOP line is seen
  for (;;) {
    if (Serial.available()) {
      String s = Serial.readStringUntil('\n'); s.trim();
      if (s == "STREAM_STOP") { Serial.println("Done"); return; }
      // ignore unknown
    }

    // average PD1..PD4
    uint32_t a1=0,a2=0,a3=0,a4=0;
    for (int i = 0; i < PD_Repeat; ++i) {
      a1 += (uint32_t)DueAdcF.ReadAnalogPin(PD1_PIN);
      a2 += (uint32_t)DueAdcF.ReadAnalogPin(PD2_PIN);
      a3 += (uint32_t)DueAdcF.ReadAnalogPin(PD3_PIN);
      a4 += (uint32_t)DueAdcF.ReadAnalogPin(PD4_PIN);
    }
    uint16_t s1 = (uint16_t)(a1 / (uint32_t)PD_Repeat);
    uint16_t s2 = (uint16_t)(a2 / (uint32_t)PD_Repeat);
    uint16_t s3 = (uint16_t)(a3 / (uint32_t)PD_Repeat);
    uint16_t s4 = (uint16_t)(a4 / (uint32_t)PD_Repeat);

    // CSV line
    char line[32];  // enough for "##### ##### ##### #####\r\n"
    snprintf(line, sizeof(line), "%5u %5u %5u %5u\r\n", s1, s2, s3, s4);
    Serial.print(line);

    if (Delay_ms) delay(Delay_ms);
    yield();
  }
}

void decode_mode_forever() {
  // No Serial.available() checks here by design (tight timing).
  // Close/reopen the port from the host to reset the board and exit.
  for (;;) {
    (void)rx_once_packet();
    // Optional: add a small guard if you want a breather
    // yield();  // harmless on many systems
  }
}

void loop() {
  String cmd = waitForLine();

  if (cmd == "STREAM_START") {
    Serial.println("ACK");
    Serial.println(F(" PD1  PD2  PD3  PD4"));
    stream_pd_loop();
  }
  else if (cmd == "DECODE_MODE") {
    Serial.println("ACK");
    decode_mode_forever();   // never returns (until the board is reset)
  }
  else if (cmd == "MEASURE_PD1") {
    uint16_t v = read_pd_channel(PD1_PIN);
    Serial.print("PD1 ");
    Serial.println(v);
  }
  else if (cmd == "MEASURE_PD2") {
    uint16_t v = read_pd_channel(PD2_PIN);
    Serial.print("PD2 ");
    Serial.println(v);
  }
  else if (cmd == "MEASURE_PD3") {
    uint16_t v = read_pd_channel(PD3_PIN);
    Serial.print("PD3 ");
    Serial.println(v);
  }
  else if (cmd == "MEASURE_PD4") {
    uint16_t v = read_pd_channel(PD4_PIN);
    Serial.print("PD4 ");
    Serial.println(v);
  }
  else if (cmd.startsWith("SET_STREAM")) {
    // Optional: "SET_STREAM,<repeat>,<delay_ms>"
    int comma1 = cmd.indexOf(',');
    int comma2 = cmd.indexOf(',', comma1+1);
    if (comma1 > 0 && comma2 > comma1) {
      int rep = cmd.substring(comma1+1, comma2).toInt();
      int dly = cmd.substring(comma2+1).toInt();
      if (rep < 1) rep = 1; if (rep > 1000) rep = 1000;
      if (dly < 0) dly = 0; if (dly > 10000) dly = 10000;
      PD_Repeat = rep; Delay_ms = (uint16_t)dly;
      Serial.println("Done");
    } else {
      Serial.println("Bad SET_STREAM");
    }
  }
  else {
    // Unknown; ignore or print debug
    // Serial.print("Unknown: "); Serial.println(cmd);
  }
}