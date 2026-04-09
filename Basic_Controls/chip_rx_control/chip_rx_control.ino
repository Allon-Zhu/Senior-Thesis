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
// #define THRESH 300
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

uint16_t read_pd_channel(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < PD_Repeat; ++i) {
    acc += (uint32_t)DueAdcF.ReadAnalogPin(pin);
  }
  return (uint16_t)(acc / (uint32_t)PD_Repeat);
}

// One-shot packet receiver (your existing approach, trimmed a bit)
bool rx_once_packet() {
  // -----------------------------
  // State 1) HOLDING
  // -----------------------------
  // Wait until the line is HIGH (first '1' of the first reversed 'U' = 10101010).
HOLDING_RESTART:
  // -----------------------------
  // State 1) HOLDING
  // -----------------------------
  while (READ_BIT() == 0) { /* spin */ }
  if (EDGE_CONFIRM_US) {
    delayMicroseconds(EDGE_CONFIRM_US);
    if (READ_BIT() == 0) goto HOLDING_RESTART;   // glitch
  }

  int last = 1;

  // -----------------------------
  // State 2) CLOCK RECOVERY
  // -----------------------------
  const uint16_t FALLS_NEEDED = (uint16_t)(4u * (uint16_t)PREFIX_COUNT);

  // Debug: record all falling edges
  uint32_t fall_ts[ (uint16_t)(4u * (uint16_t)PREFIX_COUNT) ];  // VLA if PREFIX_COUNT not const
  // If your compiler doesn't allow this, replace with: static uint32_t fall_ts[MAX_PREFIX_FALLS];

  uint16_t fall_count = 0;

  while (fall_count < FALLS_NEEDED) {
    int b = READ_BIT();
    if (b == last) continue;

    uint32_t t_edge = micros();

    if (EDGE_CONFIRM_US) {
      delayMicroseconds(EDGE_CONFIRM_US);
      if (READ_BIT() != b) {
        continue; // glitch; ignore
      }
    }

    if (last == 1 && b == 0) {
      if (fall_count < FALLS_NEEDED) {
        fall_ts[fall_count] = t_edge;
      }
      ++fall_count;
    }

    last = b;
  }

  // Recover bit time
  const uint16_t denom = (uint16_t)(8u * (uint16_t)PREFIX_COUNT - 2u);
  if (denom == 0) {
    Serial.println(F("Bad PREFIX_COUNT for clock recovery."));
    return false;
  }
  uint32_t bit_us  = (uint32_t)(fall_ts[FALLS_NEEDED - 1] - fall_ts[0]) / (uint32_t)denom;

  // Sample starts after last falling edge: next bit midpoint is +1.5T
  uint32_t t_sample = (uint32_t)fall_ts[FALLS_NEEDED - 1] + 7*bit_us/4;

  // -----------------------------
  // State 3) MESSAGE LENGTH RECOVERY (same as before)
  // -----------------------------
  uint32_t payloadLen = 0;
  const uint8_t lenWireBits = (uint8_t)(LEN_BYTES * 8);

  for (uint8_t bit = 0; bit < lenWireBits; ++bit) {
    while ((uint32_t)micros() < t_sample) { /* spin */ }
    int b = READ_BIT();
    if (b) payloadLen |= (1u << bit);   // LSB-first on wire
    t_sample += bit_us;
  }

  if (payloadLen > MAX_INPUT) {
    Serial.print(F("Bad payloadLen: "));
    Serial.println((unsigned)payloadLen);
    Serial.print(F("Falling edges ("));
    Serial.print(FALLS_NEEDED);
    Serial.println(F("):"));
    for (uint16_t i = 0; i < FALLS_NEEDED; ++i) {
      Serial.print(i);
      Serial.print(F(": "));
      Serial.println(fall_ts[i]);
    }
    return false;
  }

  // -----------------------------
  // State 4) MESSAGE DECODING (same as before)
  // -----------------------------
  char* payload = (char*)malloc((size_t)payloadLen + 1);
  if (!payload) {
    Serial.println(F("Alloc failed; dropping packet."));
    return false;
  }

  for (uint32_t k = 0; k < payloadLen; ++k) {
    uint8_t v = 0;
    for (uint8_t i = 0; i < 8; ++i) {
      while ((uint32_t)micros() < t_sample) { /* spin */ }
      int b = READ_BIT();
      if (b) v |= (1u << i);            // LSB-first
      t_sample += bit_us;
    }
    payload[k] = (char)v;
  }
  payload[payloadLen] = '\0';

  Serial.print(F("Clock="));
  Serial.print(bit_us);
  Serial.print(F("  |PayloadLen="));
  Serial.print((unsigned)payloadLen);
  Serial.print(F(" | Msg: "));
  Serial.println(payload);

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
  Serial.println("RX Ready.");
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
