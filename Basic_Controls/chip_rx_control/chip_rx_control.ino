#include <Arduino.h>
#include "dacx1416.h"
#include <math.h>
#include <DueAdcFast.h>
#include "modem.h"

/*

RxController_v22

Logical control for Rx
CDR Triggers by falling edges
Polarization compensation included
3 photon detectors only

*/


#define BaudRate 115200
DueAdcFast DueAdcF(1024);
const int PD_Repeat = 5;
uint16_t S1 = 0;
uint16_t S2 = 0;
uint16_t S3 = 0;
uint16_t S4 = 0;
uint16_t Delay_ms = 100;


const int PD1_PIN = A0;
const int PD2_PIN = A1;
const int PD3_PIN = A2;
const int PD4_PIN = A3;

uint8_t mode = 0;


#define PREAMBLE_EDGE_TIMEOUT_US     200000UL // per-edge wait cap during preamble (200 ms)
#define EDGE_CONFIRM_US              2       // microseconds to re-check level when an edge is seen
#define TRIM_PERCENT                 20 
#define THRESH            500     // threshold for HIGH/LOW (volts or ADC-converted units)
#define PREFIX_COUNT      5         // number of leading 'U' (0x55) bytes in preamble
#define MAX_INPUT 2048

constexpr uint8_t calc_len_bytes_u32(uint32_t x) {
  return (x <= 0xFFu)       ? 1 :
         (x <= 0xFFFFu)     ? 2 :
         (x <= 0xFFFFFFu)   ? 3 :
                              4;
}
constexpr uint8_t LEN_BYTES = calc_len_bytes_u32(MAX_INPUT);

// Inline macro for bit read (no functions as requested)
#define READ_BIT()  ((DueAdcF.ReadAnalogPin(PD1_PIN) > THRESH) ? 1 : 0)


void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(50);

  DueAdcF.EnablePin(A0);
  DueAdcF.EnablePin(A1);
  DueAdcF.EnablePin(A2);
  DueAdcF.EnablePin(A3);
  DueAdcF.Start1Mhz(); 
}


void ADC_Handler() {
  DueAdcF.adcHandler();
}


void loop() {
  if(Serial.available() > 0){
    mode = Serial.read() - '0';
  }
  else if(mode == int('z' - '0')){
    while(1){
      S1 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S1 += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN));
      }
      S1 = uint16_t(S1/PD_Repeat);

      Serial.println(S1);
      delay(Delay_ms);
    }
  }
  else if(mode == int('p' - '0')) {
    // === find first rising edge ===
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
      return; // resync/drop
    }

    // ===== 4) Read payload bytes (LSB-first); no max, allocate exact =====
    char* payload = (char*)malloc((size_t)payloadLen + 1);
    if (!payload) {
      Serial.println(F("Alloc failed; dropping packet."));
      // Drain bits to keep cadence (optional)
      return;
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
  }
}
