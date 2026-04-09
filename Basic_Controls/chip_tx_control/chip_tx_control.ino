#include <math.h>
#include "modem.h"
#include "control.h"
#include "Arduino.h"
#include <stdlib.h>
struct ScanAxis;

/*

TxController_v10, features:
1. Voltage control for Tx by multiple DACs
2. Interconnection with python controller
*/

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
#pragma region Global Parameters
const float pi = 3.1415926;

// DAC parameters
#define DAC0_CS 13
#define DAC1_CS 24
#define DAC0_RST 28
#define DAC1_RST 26
#define DAC_LDAC 12
#define SYNC 10
const int SPI_SPEED = 21000000;

DACX1416* dac0 = nullptr;
DACX1416* dac1 = nullptr;

const int OUT_MAX = 65535;
const uint8_t ALL_RAN = 40;   // DAC range U_40 (0..40 V logical)

const float V_MIN = 0.0f;
const float V_MAX = 30.0f;    // sanity clamp, even if DAC range is 40 V
#pragma endregion

#pragma region Tx Parameters
float Tx_MUXdH = 10.21;
float Tx_MUXdL = 12.21;

// Data transfer test
int Tx_CDR_num = 20;
static uint16_t DataBufferSize = 50;
char* Data = new char[DataBufferSize];
bool* DataTx = new bool[DataBufferSize * 8];
int DataLength = 0;
int Data_idx = 0;
int Tx_idx = 0;
float Tx_V = 0.0;
// int Tx_trigger = 0;

unsigned long Tx_t1 = 0;
unsigned long Tx_t2 = 0;

int* CDR_time = new int[Tx_CDR_num];
int CDR_idx = 0;
unsigned long* Data_time = new unsigned long[32];
int Data_Tidx = 0;
#pragma endregion

#pragma region Rx Parameters
const uint16_t Rx_DelayToTx = 20 + 45;
DueAdcFast DueAdcF(1024);
const int RxTop_PIN = A1;
const int RxBot_PIN = A0;
const int PD1_PIN = A0;
const int PD2_PIN = A1;
const int PD3_PIN = A2;
const int PD4_PIN = A3;
const int TEENSY_PIN = 53;
const int PD_Repeat = 4;
int PD_DWDM = 2;
float PD_DWDM_ratio = 0.31;
uint16_t PD_Calib_data[4] = {2230, 2595, 2344, 2067};
float PD_Calib_ratio[4] = {0.0, 0.0, 0.0, 0.0};
int RxTop_Signal = 0;
int RxBot_Signal = 0;
#pragma endregion

#pragma region QWN test Parameters
int QWN_period_us = 1000;
int QWN_RPC_period = 250;
int QWN_dir = 0;
static uint16_t QwnHeaderSize = 28;
char* QWN_Header = new char[DataBufferSize];
uint16_t QWN_RPC_delay_us = 90;
const int ORI_PIN1 = 22;
const int ORI_PIN2 = 24;
#pragma endregion

#pragma region Loop Parameters
bool loop_On = 0;
bool QF4_sweep_On = 0;

int step = -1;

// int* Monitor_data = new int[200];
unsigned long* Monitor_data = new unsigned long[200];
int Monitor_length = 0;
int Monitor_idx = 0;
unsigned long tic = 0;

#pragma endregion Loop Parameters


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
  while (!Serial.available()) {
    // blocking wait
    if (SerialUSB.available()) {
      return SerialUSB.readStringUntil('\n');
    }
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
  Tx_MUXdL = newVL;
  Tx_MUXdH = newVH;

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

/*
Helpers for Scan 2V
*/ 

struct ScanAxis {
  uint8_t chip;
  uint8_t pin;
  float v_init;
  float v_start;
  float v_stop;
  uint16_t N;
};

static bool parse_axis_line(const String& line, ScanAxis& ax) {
  int c, p;
  float vi, vs, ve;
  int n;
  int ok = sscanf(line.c_str(), "%d %d %f %f %f %d", &c, &p, &vi, &vs, &ve, &n);
  if (ok != 6) return false;

  ax.chip    = (uint8_t)c;
  ax.pin     = (uint8_t)p;
  ax.v_init  = clampV(vi);
  ax.v_start = clampV(vs);
  ax.v_stop  = clampV(ve);

  if (n < 2) return false;
  ax.N = (uint16_t)n;
  return true;
}

static inline float axis_value_vsq(const ScanAxis& ax, uint16_t k) {
  // k in [0, N-1]
  float s = clampV(ax.v_start);
  float e = clampV(ax.v_stop);

  // uniform in u = V^2
  float s2 = s * s;
  float e2 = e * e;

  float t = (ax.N <= 1) ? 0.0f : ((float)k / (float)(ax.N - 1));
  float u = s2 + t * (e2 - s2);

  if (u < 0.0f) u = 0.0f;         // should not happen after clamp, but safe
  float v = sqrtf(u);
  return clampV(v);
}

static inline int pd_index_to_pin(uint8_t pd_idx) {
  switch (pd_idx) {
    case 1: return PD1_PIN;  // A0
    case 2: return PD2_PIN;  // A1
    case 3: return PD3_PIN;  // A2
    case 4: return PD4_PIN;  // A3
    default: return PD1_PIN; // fallback
  }
}

static inline uint16_t read_pd(int pin) {
  // return (uint16_t)analogRead(pin);
  return (uint16_t)DueAdcF.ReadAnalogPin(pin);
}

// ========================== UPDATE_VOLTAGES ==========================

bool set_dac_voltage(uint8_t chip_id, uint8_t pin, float volt) {
  if(chip_id == 0){
    dac0 -> set_out(pin, v2code(volt));
    return true;
  }
  else if(chip_id == 1){
    dac1 -> set_out(pin, v2code(volt));
    return true;
  }
  else{
    return false;
  }
}

void update_voltages() {
  // Read floats in the exact order Python sends them
  delay(200);
  Serial.println("ACK");
  
  while (true) {
    String line = waitForSerialCommand();  // blocks until a line comes

    if (line.length() == 0) {
      // ignore empty lines
      continue;
    }

    line.trim();

    // Check for END of bulk update
    if (line.equals("END")) {
      break;
    }

    uint8_t chip_id, pin;
    float volt;
    int parsed = sscanf(line.c_str(), "%hhu %hhu %f", &chip_id, &pin, &volt);
    volt = clampV(volt);

    // Apply to DAC
    if (!set_dac_voltage(chip_id, pin, volt)) {
      Serial.println(F("ERR bad chip"));
    }
  }

  dac0 -> sync(1);                 // <-- always sync after an update
  delayMicroseconds(50);

  // Done with all updates
  Serial.println("Done.");
}

// ========================== SCAN_VOLTAGES ==========================

void scan_2v_store_and_return() {
  delay(50);
  Serial.println("ACK");

  // --- read 2 axes + pd pins + END ---
  ScanAxis a1, a2;
  int pd1_pin = A0, pd2_pin = A1;

  String line;

  line = waitForSerialCommand();
  if (!parse_axis_line(line, a1)) { Serial.println("ERR axis1"); return; }

  line = waitForSerialCommand();
  if (!parse_axis_line(line, a2)) { Serial.println("ERR axis2"); return; }

  line = waitForSerialCommand();
  int i1, i2;
  if (sscanf(line.c_str(), "%d %d", &i1, &i2) != 2) { Serial.println("ERR pds"); return; }
  if (i1 < 1 || i1 > 4 || i2 < 1 || i2 > 4) { Serial.println("ERR pds"); return; }

  pd1_pin = pd_index_to_pin((uint8_t)i1);
  pd2_pin = pd_index_to_pin((uint8_t)i2);

  line = waitForSerialCommand();
  if (!line.equals("END")) { Serial.println("ERR no END"); return; }

  Serial.println("ACK");

  uint16_t n1 = a1.N;
  uint16_t n2 = a2.N;
  uint32_t N = (uint32_t)n1 * (uint32_t)n2;

  // allocate arrays (PD only)
  uint16_t* pd1 = (uint16_t*)malloc(N * sizeof(uint16_t));
  uint16_t* pd2 = (uint16_t*)malloc(N * sizeof(uint16_t));
  if (!pd1 || !pd2) {
    if (pd1) free(pd1);
    if (pd2) free(pd2);
    Serial.println("ERR alloc");
    return;
  }

  // --- set initial voltages ---
  set_dac_voltage(a1.chip, a1.pin, a1.v_init);
  set_dac_voltage(a2.chip, a2.pin, a2.v_init);
  dac0->sync(1);
  dac1->sync(1);
  delayMicroseconds(50);

  // --- scan & store ---
  const uint16_t settle_us = 100;   // your requirement
  uint32_t idx = 0;

  for (uint16_t i = 0; i < n1; i++) {
    float v1 = axis_value_vsq(a1, i);
    set_dac_voltage(a1.chip, a1.pin, v1);

    for (uint16_t j = 0; j < n2; j++) {
      float v2 = axis_value_vsq(a2, j);
      set_dac_voltage(a2.chip, a2.pin, v2);

      // latch updates
      dac0->sync(1);
      dac1->sync(1);

      delayMicroseconds(settle_us);

      pd1[idx] = read_pd(pd1_pin);
      pd2[idx] = read_pd(pd2_pin);
      idx++;
    }
  }

  // --- restore initial ---
  set_dac_voltage(a1.chip, a1.pin, a1.v_init);
  set_dac_voltage(a2.chip, a2.pin, a2.v_init);
  dac0->sync(1);
  dac1->sync(1);
  delayMicroseconds(50);

  Serial.println("Scan2V Finished");

  // --- return results ---
  Serial.print("BEGIN ");
  Serial.print(n1);
  Serial.print(" ");
  Serial.println(n2);

  Serial.write((uint8_t*)pd1, N * sizeof(uint16_t));
  Serial.write((uint8_t*)pd2, N * sizeof(uint16_t));

  Serial.println("DONE");

  free(pd1);
  free(pd2);
}

// ------ QDCP Helpers ------

static size_t read_line_from_serialusb(char *buf, size_t max_len) {
  if (max_len == 0) return 0;
  size_t idx = 0;
  unsigned long t0 = millis();

  while (millis() - t0 < 5000) {
    while (SerialUSB.available()) {
      char c = (char)SerialUSB.read();
      if (c == '\r') continue;
      if (c == '\n') {
        buf[idx] = '\0';
        return idx;
      }
      if (idx < max_len - 1) {
        buf[idx++] = c;
      }
    }
  }

  buf[idx] = '\0';
  return idx;
}

static bool read_exact_serialusb(uint8_t *buf, size_t nbytes, unsigned long timeout_ms) {
  size_t got = 0;
  unsigned long t0 = millis();

  while (got < nbytes && (millis() - t0 < timeout_ms)) {
    while (SerialUSB.available() && got < nbytes) {
      buf[got++] = (uint8_t)SerialUSB.read();
    }
  }

  return got == nbytes;
}

static void print_hex_serialusb(const uint8_t *data, size_t n) {
  SerialUSB.print("QDCP_HEX ");
  for (size_t i = 0; i < n; ++i) {
    if (data[i] < 16) SerialUSB.print('0');
    SerialUSB.print(data[i], HEX);
    if (i + 1 < n) SerialUSB.print(' ');
  }
  SerialUSB.println();
}

void handle_qdcp_packet_serialusb() {
  SerialUSB.println("READY_FOR_QDCP");

  char lenbuf[24];
  size_t len_n = read_line_from_serialusb(lenbuf, sizeof(lenbuf));
  if (len_n == 0) {
    SerialUSB.println("ERR_NO_LENGTH");
    return;
  }

  long packet_len = atol(lenbuf);
  if (packet_len <= 0 || packet_len > MAX_INPUT) {
    SerialUSB.println("ERR_BAD_LENGTH");
    return;
  }

  SerialUSB.println("SEND_QDCP_BYTES");

  uint8_t *packet = (uint8_t *)malloc((size_t)packet_len);
  if (!packet) {
    SerialUSB.println("ERR_ALLOC");
    return;
  }

  bool ok = read_exact_serialusb(packet, (size_t)packet_len, 5000);
  if (!ok) {
    free(packet);
    SerialUSB.println("ERR_READ_TIMEOUT");
    return;
  }

  SerialUSB.println("QDCP_PACKET_RECEIVED");
  print_hex_serialusb(packet, (size_t)packet_len);
  SerialUSB.println("QDCP_DONE");

  free(packet);
}

// ------ setup & loop ------

void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(SerialTimeOut);
  SerialUSB.begin(BaudRate);
  SerialUSB.setTimeout(SerialTimeOut);

  #pragma region DAC preset
  dac0 = new DACX1416(DAC0_CS, DAC0_RST, DAC_LDAC, &SPI, SPI_SPEED);
  dac1 = new DACX1416(DAC1_CS, DAC1_RST, DAC_LDAC, &SPI, SPI_SPEED);

  dac0 -> read_reg(R_DEVICEID);
  dac1 -> read_reg(R_DEVICEID);
  int res0 = dac0 -> init();
  int res1 = dac1 -> init();
  dac0 -> set_int_reference(false);
  dac1 -> set_int_reference(false);

  for(int i=0; i<16; i++){
    dac0 -> set_ch_enabled(i, true);
    dac0 -> set_range(i, DACX1416::U_40);
    dac0 -> set_ch_sync(i, true);
    dac1 -> set_ch_enabled(i, true);
    dac1 -> set_range(i, DACX1416::U_40);
    dac1 -> set_ch_sync(i, true);
  }

  Serial.println("TX Ready.");

  #pragma endregion

  #pragma region ADC preset
  DueAdcF.EnablePin(A0);
  DueAdcF.EnablePin(A1);
  DueAdcF.EnablePin(A2);
  DueAdcF.EnablePin(A3);
  DueAdcF.Start1Mhz(); 

  // // Calculate PD calib ratios
  // for(int ipd = 0; ipd < 4; ipd++){ 
  //   PD_Calib_ratio[ipd] = float(PD_Calib_data[0]) / float(PD_Calib_data[ipd]);
  //   if(ipd == PD_DWDM - 1) PD_Calib_ratio[ipd] = PD_Calib_ratio[ipd] * PD_DWDM_ratio;
  // }

  #pragma endregion

}


void ADC_Handler() {
  DueAdcF.adcHandler();
}


void loop() {
  String command = waitForSerialCommand();

  if (command == "UPDATE_VOLTAGES") {
    update_voltages();
  }

  else if (command == "SET_TX_LEVELS") {
    set_tx_levels();
  }

  else if (command == "TX_SHUT_DOWN") {
    for(int i=0; i<16; i++){
      dac0 -> set_out(i, 0);
      dac1 -> set_out(i, 0);
    }

    dac0 -> sync(1);
    delay(200);
  }

  else if (command == "QDCP_PACKET") {
    handle_qdcp_packet_serialusb();
  }

  else if (command == "SEND_MESSAGE_d") {
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
          Tx_V = (packetBin[Data_idx] == 0) ? Tx_MUXdL : Tx_MUXdH;
          Data_idx++;
          // dac1->set_out(MUXd_PIN, v2code(Tx_V));
          dac1->sync(1);              // latch every bit transition
          Tx_idx++;
          delayMicroseconds(30);
        } else {
          // Done: set to idle level and break
          // dac1->set_out(MUXd_PIN, v2code(Tx_MUXdL));
          dac1->sync(1);
          break;
        }
      }
    }

    free(packetBin);
    free(packet);
    Serial.println("Done");
  }

  else if (command == "SCAN_2V") {
    scan_2v_store_and_return();
  }

}
