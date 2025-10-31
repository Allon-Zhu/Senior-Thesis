#include <math.h>
#include "modem.h"
#include "control.h"
#include "Arduino.h"
#include <stdlib.h>

/*

TxController_v42, features:
1. Voltage control for Tx
2. CR Triggers by falling edges ("UUUUU" represents CR header)
3. Polarization Compensation included, through GPC+RPC method, for both "maximization" and "stabliziation" modes
4. Ring resonances alignment helper
5. RPS improved to provide better stability
6. RPS classical monitor added
7. Full header transmitter integration, 2x2 and 2x2x3 configurations

Special note: only one ring is utilized to ease research

*/

#pragma region Global Parameters
int BaudRate = 115200;
int SerialTimeOut = 200;
const float pi = 3.1415926;
const uint16_t Tx_period_us = 100;
// const uint16_t Tx_period_us = 50;
int Serial_Input = 0;

unsigned long t1 = 0;
unsigned long t2 = 0;

#define MAX_INPUT 2048
#define PREFIX_COUNT 4
#define PREFIX_BYTE 'U'

// DAC parameters
#define DAC_CS 13
#define DAC_RST 28
#define DAC_LDAC 12
#define SYNC 10
const int SPI_SPEED = 21000000;
DACX1416* dac;
const int OUT_MAX = 65535;
const float V_MIN = 0.0;
const float V_MAX_mzi = 30.0;
const float V_MAX_qf = 30.0;
const float V_MAX_eps = 27.0;
const uint8_t ALL_RAN = 40;

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

// Sweep parameters
const int N_sweep_mzi = 60;
const int N_sweep_qf = 200;
const int N_sweep_eps = 200;
// const int N_sweep_qf = 100;
// const int N_sweep_eps = 100;
int idx = 0;
float V_sweep_mzi[N_sweep_mzi];
float V_sweep_mzi_test[N_sweep_mzi];
float Signal_sweep_mzi[N_sweep_mzi];
float V_sweep_qf[N_sweep_qf];
float Signal_sweep_qf[N_sweep_qf];
float V_sweep_eps[N_sweep_eps];
float Signal_sweep_eps[N_sweep_eps];
float V_extreme[3];
// float V_extreme1[3];
// float V_extreme2[3];
float Signal_extreme[3];

float Signal_sweep_mzmc[N_sweep_mzi];
float Signal_sweep_mzm1[N_sweep_mzi];
float Signal_sweep_mzmq[N_sweep_mzi];
float Signal_sweep_mzmtop1[N_sweep_mzi];
float Signal_sweep_mzmtop2[N_sweep_mzi];
float Signal_sweep_pstop2[N_sweep_mzi];
float Signal_sweep_mzmbot[N_sweep_mzi];

float Signal_sweep_epstop[N_sweep_eps];
float Signal_sweep_qf2top[N_sweep_qf];
float Signal_sweep_qf3top[N_sweep_qf];

float Signal_sweep_epsbot[N_sweep_eps];
float Signal_sweep_qf2bot[N_sweep_qf];
float Signal_sweep_qf3bot[N_sweep_qf];

float Signal_sweep_qf4[N_sweep_qf];

#pragma endregion Global Parameters

#pragma region Calibration Parameters
// For Tx4L_dev4_1550band
float V_extreme_mzmtop1[3] = {8.689, 17.632, 13.899};
float V_extreme_mzmtop2[3] = {8.666, 21.315, 16.27};
float V_extreme_pstop2[3] = {0, 0, 0};
float V_extreme_mzmbot[3] = {25.104, 16.873, 10.58};
float V_extreme_mzm1[3] = {25.909, 17.778, 11.766};
float V_extreme_mzmc[3] = {27.033, 18.45, 12.05};
float V_extreme_mzmq[3] = {17.835, 26.583, 11.126};
float V_extreme_epstop[3] = {0.0, 13.26, 13.12};
float V_extreme_qf2top[3] = {26.31, 26.31, 26.31};
float V_extreme_qf3top[3] = {19.25, 19.25, 19.25};
float V_extreme_epsbot[3] = {13.93, 13.8, 13.67};
float V_extreme_qf2bot[3] = {25.96, 25.96, 25.96};
float V_extreme_qf3bot[3] = {20.43, 20.43, 20.43};
float Slope_step_qf = 2;
float step_qf = 0.07;

float V_MZMtop1 = 18.1;
float V_MZMtop2 = 21.0;
float V_MZMbot = 29.5;
float V_PStop1 = 21.0;
float V_PStop2 = 23.0;
float V_PSbot = 21.9;
float V_MZM1 = 24.0;
float V_MZMQ = 22.80;
float V_MZMC = 30.5;
// float V_QF4 = 20.6;
float V_QF4 = 21.0;

float V_EPStop = 11.25;
float V_QF2top = 27.3;
float V_QF3top = 19.3;

float V_EPSbot = 11.20;
float V_QF2bot = 26.28;
float V_QF3bot = 19.60;

const int Optimization_Max_Times = 40;
const int Optimization_Period_ms = 100;
float S_monitor_qf2top[Optimization_Max_Times];
float S_monitor_qf3top[Optimization_Max_Times];
float S_monitor_qf2bot[Optimization_Max_Times];
float S_monitor_qf3bot[Optimization_Max_Times];

// float ResStb_V2_ref = 4.0;
// float ResStb_P_epstop = 0.001;
#pragma endregion

#pragma region Tx Parameters
float Tx_VHMZMC = 30.5;
float Tx_VLMZMC = 23.5;
// float Tx_VH = 18.6;
// float Tx_VL = 20.6;
float Tx_VH = 19.0;
float Tx_VL = 23.0;

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
#pragma endregion Rx Parameters

#pragma region GPC (Global Polarization Correction) Parameters
float V_MZMtop1_10 = 17.9;
float V_MZMtop1_11 = 22.0;

float V_MZMtop1_min = 5.0;
float V_MZMtop1_max = 27.0;
float V_MZMtop2_min = 5.0;
float V_MZMtop2_max = 31.5;
float V_PStop2_min = 5.0;
float V_PStop2_max = 31.5;
float V_PStop1_min = 5.0;
float V_PStop1_max = 31.5;
const int N_sweep_GPC = 40;
float V_MZMtop1_GPCarray[N_sweep_GPC];
float V_MZMtop2_GPCarray[N_sweep_GPC];
float V_PStop2_GPCarray[N_sweep_GPC];
float V_PStop1_GPCarray[N_sweep_GPC];
float dS12_MZMtop1_GPCarray[N_sweep_GPC];
float dS34_MZMtop1_GPCarray[N_sweep_GPC];
float S1_MZMtop2_GPCarray[N_sweep_GPC];
float S2_MZMtop2_GPCarray[N_sweep_GPC];
float dS12_MZMtop2_GPCarray[N_sweep_GPC];
float S3_PStop2_GPCarray[N_sweep_GPC];
float S4_PStop2_GPCarray[N_sweep_GPC];
float dS34_PStop2_GPCarray[N_sweep_GPC];
float S3_PStop1_GPCarray[N_sweep_GPC];
float S4_PStop1_GPCarray[N_sweep_GPC];
float dS34_PStop1_GPCarray[N_sweep_GPC];

// 2D sweep arrays
float S1_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float S2_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float dS12_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float S3_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float S4_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float dS34_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float sS12_MZMtop2_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float dS12_MZMtop1_GPC2Darray[N_sweep_GPC * N_sweep_GPC];
float dS34_MZMtop1_GPC2Darray[N_sweep_GPC * N_sweep_GPC];

// float R_state01 = 0.906;
float V_MZMtop2_med = 24.9;
float V_PStop2_med = 23.0;
float R_alpha_V2 = 0.00565;
float R_beta_V2 = 0.006;

// uint16_t GO_S = 0;
uint16_t PC_settle_delay_us = 100;

#pragma endregion

#pragma region RPC (Real-time Polarization Correction) Parameters
float Vsq_MZMtop2_min = sq(V_MZMtop2_min);
float Vsq_MZMtop2_max = sq(V_MZMtop2_max);
float Vsq_PStop2_min = sq(V_PStop2_min);
float Vsq_PStop2_max = sq(V_PStop2_max);
float Vsq_PStop1_min = sq(V_PStop1_min);
float Vsq_PStop1_max = sq(V_PStop1_max);
float Vsq_step_RPC = 4.0; // Step used to controll test measurement
float V_PStop2_RPC = 0.0;
float V_MZMtop2_RPC = 0.0;
float V_PStop1_RPC = 0.0;

const int N_sweep_RPC = 10;
float V_PStop1_RPCarray[N_sweep_RPC];
float dS12_PStop1_RPCarray[N_sweep_RPC];
float dS34_PStop1_RPCarray[N_sweep_RPC];

// Prameters for RPC maximization version
float dS12_RPC_threshold = 0.94;
float dS34_RPC_limit = 0.06;
float dS34_RPCY_threshold = 0.94;
// float dS34_RPCY_limit = 0.08;
// float dS12_RPC_threshold = 0.90;
// float dS34_RPC_limit = 0.10;
float S1_RPC = 0.0;
float S2_RPC = 0.0;
float S3_RPC = 0.0;
float S4_RPC = 0.0;
float dS12_RPC = 0.0;
float dS34_RPC = 0.0;
float dS12_RPC_prev = 0.0;
float dS34_RPC_prev = 0.0;
float dS12_RPC_output = 0.0;
float dS34_RPC_output = 0.0;
float dS12_PStop2_RPCarray[3];
float dS12_MZMtop2_RPCarray[3];
float dS34_PStop2_RPCarray[3];
float dS34_MZMtop2_RPCarray[3];
float dS12_max = 0.0;
float dS34_min = 0.0;
float dS34_max = 0.0;
float VIS34_gamma = 0.0;

float dS12_RPCY = 0.0;
float dS34_RPCY = 0.0;
float dS12_RPCY_prev = 0.0;
float dS34_RPCY_prev = 0.0;
float dS12_RPCY_output = 0.0;
float dS34_RPCY_output = 0.0;

// Parameters for RPC stabilization version
float S1_RPS = 0.0;
float S2_RPS = 0.0;
float S3_RPS = 0.0;
float S4_RPS = 0.0;
float V_PStop2_RPS = 0.0;
float V_MZMtop2_RPS = 0.0;
float V_PStop1_RPS = 0.0;
float V_PStop2_RPStmp = 0.0;
float V_MZMtop2_RPStmp = 0.0;
float V_PStop1_RPStmp = 0.0;
float dS12_RPS = 0.0;
float dS34_RPS = 0.0;
float dS12_RPSY = 0.0;
float dS34_RPSY = 0.0;
float dS12_RPS_prev = 0.0;
float dS34_RPS_prev = 0.0;
float dS12_RPSY_prev = 0.0;
float dS34_RPSY_prev = 0.0;
float dS12_RPS_output = 0.0;
float dS34_RPS_output = 0.0;
float dS12_RPSY_output = 0.0;
float dS34_RPSY_output = 0.0;
float dS12_RPS_target = 0.0;
float dS34_RPS_target = 0.0;
float dS12_RPSY_target = 0.0;
float dS34_RPSY_target = 0.0;
float dS12_RPS_threshold = 0.08;
float dS34_RPS_threshold = 0.08;
float dS12_RPSY_threshold = 0.08;
float dS34_RPSY_threshold = 0.08;
float dS12_RPS_mark = 0.0;
float dS34_RPS_mark = 0.0;
float dS12_RPSY_mark = 0.0;
float dS34_RPSY_mark = 0.0;

int d_a = random(2) * 2 - 1;
int d_b = random(2) * 2 - 1;
int d_c = random(2) * 2 - 1;
int i_a = random(3);
int i_b = random(3);
int i_c = random(3);
int n_a = 0;
int n_b = 0;
int n_c = 0;

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

float QWN_V_MZMtop2[2] = {V_MZMtop2, V_MZMtop2};
float QWN_V_PStop2[2] = {V_PStop2, V_PStop2};
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


void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(SerialTimeOut);

  #pragma region Preset output voltage candidates
  // int value = 0;
  for(idx = 0; idx < N_sweep_mzi; idx++){
    V_sweep_mzi[idx] = sqrt((sq(V_MAX_mzi) - sq(V_MIN))/(N_sweep_mzi-1)*idx + sq(V_MIN));
    V_sweep_mzi_test[idx] = 0.0;
  }
  for(idx = 0; idx < N_sweep_qf; idx++){
    V_sweep_qf[idx] = sqrt((sq(V_MAX_qf) - sq(V_MIN))/(N_sweep_qf-1)*idx + sq(V_MIN));
  }
  for(idx = 0; idx < N_sweep_eps; idx++){
    V_sweep_eps[idx] = sqrt((sq(V_MAX_eps) - sq(V_MIN))/(N_sweep_eps-1)*idx + sq(V_MIN));
  }
  for(idx = 0; idx < N_sweep_GPC; idx++){
    V_MZMtop1_GPCarray[idx] = sqrt((sq(V_MZMtop1_max) - sq(V_MZMtop1_min))/(N_sweep_GPC-1)*idx + sq(V_MZMtop1_min));
    V_MZMtop2_GPCarray[idx] = sqrt((sq(V_MZMtop2_max) - sq(V_MZMtop2_min))/(N_sweep_GPC-1)*idx + sq(V_MZMtop2_min));
    V_PStop2_GPCarray[idx] = sqrt((sq(V_PStop2_max) - sq(V_PStop2_min))/(N_sweep_GPC-1)*idx + sq(V_PStop2_min));
    V_PStop1_GPCarray[idx] = sqrt((sq(V_PStop1_max) - sq(V_PStop1_min))/(N_sweep_GPC-1)*idx + sq(V_PStop1_min));
  }
  for(idx = 0; idx < N_sweep_RPC; idx++){
    V_PStop1_RPCarray[idx] = sqrt((sq(V_PStop1_max) - sq(V_PStop1_min))/(N_sweep_RPC-1)*idx + sq(V_PStop1_min));
  }
  #pragma endregion

  #pragma region DAC preset
  pinMode(SYNC, OUTPUT);

  // Teensy Digital Output
  pinMode(TEENSY_PIN, OUTPUT);

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
  #pragma endregion

  #pragma region ADC preset
  DueAdcF.EnablePin(A0);
  // DueAdcF.EnablePin(A1);
  DueAdcF.EnablePin(A2);
  DueAdcF.EnablePin(A3);
  DueAdcF.Start1Mhz(); 

  // Calculate PD calib ratios
  for(int ipd = 0; ipd < 4; ipd++){ 
    PD_Calib_ratio[ipd] = float(PD_Calib_data[0]) / float(PD_Calib_data[ipd]);
    if(ipd == PD_DWDM - 1) PD_Calib_ratio[ipd] = PD_Calib_ratio[ipd] * PD_DWDM_ratio;
  }

  #pragma endregion

  #pragma region Prepare QWN Header
  // CR1
  QWN_Header[0] = 'U';
  QWN_Header[1] = 'U';
  QWN_Header[2] = 'U';
  QWN_Header[3] = 'U';
  QWN_Header[4] = 'U';

  // ID1
  QWN_Header[5] = '1';
  // Ori1
  QWN_Header[6] = 0x01;
  // Dest1
  QWN_Header[7] = 0x01;
  // Terminator 1
  QWN_Header[8] = 0x00;

  // CR2
  QWN_Header[9] = 'U';
  QWN_Header[10] = 'U';
  QWN_Header[11] = 'U';
  QWN_Header[12] = 'U';
  QWN_Header[13] = 'U';

  // ID2
  QWN_Header[14] = '2';
  // Ori2
  QWN_Header[15] = 0x03;
  // Dest2
  QWN_Header[16] = 0x03;
  // Terminator 2
  QWN_Header[17] = 0x00;

  // CR3
  QWN_Header[18] = 'U';
  QWN_Header[19] = 'U';
  QWN_Header[20] = 'U';
  QWN_Header[21] = 'U';
  QWN_Header[22] = 'U';

  // ID3
  QWN_Header[23] = '3';
  // Ori3
  QWN_Header[24] = 0x03;
  // Dest3
  QWN_Header[25] = 0x03;
  // Payload Duration
  QWN_Header[26] = char(20);
  // Terminator3
  QWN_Header[27] = 0x00;

  pinMode(ORI_PIN1, OUTPUT);
  pinMode(ORI_PIN2, OUTPUT);
  delayMicroseconds(300);
  digitalWrite(ORI_PIN1, HIGH);
  digitalWrite(ORI_PIN2, LOW);

  #pragma endregion
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


void ADC_Handler() {
  DueAdcF.adcHandler();
}


void loop() {
  if(Serial.available() > 0){
    step = Serial.read() - '0';
  }
  // step = 0;

  if(step == 0){
    // Turn-off the whole system
    for(idx = 0; idx < 16; idx++) dac -> set_out(idx, 0);
    dac -> sync(1);
    delay(200);
  }
  else if(step == 1){
    // Calibration for MZIs and rings
    step = -1;

    #pragma region Calibrate MZMC, MZM1, MZMtop1, and MZMtop2, based on classical transmission channel
    // Wait for the transmission being tuned to QF4 resonance
    // dac -> set_out(QF2top_PIN, int(12.0/ALL_RAN*OUT_MAX));
    // dac -> set_out(QF2bot_PIN, int(12.0/ALL_RAN*OUT_MAX));
    // dac -> set_out(MZM1_PIN, int(10.0/ALL_RAN*OUT_MAX));
    // dac -> sync(1);
    // delayMicroseconds(50);
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // MZMC sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZMC_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzmc, V_extreme_mzmc, Signal_extreme);
    delayMicroseconds(50);

    // MZM1 sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZM1_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzm1, V_extreme_mzm1, Signal_extreme);
    delayMicroseconds(50);

    // MZMtop1 sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZMtop1_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzmtop1, V_extreme_mzmtop1, Signal_extreme);
    delayMicroseconds(50);

    // MZMtop2 sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZMtop2_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzmtop2, V_extreme_mzmtop2, Signal_extreme);
    delayMicroseconds(50);

    #pragma endregion

    // Output sweep results if there is a query
    dac -> set_out(MZM1_PIN, 0);
    dac -> set_out(QF2top_PIN, 0);
    dac -> set_out(QF2bot_PIN, 0);
    dac -> sync(1);
    Serial.println("Done.");
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzmtop1, V_sweep_mzi, Signal_sweep_mzmtop1);
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzmtop2, V_sweep_mzi, Signal_sweep_mzmtop2);
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzm1, V_sweep_mzi, Signal_sweep_mzm1);
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzmc, V_sweep_mzi, Signal_sweep_mzmc);

  }
  else if(step == 2){
    step = -1;

    // Set voltages for bot branch
    dac -> set_out(MZM1_PIN, int(V_extreme_mzm1[1]/ALL_RAN*OUT_MAX));
    // dac -> set_out(MZMtop2_PIN, int(V_extreme_mzmtop2[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMtop2_PIN, int(V_extreme_mzmtop2[0]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMC_PIN, int(V_extreme_mzmc[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(QF2bot_PIN, int(14.0/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);

    #pragma region Calibrate MZMQ
    // Wait for the transmission being tuned to QF2top resonance, in order to calibrate MZMQ
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // MZMQ sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZMQ_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzmq, V_extreme_mzmq, Signal_extreme);
    delayMicroseconds(50);
    #pragma endregion

    dac -> set_out(QF2bot_PIN, 0);
    dac -> sync(1);
    Serial.println("Done.");
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzmq, V_sweep_mzi, Signal_sweep_mzmq);

  }
  else if(step == 3){
    step = -1;

    // Set voltages for top branch
    dac -> set_out(MZM1_PIN, int(V_extreme_mzm1[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMtop2_PIN, int(V_extreme_mzmtop2[0]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMC_PIN, int(V_extreme_mzmc[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMQ_PIN, int(V_extreme_mzmq[0]/ALL_RAN*OUT_MAX));
    dac -> set_out(EPStop_PIN, int(0.0/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);

    #pragma region Calibrate QF3
    // Wait for the transmission being tuned to QF3bot resonance, in order to calibrate MZMbot
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF3 sweep
    Sweep_Measure_1D(dac, &DueAdcF, MZMbot_PIN, ALL_RAN, RxBot_PIN, Rx_DelayToTx, N_sweep_mzi, V_sweep_mzi, Signal_sweep_mzmbot, V_extreme_mzmbot, Signal_extreme);
    delayMicroseconds(50);
    #pragma endregion

    dac -> set_out(EPStop_PIN, 0);
    dac -> sync(1);
    Serial.println("Done.");
    Sweep_Serial_Output_1D_query(N_sweep_mzi, V_extreme_mzmbot, V_sweep_mzi, Signal_sweep_mzmbot);

  }
  else if(step == 7){
    step = -1;

    // Tune MZM1, MZMbot, MZMQ(top branch) to the correct voltage
    dac -> set_out(MZM1_PIN, int(V_extreme_mzm1[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMtop2_PIN, int(V_extreme_mzmtop2[0]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMbot_PIN, int(V_extreme_mzmbot[0]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMC_PIN, int(V_extreme_mzmc[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMQ_PIN, int(V_extreme_mzmq[0]/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    
    #pragma region Calibrate EPS
    // Wait for the transmission being tuned to ITU channel P
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF3top calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF3top_PIN, ALL_RAN, RxBot_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf3top, V_extreme_qf3top, Signal_extreme);
    delayMicroseconds(50);
    V_QF3top = V_extreme_qf3top[0];
    dac -> set_out(QF3top_PIN, int(V_QF3top/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    uint8_t DAC_PINs_qf3top[1] = {QF3top_PIN};
    uint8_t DAC_RANs_qf3top[1] = {ALL_RAN};
    float V_init_qf3top[1] = {V_QF3top};
    float Slope_step_qf3top[1] = {Slope_step_qf};
    float step_qf3top[1] = {step_qf};
    int Direction_qf3top[1] = {1};
    float* V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf3top, DAC_RANs_qf3top, RxBot_PIN, Rx_DelayToTx, V_init_qf3top, Slope_step_qf3top, step_qf3top, Direction_qf3top, S_monitor_qf3top, Optimization_Max_Times, Optimization_Period_ms);
    V_QF3top = V_temp[0];

    // EPS calibrate
    V_init_qf3top[0] = {V_QF3top};
    Optimizer_Multi_Sweep_1D(dac, &DueAdcF, 1, DAC_PINs_qf3top, EPStop_PIN, DAC_RANs_qf3top, ALL_RAN, RxBot_PIN, Rx_DelayToTx, V_init_qf3top, N_sweep_eps, V_sweep_eps, Signal_sweep_epstop, V_extreme_epstop, Slope_step_qf3top, step_qf3top, Direction_qf3top);
    delayMicroseconds(50);
    V_EPStop = V_extreme_epstop[1];
    dac -> set_out(EPStop_PIN, int(V_EPStop/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF3top, S_monitor_qf3top);
    Sweep_Serial_Output_1D_query(N_sweep_eps, V_extreme_epstop, V_sweep_eps, Signal_sweep_epstop);

    #pragma endregion

    #pragma region Calibrate QF3top
    // Wait for the transmission being tuned to ITU channel I
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF3top calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF3top_PIN, ALL_RAN, RxBot_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf3top, V_extreme_qf3top, Signal_extreme);
    delayMicroseconds(50);
    V_QF3top = V_extreme_qf3top[0];
    dac -> set_out(QF3top_PIN, int(V_QF3top/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    V_init_qf3top[0] = {V_QF3top};
    V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf3top, DAC_RANs_qf3top, RxBot_PIN, Rx_DelayToTx, V_init_qf3top, Slope_step_qf3top, step_qf3top, Direction_qf3top, S_monitor_qf3top, Optimization_Max_Times, Optimization_Period_ms);
    V_QF3top = V_temp[0];

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF3top, S_monitor_qf3top);
    Sweep_Serial_Output_1D_query(N_sweep_qf, V_extreme_qf3top, V_sweep_qf, Signal_sweep_qf3top);
    
    #pragma endregion

    #pragma region Calibrate QF2top
    // Wait for the transmission being tuned to ITU channel S
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF2top calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF2top_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf2top, V_extreme_qf2top, Signal_extreme);
    delayMicroseconds(50);
    V_QF2top = V_extreme_qf2top[0];
    dac -> set_out(QF2top_PIN, int(V_QF2top/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    uint8_t DAC_PINs_qf2top[1] = {QF2top_PIN};
    uint8_t DAC_RANs_qf2top[1] = {ALL_RAN};
    float V_init_qf2top[1] = {V_QF2top};
    float Slope_step_qf2top[1] = {Slope_step_qf};
    float step_qf2top[1] = {step_qf};
    int Direction_qf2top[1] = {1};
    V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf2top, DAC_RANs_qf2top, RxTop_PIN, Rx_DelayToTx, V_init_qf2top, Slope_step_qf2top, step_qf2top, Direction_qf2top, S_monitor_qf2top, Optimization_Max_Times, Optimization_Period_ms);
    V_QF2top = V_temp[0];

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF2top, S_monitor_qf2top);
    Sweep_Serial_Output_1D_query(N_sweep_qf, V_extreme_qf2top, V_sweep_qf, Signal_sweep_qf2top);

    #pragma endregion

  }
  else if(step == 8){
    step = -1;

    // Tune MZM1, MZMbot, MZMQ(bot branch) to the correct voltage
    dac -> set_out(MZM1_PIN, int(V_extreme_mzm1[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMtop2_PIN, int(V_extreme_mzmtop2[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMbot_PIN, int(V_extreme_mzmbot[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMC_PIN, int(V_extreme_mzmc[1]/ALL_RAN*OUT_MAX));
    dac -> set_out(MZMQ_PIN, int(V_extreme_mzmq[1]/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    
    #pragma region Calibrate EPS
    // Wait for the transmission being tuned to ITU channel P
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF3bot calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF3bot_PIN, ALL_RAN, RxBot_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf3bot, V_extreme_qf3bot, Signal_extreme);
    delayMicroseconds(50);
    V_QF3bot = V_extreme_qf3bot[0];
    dac -> set_out(QF3bot_PIN, int(V_QF3bot/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    uint8_t DAC_PINs_qf3bot[1] = {QF3bot_PIN};
    uint8_t DAC_RANs_qf3bot[1] = {ALL_RAN};
    float V_init_qf3bot[1] = {V_QF3bot};
    float Slope_step_qf3bot[1] = {Slope_step_qf};
    float step_qf3bot[1] = {step_qf};
    int Direction_qf3bot[1] = {1};
    float* V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf3bot, DAC_RANs_qf3bot, RxBot_PIN, Rx_DelayToTx, V_init_qf3bot, Slope_step_qf3bot, step_qf3bot, Direction_qf3bot, S_monitor_qf3bot, Optimization_Max_Times, Optimization_Period_ms);
    V_QF3bot = V_temp[0];

    // EPS calibrate
    V_init_qf3bot[0] = {V_QF3bot};
    Optimizer_Multi_Sweep_1D(dac, &DueAdcF, 1, DAC_PINs_qf3bot, EPSbot_PIN, DAC_RANs_qf3bot, ALL_RAN, RxBot_PIN, Rx_DelayToTx, V_init_qf3bot, N_sweep_eps, V_sweep_eps, Signal_sweep_epsbot, V_extreme_epsbot, Slope_step_qf3bot, step_qf3bot, Direction_qf3bot);
    delayMicroseconds(50);
    V_EPSbot = V_extreme_epsbot[1];
    dac -> set_out(EPSbot_PIN, int(V_EPSbot/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF3bot, S_monitor_qf3bot);
    Sweep_Serial_Output_1D_query(N_sweep_eps, V_extreme_epsbot, V_sweep_eps, Signal_sweep_epsbot);

    #pragma endregion

    #pragma region Calibrate QF3bot
    // Wait for the transmission being tuned to ITU channel I
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF3bot calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF3bot_PIN, ALL_RAN, RxBot_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf3bot, V_extreme_qf3bot, Signal_extreme);
    delayMicroseconds(50);
    V_QF3bot = V_extreme_qf3bot[0];
    dac -> set_out(QF3bot_PIN, int(V_QF3bot/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    V_init_qf3bot[0] = {V_QF3bot};
    V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf3bot, DAC_RANs_qf3bot, RxBot_PIN, Rx_DelayToTx, V_init_qf3bot, Slope_step_qf3bot, step_qf3bot, Direction_qf3bot, S_monitor_qf3bot, Optimization_Max_Times, Optimization_Period_ms);
    V_QF3bot = V_temp[0];

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF3bot, S_monitor_qf3bot);
    Sweep_Serial_Output_1D_query(N_sweep_qf, V_extreme_qf3bot, V_sweep_qf, Signal_sweep_qf3bot);
    
    #pragma endregion

    #pragma region Calibrate QF2bot
    // Wait for the transmission being tuned to ITU channel S
    while (Serial.available() == 0) {}
    Serial.parseInt();

    // QF2bot calibrate and stablization
    Sweep_Measure_1D(dac, &DueAdcF, QF2bot_PIN, ALL_RAN, RxTop_PIN, Rx_DelayToTx, N_sweep_qf, V_sweep_qf, Signal_sweep_qf2bot, V_extreme_qf2bot, Signal_extreme);
    delayMicroseconds(50);
    V_QF2bot = V_extreme_qf2bot[0];
    dac -> set_out(QF2bot_PIN, int(V_QF2bot/ALL_RAN*OUT_MAX));
    dac -> sync(1);
    delayMicroseconds(50);
    uint8_t DAC_PINs_qf2bot[1] = {QF2bot_PIN};
    uint8_t DAC_RANs_qf2bot[1] = {ALL_RAN};
    float V_init_qf2bot[1] = {V_QF2bot};
    float Slope_step_qf2bot[1] = {Slope_step_qf};
    float step_qf2bot[1] = {step_qf};
    int Direction_qf2bot[1] = {1};
    V_temp = Optimizer_Multi(dac, &DueAdcF, 1, DAC_PINs_qf2bot, DAC_RANs_qf2bot, RxTop_PIN, Rx_DelayToTx, V_init_qf2bot, Slope_step_qf2bot, step_qf2bot, Direction_qf2bot, S_monitor_qf2bot, Optimization_Max_Times, Optimization_Period_ms);
    V_QF2bot = V_temp[0];

    Serial.println("Done.");
    TimeTrace_Serial_Output_1D_query(Optimization_Max_Times, Optimization_Period_ms, V_QF2bot, S_monitor_qf2bot);
    Sweep_Serial_Output_1D_query(N_sweep_qf, V_extreme_qf2bot, V_sweep_qf, Signal_sweep_qf2bot);

    #pragma endregion


  }
  else if(step == 9){
    step = -1;

    #pragma region Init
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
    #pragma endregion

    // Receive new voltages and apply them
    int op = -1;
    while(true){
      if(Serial.available() > 0){
        op = Serial.read() - '0';
      }

      if(op == 0){
        // Quit
        break;
      }
      else if(op == 1){
        // Voltages update
        op = -1;
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
        PD_DWDM = Serial.parseFloat();
        PD_DWDM_ratio = Serial.parseFloat();

        for(int ipd = 0; ipd < 4; ipd++){ 
          PD_Calib_ratio[ipd] = float(PD_Calib_data[0]) / float(PD_Calib_data[ipd]);
          if(ipd == int(PD_DWDM) - 1) PD_Calib_ratio[ipd] = PD_Calib_ratio[ipd] / float(PD_DWDM_ratio);
        }   

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
      else if(op == 2){
        // GPC - Step 1 - alpha & beta correction
        op = -1;
        // dac -> set_out(MZMC_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        #pragma region 2D sweeping measurement
        // Sweep_Measure_2D_3PDdiff(dac, &DueAdcF, PStop2_PIN, MZMtop2_PIN, ALL_RAN, PD1_PIN, PD_Calib_ratio[0], PD3_PIN, PD_Calib_ratio[2], PD4_PIN, PD_Calib_ratio[3], PC_settle_delay_us, N_sweep_GPC, V_PStop2_GPCarray, N_sweep_GPC, V_MZMtop2_GPCarray, dS12_MZMtop2_GPC2Darray, dS34_MZMtop2_GPC2Darray);
        Sweep_Measure_2D_3PDdiffAVG(dac, &DueAdcF, PStop2_PIN, MZMtop2_PIN, ALL_RAN, PD1_PIN, PD_Calib_ratio[0], PD3_PIN, PD_Calib_ratio[2], PD4_PIN, PD_Calib_ratio[3], PC_settle_delay_us, N_sweep_GPC, V_PStop2_GPCarray, N_sweep_GPC, V_MZMtop2_GPCarray, dS12_MZMtop2_GPC2Darray, dS34_MZMtop2_GPC2Darray, PD_Repeat);
        delayMicroseconds(PC_settle_delay_us);

        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        
        Serial.println("Finished.");
        Sweep_Serial_Output_2D_query(N_sweep_GPC, N_sweep_GPC, V_PStop2_GPCarray, V_MZMtop2_GPCarray, dS12_MZMtop2_GPC2Darray);
        Sweep_Serial_Output_2D_query(N_sweep_GPC, N_sweep_GPC, V_PStop2_GPCarray, V_MZMtop2_GPCarray, dS34_MZMtop2_GPC2Darray);
        #pragma endregion

      }
      else if(op == 3){
        // GPC - Step 2 - gamma scanning and correction
        op = -1;
        // dac -> set_out(MZMC_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        #pragma region 1D sweeping measurements - obseleted
        // MZMtop2 sweep
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_11/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        Sweep_Measure_1D_2PDdiff(dac, &DueAdcF, PStop1_PIN, ALL_RAN, PD3_PIN, PD_Calib_ratio[2], PD4_PIN, PD_Calib_ratio[3], PC_settle_delay_us, N_sweep_GPC, V_PStop1_GPCarray, dS34_PStop1_GPCarray);

        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1/ALL_RAN*OUT_MAX));
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        // Output measured values
        Serial.println("GPC.");
        Sweep_Serial_Output_1D_queryNEW(N_sweep_GPC, V_PStop1_GPCarray, dS34_PStop1_GPCarray);

        #pragma endregion

      }
      else if(op == 'c' - '0'){
        // GPC - Step 2 - gamma scanning and correction
        op = -1;

        #pragma region 1D sweeping measurements - obseleted
        // MZMtop2 sweep
        Sweep_Measure_1D_2PDdiff(dac, &DueAdcF, PStop1_PIN, ALL_RAN, PD3_PIN, PD_Calib_ratio[2], PD4_PIN, PD_Calib_ratio[3], PC_settle_delay_us, N_sweep_GPC, V_PStop1_GPCarray, dS34_PStop1_GPCarray);

        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        // Output measured values
        Serial.println("GPC.");
        Sweep_Serial_Output_1D_queryNEW(N_sweep_GPC, V_PStop1_GPCarray, dS34_PStop1_GPCarray);

        #pragma endregion

      }
      else if(op == 4){
        //Calibration for MZMtop1 according to PStop1
        op = -1;

        #pragma region 2D sweeping measurement
        Sweep_Measure_2D_3PDdiffAVG(dac, &DueAdcF, MZMtop1_PIN, PStop1_PIN, ALL_RAN, PD1_PIN, PD_Calib_ratio[0], PD3_PIN, PD_Calib_ratio[2], PD4_PIN, PD_Calib_ratio[3], PC_settle_delay_us, N_sweep_GPC, V_MZMtop1_GPCarray, N_sweep_GPC, V_PStop1_GPCarray, dS12_MZMtop1_GPC2Darray, dS34_MZMtop1_GPC2Darray, PD_Repeat);
        delayMicroseconds(PC_settle_delay_us);

        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1/ALL_RAN*OUT_MAX));
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        
        Serial.println("Finished.");
        Sweep_Serial_Output_2D_query(N_sweep_GPC, N_sweep_GPC, V_MZMtop1_GPCarray, V_PStop1_GPCarray, dS12_MZMtop1_GPC2Darray);
        Sweep_Serial_Output_2D_query(N_sweep_GPC, N_sweep_GPC, V_MZMtop1_GPCarray, V_PStop1_GPCarray, dS34_MZMtop1_GPC2Darray);
        #pragma endregion
      }
      else if(op == int('d' - '0')){
        // RPS - record target values
        op = -1;
        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat * 10; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/float(PD_Repeat * 10);
        S3_RPS = S3_RPS/float(PD_Repeat * 10);
        S4_RPS = S4_RPS/float(PD_Repeat * 10);
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_11/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat * 10; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/float(PD_Repeat * 10);
        S3_RPS = S3_RPS/float(PD_Repeat * 10);
        S4_RPS = S4_RPS/float(PD_Repeat * 10);
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPSY = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPSY = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_10/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        dS12_RPS_target = dS12_RPS;
        dS34_RPS_target = dS34_RPS;
        dS12_RPSY_target = dS12_RPSY;
        dS34_RPSY_target = dS34_RPSY;
        Serial.println("RPS.");
        Serial.println(dS12_RPS_target);
        Serial.println(dS34_RPS_target);
        Serial.println(dS12_RPSY_target);
        Serial.println(dS34_RPSY_target);
      }
      else if(op == int('e' - '0')){
        // RPS - just record four values
        op = -1;
        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/float(PD_Repeat);
        S3_RPS = S3_RPS/float(PD_Repeat);
        S4_RPS = S4_RPS/float(PD_Repeat);
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_11/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/float(PD_Repeat);
        S3_RPS = S3_RPS/float(PD_Repeat);
        S4_RPS = S4_RPS/float(PD_Repeat);
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPSY = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPSY = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_10/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);

        Serial.println("RPS.");
        Serial.println(dS12_RPS);
        Serial.println(dS34_RPS);
        Serial.println(dS12_RPSY);
        Serial.println(dS34_RPSY);
      }
      else if(op == int('f' - '0')){
        // RPS - just record intensity (PD3 = PD4)
        op = -1;
        // dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        // dac -> sync(1);
        // delayMicroseconds(PC_settle_delay_us);

        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S3_RPS = S3_RPS/float(PD_Repeat);
        S4_RPS = S4_RPS/float(PD_Repeat);

        Serial.println("RPS.");
        Serial.println(S3_RPS);
        Serial.println(S4_RPS);
      }
      else if(op == int('g' - '0')){
        // RPS - just record two values
        op = -1;
        
        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/float(PD_Repeat);
        S3_RPS = S3_RPS/float(PD_Repeat);
        S4_RPS = S4_RPS/float(PD_Repeat);
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

        Serial.println("RPS.");
        Serial.println(dS12_RPS);
        Serial.println(dS34_RPS);
      }
      else if(op == 7){
        // RPC - maximization method (for signal light)
        op = -1;

        // dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> set_out(QF2top_PIN, int((V_QF2top-8.0)/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        
        #pragma region Alpha and Beta correction
        digitalWrite(SYNC, HIGH);
        S1_RPC = 0;
        S2_RPC = 0;
        S3_RPC = 0;
        S4_RPC = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPC = S1_RPC/PD_Repeat;
        S3_RPC = S3_RPC/PD_Repeat;
        S4_RPC = S4_RPC/PD_Repeat;
        S2_RPC = (S3_RPC + S4_RPC < S1_RPC) ? 0 : S3_RPC + S4_RPC - S1_RPC;
        dS12_RPC = abs(float(S1_RPC - S2_RPC) / float(S1_RPC + S2_RPC));
        dS34_RPC = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));
        dS12_max = dS12_RPC;
        dS34_min = dS34_RPC;
        dS12_RPC_prev = dS12_RPC;
        dS34_RPC_prev = dS34_RPC;
        dS12_RPC_output = dS12_RPC;
        dS34_RPC_output = dS34_RPC;
        V_PStop2_RPStmp = V_PStop2;
        V_PStop1_RPStmp = V_PStop1;
        V_MZMtop2_RPStmp = V_MZMtop2;
        
        d_a = random(2) * 2 - 1;
        d_b = random(2) * 2 - 1;
        i_a = random(3);
        i_b = random(3);
        n_b = 0;
        n_a = 0;

        digitalWrite(SYNC, LOW);

        while(1){
          if(n_b == 3) break;

          if(!(i_a == 1 && i_b == 1)){
            // Measure
            V_PStop2_RPC = sqrt(sq(V_PStop2) + (i_a-1) * Vsq_step_RPC);
            V_MZMtop2_RPC = sqrt(sq(V_MZMtop2) + (i_b-1) * Vsq_step_RPC);
            dac -> set_out(PStop2_PIN, int(V_PStop2_RPC/ALL_RAN*OUT_MAX));
            dac -> set_out(MZMtop2_PIN, int(V_MZMtop2_RPC/ALL_RAN*OUT_MAX));
            dac -> sync(1);
            delayMicroseconds(PC_settle_delay_us);
            S1_RPC = 0;
            S2_RPC = 0;
            S3_RPC = 0;
            S4_RPC = 0;
            for(int idx=0; idx<PD_Repeat; idx++){
              S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
              S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
              S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
            }
            S1_RPC = S1_RPC/PD_Repeat;
            S3_RPC = S3_RPC/PD_Repeat;
            S4_RPC = S4_RPC/PD_Repeat;
            S2_RPC = (S3_RPC + S4_RPC < S1_RPC) ? 0 : S3_RPC + S4_RPC - S1_RPC;
            dS12_RPC = abs(float(S1_RPC - S2_RPC) / float(S1_RPC + S2_RPC));
            dS34_RPC = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));

            if(dS12_RPC_prev < dS12_RPC_threshold){
              if(dS12_RPC > dS12_max){
                dS12_max = dS12_RPC;
                V_PStop2_RPStmp = V_PStop2_RPC;
                V_MZMtop2_RPStmp = V_MZMtop2_RPC;
                dS12_RPC_output = dS12_RPC;
                dS34_RPC_output = dS34_RPC;
              }
            }
            else if(dS12_RPC_prev >= dS12_RPC_threshold && dS34_RPC_prev > dS34_RPC_limit){
              if(dS34_RPC < dS34_min){
                dS34_min = dS34_RPC;
                V_PStop2_RPStmp = V_PStop2_RPC;
                V_MZMtop2_RPStmp = V_MZMtop2_RPC;
                dS12_RPC_output = dS12_RPC;
                dS34_RPC_output = dS34_RPC;
              }
            }
          }

          i_a = (i_a + d_a + 3) % 3;
          n_a++;
          if(n_a == 3){
            n_a = 0;
            i_b = (i_b + d_b + 3) % 3;
            n_b++;
          }
        }

        V_PStop2 = V_PStop2_RPStmp;
        V_MZMtop2 = V_MZMtop2_RPStmp;
        if(V_PStop2 < V_PStop2_min) V_PStop2 = V_PStop2_min;
        if(V_PStop2 > V_PStop2_max) V_PStop2 = V_PStop2_max;
        if(V_MZMtop2 < V_MZMtop2_min) V_MZMtop2 = V_MZMtop2_min;
        if(V_MZMtop2 > V_MZMtop2_max) V_MZMtop2 = V_MZMtop2_max;
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> set_out(QF2top_PIN, int(V_QF2top/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        #pragma endregion

        #pragma region Gamma correction
        S1_RPC = 0;
        S2_RPC = 0;
        S3_RPC = 0;
        S4_RPC = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPC = S1_RPC/PD_Repeat;
        S3_RPC = S3_RPC/PD_Repeat;
        S4_RPC = S4_RPC/PD_Repeat;
        dS34_RPCY = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));
        dS34_RPCY_prev = dS34_RPCY;
        dS34_max = dS34_RPCY;
        dS34_RPCY_output = dS34_RPCY;

        // Only work on gamma if dS34_PRCY_prev doesn't meet the threshold
        if(dS34_RPCY_prev < dS34_RPCY_threshold){
          d_c = random(2) * 2 - 1;
          i_c = random(3);
          n_c = 0;

          while(1){
            if(n_c == 3) break;

            if(i_c != 1){
              // Measure
              V_PStop1_RPC = sqrt(sq(V_PStop1) + (i_c-1) * Vsq_step_RPC);
              dac -> set_out(PStop1_PIN, int(V_PStop1_RPC/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              delayMicroseconds(PC_settle_delay_us);
              S1_RPC = 0;
              S2_RPC = 0;
              S3_RPC = 0;
              S4_RPC = 0;
              for(int idx=0; idx<PD_Repeat; idx++){
                S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
              }
              S1_RPC = S1_RPC/PD_Repeat;
              S3_RPC = S3_RPC/PD_Repeat;
              S4_RPC = S4_RPC/PD_Repeat;
              S2_RPC = (S3_RPC + S4_RPC < S1_RPC) ? 0 : S3_RPC + S4_RPC - S1_RPC;
              dS34_RPCY = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));

              if(dS34_RPCY > dS34_max){
                dS34_max = dS34_RPCY;
                V_PStop1_RPStmp = V_PStop1_RPC;
                dS34_RPCY_output = dS34_RPCY;
              }
            }

            i_c = (i_c + d_c + 3) % 3;
            n_c++;
          }
        }

        V_PStop1 = V_PStop1_RPStmp;
        if(V_PStop1 < V_PStop1_min) V_PStop1 = V_PStop1_min;
        if(V_PStop1 > V_PStop1_max) V_PStop1 = V_PStop1_max;
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        // dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        #pragma endregion

        Serial.println("RPC.");
        Serial.println(V_PStop2);
        Serial.println(V_MZMtop2);
        Serial.println(V_PStop1);
        Serial.println(dS12_RPC_output);
        Serial.println(dS34_RPC_output);
        Serial.println(dS34_RPCY_output);
      } 
      else if(op == int('g' - '0')){
        // RPS - stabilization method (for pump light)
        op = -1;

        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        
        #pragma region Alpha and Beta correction
        digitalWrite(SYNC, HIGH);
        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/PD_Repeat;
        S3_RPS = S3_RPS/PD_Repeat;
        S4_RPS = S4_RPS/PD_Repeat;
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);
        dS12_RPS_mark = dS12_RPS;
        dS34_RPS_mark = dS34_RPS;
        dS12_RPS_prev = dS12_RPS;
        dS34_RPS_prev = dS34_RPS;
        dS12_RPS_output = dS12_RPS;
        dS34_RPS_output = dS34_RPS;
        V_PStop2_RPStmp = V_PStop2;
        V_PStop1_RPStmp = V_PStop1;
        V_MZMtop2_RPStmp = V_MZMtop2;
        
        d_a = random(2) * 2 - 1;
        d_b = random(2) * 2 - 1;
        i_a = random(3);
        i_b = random(3);
        n_b = 0;
        n_a = 0;

        digitalWrite(SYNC, LOW);

        while(1){
          if(n_b == 3) break;

          if(!(i_a == 1 && i_b == 1)){
            // Measure
            V_PStop2_RPS = sqrt(sq(V_PStop2) + (i_a-1) * Vsq_step_RPC);
            V_MZMtop2_RPS = sqrt(sq(V_MZMtop2) + (i_b-1) * Vsq_step_RPC);
            dac -> set_out(PStop2_PIN, int(V_PStop2_RPS/ALL_RAN*OUT_MAX));
            dac -> set_out(MZMtop2_PIN, int(V_MZMtop2_RPS/ALL_RAN*OUT_MAX));
            dac -> sync(1);
            delayMicroseconds(PC_settle_delay_us);
            S1_RPS = 0;
            S2_RPS = 0;
            S3_RPS = 0;
            S4_RPS = 0;
            for(int idx=0; idx<PD_Repeat; idx++){
              S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
              S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
              S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
            }
            S1_RPS = S1_RPS/PD_Repeat;
            S3_RPS = S3_RPS/PD_Repeat;
            S4_RPS = S4_RPS/PD_Repeat;
            S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
            dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
            dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

            if(abs(dS12_RPS_prev - dS12_RPS_target) > dS12_RPS_threshold){
              if(abs(dS12_RPS - dS12_RPS_target) < abs(dS12_RPS_mark - dS12_RPS_target)){
                dS12_RPS_mark = dS12_RPS;
                V_PStop2_RPStmp = V_PStop2_RPS;
                V_MZMtop2_RPStmp = V_MZMtop2_RPS;
                dS12_RPS_output = dS12_RPS;
                dS34_RPS_output = dS34_RPS;
              }
            }
            else if(abs(dS12_RPS_prev - dS12_RPS_target) <= dS12_RPS_threshold && abs(dS34_RPS_prev - dS34_RPS_target) > dS34_RPS_threshold){
              if(abs(dS34_RPS - dS34_RPS_target) < abs(dS34_RPS_mark - dS34_RPS_target)){
                dS34_RPS_mark = dS34_RPS;
                V_PStop2_RPStmp = V_PStop2_RPS;
                V_MZMtop2_RPStmp = V_MZMtop2_RPS;
                dS12_RPS_output = dS12_RPS;
                dS34_RPS_output = dS34_RPS;
              }
            }
          }

          i_a = (i_a + d_a + 3) % 3;
          n_a++;
          if(n_a == 3){
            n_a = 0;
            i_b = (i_b + d_b + 3) % 3;
            n_b++;
          }
        }

        V_PStop2 = V_PStop2_RPStmp;
        V_MZMtop2 = V_MZMtop2_RPStmp;
        if(V_PStop2 < V_PStop2_min) V_PStop2 = V_PStop2_min;
        if(V_PStop2 > V_PStop2_max) V_PStop2 = V_PStop2_max;
        if(V_MZMtop2 < V_MZMtop2_min) V_MZMtop2 = V_MZMtop2_min;
        if(V_MZMtop2 > V_MZMtop2_max) V_MZMtop2 = V_MZMtop2_max;
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_11/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        #pragma endregion

        #pragma region Gamma correction
        S1_RPC = 0;
        S2_RPC = 0;
        S3_RPC = 0;
        S4_RPC = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPC = S1_RPC/PD_Repeat;
        S3_RPC = S3_RPC/PD_Repeat;
        S4_RPC = S4_RPC/PD_Repeat;
        dS34_RPSY = float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC);
        dS34_RPSY_mark = dS34_RPSY;
        dS34_RPSY_prev = dS34_RPSY;
        dS34_RPSY_output = dS34_RPSY;

        // Only work on gamma if dS34_RPSY_prev doesn't meet the threshold
        if(abs(dS34_RPSY_prev - dS34_RPSY_target) > dS34_RPSY_threshold){
          d_c = random(2) * 2 - 1;
          i_c = random(3);
          n_c = 0;

          while(1){
            if(n_c == 3) break;

            if(i_c != 1){
              // Measure
              V_PStop1_RPS = sqrt(sq(V_PStop1) + (i_c-1) * Vsq_step_RPC);
              dac -> set_out(PStop1_PIN, int(V_PStop1_RPS/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              delayMicroseconds(PC_settle_delay_us);
              S1_RPC = 0;
              S2_RPC = 0;
              S3_RPC = 0;
              S4_RPC = 0;
              for(int idx=0; idx<PD_Repeat; idx++){
                S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
              }
              S1_RPC = S1_RPC/PD_Repeat;
              S3_RPC = S3_RPC/PD_Repeat;
              S4_RPC = S4_RPC/PD_Repeat;
              dS34_RPSY = float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC);

              if(abs(dS34_RPSY - dS34_RPSY_target) < abs(dS34_RPSY_mark - dS34_RPSY_target)){
                dS34_max = dS34_RPSY;
                V_PStop1_RPStmp = V_PStop1_RPS;
                dS34_RPSY_output = dS34_RPSY;
              }
            }

            i_c = (i_c + d_c + 3) % 3;
            n_c++;
          }
        }

        V_PStop1 = V_PStop1_RPStmp;
        if(V_PStop1 < V_PStop1_min) V_PStop1 = V_PStop1_min;
        if(V_PStop1 > V_PStop1_max) V_PStop1 = V_PStop1_max;
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_10/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        #pragma endregion

        Serial.println("RPS.");
        Serial.println(V_PStop2);
        Serial.println(V_MZMtop2);
        Serial.println(V_PStop1);
        Serial.println(dS12_RPS_output);
        Serial.println(dS34_RPS_output);
        Serial.println(dS34_RPSY_output);
      }
      else if(op == int('h' - '0')){
        // RPS - stabilization method (for pump light, three parameters at once)
        op = -1;

        dac -> set_out(QF4_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        
        #pragma region H input stabilization
        digitalWrite(SYNC, HIGH);
        S1_RPS = 0;
        S2_RPS = 0;
        S3_RPS = 0;
        S4_RPS = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPS = S1_RPS/PD_Repeat;
        S3_RPS = S3_RPS/PD_Repeat;
        S4_RPS = S4_RPS/PD_Repeat;
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);
        dS12_RPS_mark = dS12_RPS;
        dS34_RPS_mark = dS34_RPS;
        dS12_RPS_prev = dS12_RPS;
        dS34_RPS_prev = dS34_RPS;
        dS12_RPS_output = dS12_RPS;
        dS34_RPS_output = dS34_RPS;
        V_PStop2_RPStmp = V_PStop2;
        V_PStop1_RPStmp = V_PStop1;
        V_MZMtop2_RPStmp = V_MZMtop2;
        digitalWrite(SYNC, LOW);

        if(abs(dS12_RPS_prev - dS12_RPS_target) > dS12_RPS_threshold || abs(dS34_RPS_prev - dS34_RPS_target) > dS34_RPS_threshold){
          d_a = random(2) * 2 - 1;
          d_b = random(2) * 2 - 1;
          d_c = random(2) * 2 - 1;
          i_a = random(3);
          i_b = random(3);
          i_c = random(3);
          n_a = 0;
          n_b = 0;
          n_c = 0;

          while(1){
            if(n_c == 3) break;

            if(!(i_a == 1 && i_b == 1 && i_c == 1)){
              // Measure
              V_PStop2_RPS = sqrt(sq(V_PStop2) + (i_a-1) * Vsq_step_RPC);
              V_MZMtop2_RPS = sqrt(sq(V_MZMtop2) + (i_b-1) * Vsq_step_RPC);
              V_PStop1_RPS = sqrt(sq(V_PStop1) + (i_c-1) * Vsq_step_RPC);
              dac -> set_out(PStop2_PIN, int(V_PStop2_RPS/ALL_RAN*OUT_MAX));
              dac -> set_out(MZMtop2_PIN, int(V_MZMtop2_RPS/ALL_RAN*OUT_MAX));
              dac -> set_out(PStop1_PIN, int(V_PStop1_RPS/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              delayMicroseconds(PC_settle_delay_us);
              S1_RPS = 0;
              S2_RPS = 0;
              S3_RPS = 0;
              S4_RPS = 0;
              for(int idx=0; idx<PD_Repeat; idx++){
                S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
              }
              S1_RPS = S1_RPS/PD_Repeat;
              S3_RPS = S3_RPS/PD_Repeat;
              S4_RPS = S4_RPS/PD_Repeat;
              S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
              dS12_RPS = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
              dS34_RPS = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

              
              if(abs(dS12_RPS - dS12_RPS_target) + abs(dS34_RPS - dS34_RPS_target) < abs(dS12_RPS_mark - dS12_RPS_target) + abs(dS34_RPS_mark - dS34_RPS_target)){
                dS12_RPS_mark = dS12_RPS;
                dS34_RPS_mark = dS34_RPS;
                V_PStop2_RPStmp = V_PStop2_RPS;
                V_MZMtop2_RPStmp = V_MZMtop2_RPS;
                V_PStop1_RPStmp = V_PStop1_RPS;
                dS12_RPS_output = dS12_RPS;
                dS34_RPS_output = dS34_RPS;
              }
            }

            i_a = (i_a + d_a + 3) % 3;
            n_a++;
            if(n_a == 3){
              n_a = 0;
              i_b = (i_b + d_b + 3) % 3;
              n_b++;
              if(n_b == 3){
                n_b = 0;
                i_c = (i_c + d_c + 3) % 3;
                n_c++;
              }
            }
          }
        }

        V_PStop2 = V_PStop2_RPStmp;
        V_MZMtop2 = V_MZMtop2_RPStmp;
        V_PStop1 = V_PStop1_RPStmp;
        if(V_PStop1 < V_PStop1_min) V_PStop1 = V_PStop1_min;
        if(V_PStop1 > V_PStop1_max) V_PStop1 = V_PStop1_max;
        if(V_PStop2 < V_PStop2_min) V_PStop2 = V_PStop2_min;
        if(V_PStop2 > V_PStop2_max) V_PStop2 = V_PStop2_max;
        if(V_MZMtop2 < V_MZMtop2_min) V_MZMtop2 = V_MZMtop2_min;
        if(V_MZMtop2 > V_MZMtop2_max) V_MZMtop2 = V_MZMtop2_max;
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_11/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        delayMicroseconds(PC_settle_delay_us);
        #pragma endregion

        #pragma region D input stabilization
        S1_RPC = 0;
        S2_RPC = 0;
        S3_RPC = 0;
        S4_RPC = 0;
        for(int idx=0; idx<PD_Repeat; idx++){
          S1_RPC += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
          S3_RPC += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
          S4_RPC += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
        }
        S1_RPC = S1_RPC/PD_Repeat;
        S3_RPC = S3_RPC/PD_Repeat;
        S4_RPC = S4_RPC/PD_Repeat;
        S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
        dS12_RPSY = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
        dS34_RPSY = float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC);
        dS12_RPSY_mark = dS12_RPSY;
        dS12_RPSY_prev = dS12_RPSY;
        dS12_RPSY_output = dS12_RPSY;
        dS34_RPSY_mark = dS34_RPSY;
        dS34_RPSY_prev = dS34_RPSY;
        dS34_RPSY_output = dS34_RPSY;
        V_PStop2_RPStmp = V_PStop2;
        V_PStop1_RPStmp = V_PStop1;
        V_MZMtop2_RPStmp = V_MZMtop2;

        if(abs(dS12_RPSY_prev - dS12_RPSY_target) > dS12_RPSY_threshold || abs(dS34_RPSY_prev - dS34_RPSY_target) > dS34_RPSY_threshold){
          d_a = random(2) * 2 - 1;
          d_b = random(2) * 2 - 1;
          d_c = random(2) * 2 - 1;
          i_a = random(3);
          i_b = random(3);
          i_c = random(3);
          n_a = 0;
          n_b = 0;
          n_c = 0;

          while(1){
            if(n_c == 3) break;

            if(!(i_a == 1 && i_b == 1 && i_c == 1)){
              // Measure
              V_PStop2_RPS = sqrt(sq(V_PStop2) + (i_a-1) * Vsq_step_RPC);
              V_MZMtop2_RPS = sqrt(sq(V_MZMtop2) + (i_b-1) * Vsq_step_RPC);
              V_PStop1_RPS = sqrt(sq(V_PStop1) + (i_c-1) * Vsq_step_RPC);
              dac -> set_out(PStop2_PIN, int(V_PStop2_RPS/ALL_RAN*OUT_MAX));
              dac -> set_out(MZMtop2_PIN, int(V_MZMtop2_RPS/ALL_RAN*OUT_MAX));
              dac -> set_out(PStop1_PIN, int(V_PStop1_RPS/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              delayMicroseconds(PC_settle_delay_us);
              S1_RPS = 0;
              S2_RPS = 0;
              S3_RPS = 0;
              S4_RPS = 0;
              for(int idx=0; idx<PD_Repeat; idx++){
                S1_RPS += float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                S3_RPS += float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                S4_RPS += float(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
              }
              S1_RPS = S1_RPS/PD_Repeat;
              S3_RPS = S3_RPS/PD_Repeat;
              S4_RPS = S4_RPS/PD_Repeat;
              S2_RPS = (S3_RPS + S4_RPS < S1_RPS) ? 0 : S3_RPS + S4_RPS - S1_RPS;
              dS12_RPSY = float(S1_RPS - S2_RPS) / float(S1_RPS + S2_RPS);
              dS34_RPSY = float(S3_RPS - S4_RPS) / float(S3_RPS + S4_RPS);

              
              if(abs(dS12_RPSY - dS12_RPSY_target) + abs(dS34_RPSY - dS34_RPSY_target) < abs(dS12_RPSY_mark - dS12_RPSY_target) + abs(dS34_RPSY_mark - dS34_RPSY_target)){
                dS12_RPSY_mark = dS12_RPSY;
                dS34_RPSY_mark = dS34_RPSY;
                V_PStop2_RPStmp = V_PStop2_RPS;
                V_MZMtop2_RPStmp = V_MZMtop2_RPS;
                V_PStop1_RPStmp = V_PStop1_RPS;
                dS12_RPSY_output = dS12_RPSY;
                dS34_RPSY_output = dS34_RPSY;
              }
            }

            i_a = (i_a + d_a + 3) % 3;
            n_a++;
            if(n_a == 3){
              n_a = 0;
              i_b = (i_b + d_b + 3) % 3;
              n_b++;
              if(n_b == 3){
                n_b = 0;
                i_c = (i_c + d_c + 3) % 3;
                n_c++;
              }
            }
          }
        }

        V_PStop2 = V_PStop2_RPStmp;
        V_MZMtop2 = V_MZMtop2_RPStmp;
        V_PStop1 = V_PStop1_RPStmp;
        if(V_PStop1 < V_PStop1_min) V_PStop1 = V_PStop1_min;
        if(V_PStop1 > V_PStop1_max) V_PStop1 = V_PStop1_max;
        if(V_PStop2 < V_PStop2_min) V_PStop2 = V_PStop2_min;
        if(V_PStop2 > V_PStop2_max) V_PStop2 = V_PStop2_max;
        if(V_MZMtop2 < V_MZMtop2_min) V_MZMtop2 = V_MZMtop2_min;
        if(V_MZMtop2 > V_MZMtop2_max) V_MZMtop2 = V_MZMtop2_max;
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> set_out(PStop1_PIN, int(V_PStop1/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop1_PIN, int(V_MZMtop1_10/ALL_RAN*OUT_MAX));
        dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        #pragma endregion

        Serial.println("RPS.");
        Serial.println(V_PStop2);
        Serial.println(V_MZMtop2);
        Serial.println(V_PStop1);
        Serial.println(dS12_RPS_output);
        Serial.println(dS34_RPS_output);
        Serial.println(dS12_RPSY_output);
        Serial.println(dS34_RPSY_output);
      }
      else if(op == 5){
        // Auto message transmitting
        op = -1;

        Serial.println("Input here");

        while(!Serial.available()){ }

        delay(10);
        Data_idx = 0;
        while (Serial.available() > 0) {
          Data[Data_idx] = Serial.read();
          Data_idx++;
        }
        Data[Data_idx] = '\0';
        DataLength = Data_idx + 1;
        Encoder(Data, DataLength, DataTx);

        Tx_t1 = micros();
        Tx_idx = 0;
        Data_idx = 0;

        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < DataLength * 8){
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);
            }
            else if(Tx_idx >= Tx_CDR_num * 2 + DataLength * 8){
              dac -> set_out(MZMC_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }

            Tx_idx++;
          }
        }

        Serial.println("Done.");
      }
      else if(op == 8){
        /* 
        QWN test, single datagram transmitting
        This is dedicated for high-speed switching test, which only measures VV and HV components
        Speed and GPC results should be given initially (by Python)
        RPC is incorporated within the classical header, for a fixed amount of time usage
        */

        op = -1;

        // Acquire QWN direction
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          QWN_Header[6] = 0x01;
          QWN_Header[13] = 0x01;
        }
        else if(QWN_dir == 1){
          QWN_Header[6] = 0x03;
          QWN_Header[13] = 0x03;
        }
        Encoder(QWN_Header, QwnHeaderSize, DataTx);
        // for(int didx = 0; didx < QwnHeaderSize*8; didx++) Serial.print(DataTx[didx]);
        // Serial.print("\n");

        // Retrieve voltages for this path
        V_PStop2 = QWN_V_PStop2[QWN_dir];
        V_MZMtop2 = QWN_V_MZMtop2[QWN_dir];
        dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
        dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
        dac -> sync(1);
        int i_a = 0;
        int i_b = 0;
        int Tx_idx_ovhd = 0;

        // Transmit header data (before payload duration)
        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < 14 * 8){
              if(Tx_idx == 101){
                // Prepare for RPC
                // S1_RPC = DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0];
                // S3_RPC = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2];
                // S4_RPC = DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];
                // S2_RPC = S3_RPC + S4_RPC - S1_RPC;
                S1_RPC = 0;
                S2_RPC = 0;
                S3_RPC = 0;
                S4_RPC = 0;
                for(int idx=0; idx<PD_Repeat; idx++){
                  S1_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                  S3_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                  S4_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
                }
                S1_RPC = uint16_t(S1_RPC/PD_Repeat);
                S3_RPC = uint16_t(S3_RPC/PD_Repeat);
                S4_RPC = uint16_t(S4_RPC/PD_Repeat);
                S2_RPC = (S3_RPC + S4_RPC < S1_RPC) ? 0 : S3_RPC + S4_RPC - S1_RPC;
                dS12_RPC = abs(float(S1_RPC - S2_RPC) / float(S1_RPC + S2_RPC));
                dS34_RPC = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));
                dS12_max = dS12_RPC;
                dS34_min = dS34_RPC;
                dS12_RPC_prev = dS12_RPC;
                dS34_RPC_prev = dS34_RPC;
                dS12_RPC_output = dS12_RPC;
                dS34_RPC_output = dS34_RPC;
              }
              // CR, ID, IP
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;
            }
            else if(Tx_idx >= 14*8 && Tx_idx < 17*8){
              // RPC
              if(i_a == 1 && i_b == 0) Tx_idx_ovhd = 2;
              i_a = int((Tx_idx + Tx_idx_ovhd - 14*8)/2) / 3;
              i_b = int((Tx_idx + Tx_idx_ovhd - 14*8)/2) % 3;
              // Measure
              V_PStop2_RPC = sqrt(sq(V_PStop2) + (i_a-1) * Vsq_step_RPC);
              V_MZMtop2_RPC = sqrt(sq(V_MZMtop2) + (i_b-1) * Vsq_step_RPC);
              dac -> set_out(PStop2_PIN, int(V_PStop2_RPC/ALL_RAN*OUT_MAX));
              dac -> set_out(MZMtop2_PIN, int(V_MZMtop2_RPC/ALL_RAN*OUT_MAX));
              dac -> set_out(MZMC_PIN, int(Tx_VH/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              delayMicroseconds(QWN_RPC_delay_us);
              // S1_RPC = DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0];
              // S3_RPC = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2];
              // S4_RPC = DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];
              // S2_RPC = S3_RPC + S4_RPC - S1_RPC;
              S1_RPC = 0;
              S2_RPC = 0;
              S3_RPC = 0;
              S4_RPC = 0;
              for(int idx=0; idx<PD_Repeat; idx++){
                S1_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
                S3_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
                S4_RPC += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
              }
              S1_RPC = uint16_t(S1_RPC/PD_Repeat);
              S3_RPC = uint16_t(S3_RPC/PD_Repeat);
              S4_RPC = uint16_t(S4_RPC/PD_Repeat);
              S2_RPC = (S3_RPC + S4_RPC < S1_RPC) ? 0 : S3_RPC + S4_RPC - S1_RPC;
              dS12_RPC = abs(float(S1_RPC - S2_RPC) / float(S1_RPC + S2_RPC));
              dS34_RPC = abs(float(S3_RPC - S4_RPC) / float(S3_RPC + S4_RPC));

              if(dS12_RPC_prev < dS12_RPC_threshold){
                if(dS12_RPC > dS12_max){
                  dS12_max = dS12_RPC;
                  V_PStop2 = V_PStop2_RPC;
                  V_MZMtop2 = V_MZMtop2_RPC;
                  dS12_RPC_output = dS12_RPC;
                  dS34_RPC_output = dS34_RPC;
                }
              }
              else if(dS12_RPC_prev >= dS12_RPC_threshold && dS34_RPC_prev > dS34_RPC_limit){
                if(dS34_RPC < dS34_min){
                  dS34_min = dS34_RPC;
                  V_PStop2 = V_PStop2_RPC;
                  V_MZMtop2 = V_MZMtop2_RPC;
                  dS12_RPC_output = dS12_RPC;
                  dS34_RPC_output = dS34_RPC;
                }
              }

              dac -> set_out(PStop2_PIN, int(V_PStop2/ALL_RAN*OUT_MAX));
              dac -> set_out(MZMtop2_PIN, int(V_MZMtop2/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              QWN_V_PStop2[QWN_dir] = V_PStop2;
              QWN_V_MZMtop2[QWN_dir] = V_MZMtop2;

              Tx_idx = Tx_idx + 3;
            }
            else if(Tx_idx >= 17*8 && Tx_idx < 19*8){
              // Duration and terminator
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(50);
            }
            else if(Tx_idx >= 19*8){
              dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }

        // Return finish signal
        Serial.println("Header.");
        Serial.println(V_PStop2);
        Serial.println(V_MZMtop2);
        Serial.println(dS12_RPC_output);
        Serial.println(dS34_RPC_output);

        // for(Monitor_idx = 0; Monitor_idx < Monitor_length; Monitor_idx++) Serial.println(Monitor_data[Monitor_idx]);

      }
      else if(op == int('i' - '0')){
        /* 
        QWN test, single datagram transmitting
        This is dedicated for classical light switching test, without any polarization correction
        Speed should be given initially (by Python)
        */

        op = -1;

        // Acquire QWN direction
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          QWN_Header[6] = 0x01;
          QWN_Header[13] = 0x01;
          digitalWrite(SYNC, HIGH);
        }
        else if(QWN_dir == 1){
          QWN_Header[6] = 0x03;
          QWN_Header[13] = 0x03;
          digitalWrite(SYNC, LOW);
        }
        Encoder(QWN_Header, QwnHeaderSize, DataTx);
        // for(int didx = 0; didx < QwnHeaderSize*8; didx++) Serial.print(DataTx[didx]);
        // Serial.print("\n");

        // Transmit header data (before payload duration)
        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        // while(true){
        //   if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
        //     if(Tx_idx < 14 * 8){
        //       // CR, ID, IP
        //       Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
        //       Data_idx++;
        //       // dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
        //       dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
        //       dac -> sync(1);

        //       Tx_idx++;
        //     }
        //     else if(Tx_idx >= 14*8 && Tx_idx < 16*8){
        //       // Duration and terminator
        //       Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
        //       Data_idx++;
        //       // dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
        //       dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
        //       dac -> sync(1);

        //       Tx_idx++;

        //       delayMicroseconds(50);
        //     }
        //     else if(Tx_idx >= 16*8){
        //       // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        //       dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
        //       dac -> sync(1);
        //       break;
        //     }
        //   }
        // }

        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < 14 * 8){
              // CR, ID, IP
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VLMZMC : Tx_VHMZMC;
              // Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              // dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;
            }
            else if(Tx_idx >= 14*8 && Tx_idx < 41*8){
              // Duration and terminator
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VLMZMC : Tx_VHMZMC;
              // Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(MZMC_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              // dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(50);
            }
            else if(Tx_idx >= 41*8){
              dac -> set_out(MZMC_PIN, int(Tx_VLMZMC/ALL_RAN*OUT_MAX));
              // dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }

        // Return finish signal
        Serial.println("Header.");
      }
      else if(op == int('j' - '0')){
        /* 
        QWN test, single datagram transmitting
        This is dedicated for quantum light switching test, without any polarization correction
        Speed should be given initially (by Python)
        */

        op = -1;

        // Acquire QWN direction
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x01;
          QWN_Header[14] = 0x01;
          QWN_Header[15] = 0x01;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
        }
        else if(QWN_dir == 1){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x03;
          QWN_Header[14] = 0x01;
          QWN_Header[15] = 0x03;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          // digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 2){
          QWN_Header[6] = 0x03;
          QWN_Header[7] = 0x01;
          QWN_Header[14] = 0x03;
          QWN_Header[15] = 0x01;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          // digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 3){
          QWN_Header[6] = 0x03;
          QWN_Header[7] = 0x03;
          QWN_Header[14] = 0x03;
          QWN_Header[15] = 0x03;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          // digitalWrite(SYNC, LOW);
        }
        Encoder(QWN_Header, QwnHeaderSize, DataTx);
        digitalWrite(SYNC, HIGH);
        delayMicroseconds(10);
        digitalWrite(SYNC, LOW);

        // Transmit header data (before payload duration)
        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < 18 * 8){
              // CR, ID, IP
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(50);
            }
            // else if(Tx_idx >= 14*8 && Tx_idx < 16*8){
            //   // Duration and terminator
            //   Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
            //   Data_idx++;
            //   dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
            //   dac -> sync(1);

            //   Tx_idx++;

            //   delayMicroseconds(50);
            // }
            else if(Tx_idx >= 18*8){
              // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }

        // // Return finish signal
        // Serial.println("Header.");
      }
      else if(op == int('k' - '0')){
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
        }
        else if(QWN_dir == 1){
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
        }
      }
      else if(op == int('m' - '0')){
        /* 
        QWN test, single datagram transmitting
        This is dedicated for quantum light switching test, without any polarization correction
        Speed should be given initially (by Python)
        This is for case 2x2x3
        */

        op = -1;

        // Acquire QWN direction
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x01;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x01;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x01;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, HIGH);
        }
        else if(QWN_dir == 1){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x02;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x02;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x02;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 2){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x03;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x03;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x03;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 3){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x01;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x01;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x01;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 4){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x02;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x02;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x02;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 5){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x03;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x03;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x03;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, LOW);
        }
        Encoder(QWN_Header, QwnHeaderSize, DataTx);
        // digitalWrite(SYNC, HIGH);
        // delayMicroseconds(10);
        // digitalWrite(SYNC, LOW);

        // Transmit header data (before payload duration)
        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < QwnHeaderSize * 8){
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(30);
            }
            else if(Tx_idx >= QwnHeaderSize*8){
              // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }

      }
      else if(op == int('n' - '0')){
        /* 
        QWN test, single datagram transmitting
        This is dedicated for quantum light switching test, without any polarization correction
        Speed should be given initially (by Python)
        This is for case 1x2, but using the same code as of 2x2x3, and 3 sends out high level.
        */

        op = -1;

        // Acquire QWN direction
        while(!Serial.available()){ } // Is this necessary?
        QWN_dir = Serial.read() - '0';
        if(QWN_dir == 0){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x01;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x01;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x01;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, HIGH);
        }
        else if(QWN_dir == 1){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x02;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x02;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x02;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 2){
          QWN_Header[6] = 0x01;
          QWN_Header[7] = 0x03;
          QWN_Header[15] = 0x01;
          QWN_Header[16] = 0x03;
          QWN_Header[24] = 0x01;
          QWN_Header[25] = 0x03;
          digitalWrite(ORI_PIN1, LOW);
          digitalWrite(ORI_PIN2, HIGH);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 3){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x01;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x01;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x01;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, HIGH);
        }
        else if(QWN_dir == 4){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x02;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x02;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x02;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, LOW);
        }
        else if(QWN_dir == 5){
          QWN_Header[6] = 0x02;
          QWN_Header[7] = 0x03;
          QWN_Header[15] = 0x02;
          QWN_Header[16] = 0x03;
          QWN_Header[24] = 0x02;
          QWN_Header[25] = 0x03;
          digitalWrite(ORI_PIN1, HIGH);
          digitalWrite(ORI_PIN2, LOW);
          digitalWrite(SYNC, LOW);
        }
        Encoder(QWN_Header, QwnHeaderSize, DataTx);
        // digitalWrite(SYNC, HIGH);
        // delayMicroseconds(10);
        // digitalWrite(SYNC, LOW);

        // Transmit header data (before payload duration)
        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < QwnHeaderSize * 8){
              Tx_V = (DataTx[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(30);
            }
            else if(Tx_idx >= QwnHeaderSize*8){
              // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }
      }
      else if(op == int('p' - '0')){
        // Serial.println("Received command p");
        op = -1;
        while(!Serial.available()){ }
        char inputBuf[MAX_INPUT + 1];
        size_t n = Serial.readBytesUntil('\n', inputBuf, MAX_INPUT);
        inputBuf[n] = '\0';

        // --- Step 2: Compute packet length ---
        uint8_t payloadLen = static_cast<uint8_t>(n); // fits into 8-bit automatically
        
        size_t totalLen = PREFIX_COUNT + 1 + payloadLen;

        // --- Step 3: Build the packet dynamically on the stack ---
        char packet[totalLen + 1]; // max possible size
        size_t idx = 0;
        // Serial.print("Payload Length is ");
        // Serial.println(payloadLen);
        // Serial.println(inputBuf);
        // Add prefix 'U' * 5
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
        // Serial.println(packet);

        bool packetBin[totalLen * 8];
        Encoder(packet, totalLen, packetBin);

        Tx_t1 = micros();
        tic = Tx_t1;
        Tx_idx = 0;
        Data_idx = 0;
        // Serial.println("Beginning to reprint packet");

        // usage
        // printPacketBits(packetBin, totalLen * 8);

        while(true){
          if(micros() - Tx_t1 >= Tx_idx * Tx_period_us){
            if(Tx_idx < totalLen * 8){
              Tx_V = (packetBin[Data_idx] == 0) ? Tx_VL : Tx_VH;
              Data_idx++;
              dac -> set_out(QF4_PIN, int(Tx_V/ALL_RAN*OUT_MAX));
              dac -> sync(1);

              Tx_idx++;

              delayMicroseconds(30);
            }
            else if(Tx_idx >= totalLen*8){
              // dac -> set_out(MZMC_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> set_out(QF4_PIN, int(Tx_VL/ALL_RAN*OUT_MAX));
              dac -> sync(1);
              break;
            }
          }
        }
        Serial.println("Done");
      }
      else if (op == int('t' - '0')) {
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
  }
  else{

  }
}
