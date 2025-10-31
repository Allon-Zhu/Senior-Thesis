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


#pragma region Global Parameters
#define BaudRate 115200
const uint16_t Tx_period_us = 100;
const uint16_t Rx_period_us = 1;
#pragma endregion Global Parameters

#pragma region Rx Parameters
DueAdcFast DueAdcF(1024);
const int Rx_PIN = A1;
const int PD_Repeat = 5;
uint16_t S1 = 0;
uint16_t S2 = 0;
uint16_t S3 = 0;
uint16_t S4 = 0;
uint16_t Delay_ms = 100;
uint16_t PD_Calib_data[4] = {2900, 3500, 3160, 2660};
// uint16_t PD_Calib_data[4] = {1, 1, 1, 1};
float PD_Calib_ratio[4] = {0.0, 0.0, 0.0, 0.0};
int PD_DWDM = 2;
float PD_DWDM_ratio = 0.3;

uint16_t S_init = 10000;
uint16_t Rx_SH = 0;
uint16_t Rx_SL = 0;
uint16_t Rx_Sdes = 700;
float Rx_init_des = 0.2;

int Tx_CDR_num = 20;
static uint16_t DataBufferSize = 200;
char* Data = new char[DataBufferSize];
bool* DataTx = new bool[DataBufferSize * 8];
int DataLength = 0;
int Data_idx = 0;

int Rx_CDR_idx = 0;
int Rx_CDR_phase = 50;
unsigned long Rx_CDR_t1 = 0;
unsigned long Rx_CDR_t2 = 0;
float Rx_DataPeriod_us = 0.0;
char Rx_IP = 0x21;
int Rx_Data_idx = 0;
unsigned long Rx_Data_t1 = 0;
unsigned long Rx_Data_t2 = 0;
uint8_t Rx_Data_bit = 0;
char Rx_Data_Reg = 0;
uint8_t Rx_Data_Reg_idx = 0;
int Rx_Data_Length = 0;

int* Monitor_data = new int[40];
// uint16_t* Monitor_data = new uint16_t[1000];
int Monitor_idx = 0;
int Monitor_length = 0;

const int PD1_PIN = A0;
const int PD2_PIN = A1;
const int PD3_PIN = A2;
const int PD4_PIN = A3;
int t1 = 0;
int t2 = 0;
int dt = 0;

int FT_buf = 10000;
unsigned int* FT_dt = new unsigned int[FT_buf];
// uint16_t* FT_s = new uint16_t[FT_buf];
float* FT_s = new float[FT_buf];
int FT_num = 0;
int FT_delay = 2000;

#pragma endregion Rx Parameters

#pragma region QSWR Parameters
const int QWSR_PIN = 2;
bool QWSR_dir = 0;
uint16_t QWN_S1 = 0;
uint16_t QWN_S2 = 0;
#pragma endregion

#pragma region QWNC Parameters
int N_QWNC_c = 100;
int Delay_QWNC_us_c = 200;
uint16_t* S3_QWNC = new uint16_t[N_QWNC_c];
uint16_t* S4_QWNC = new uint16_t[N_QWNC_c];
int QWNC_idx = 0;
int QWNC_t0 = 0;
int QWNC_t1 = 0;
int QWNC_dt = 0;
#pragma endregion

#pragma region Logic Control Parameters
uint8_t Inf_idx = 0;
uint8_t Inf_ID_idx = 1;
uint8_t Inf_IP_idx = 2;
uint8_t Inf_RPC_idx = 3;  // RPC for 2 bytes
uint8_t Inf_QPD_idx = 4;
uint8_t Inf_TER_idx = 5;
bool Inf_RPC_marker = 0;

uint8_t RPC_byte_idx = 0;
uint8_t RPC_byte_num = 3;
uint8_t QPD_byte_num = 1;
uint8_t QPD_byte_idx = 0;
#pragma endregion

#pragma region Loop Parameters
int Rx_read = 0;
bool start_loop = 0;
uint8_t mode = 0;
int num_loop = 0;

unsigned long tic = 0;
#pragma endregion Loop Parameters

#pragma region Alan Detection Parameters

#define PREAMBLE_EDGE_TIMEOUT_US     200000UL // per-edge wait cap during preamble (200 ms)
#define EDGE_CONFIRM_US              2       // microseconds to re-check level when an edge is seen
#define TRIM_PERCENT                 20 
#define THRESH            500     // threshold for HIGH/LOW (volts or ADC-converted units)
#define PREFIX_COUNT      4         // number of leading 'U' (0x55) bytes in preamble

// Inline macro for bit read (no functions as requested)
#define READ_BIT()  ((DueAdcF.ReadAnalogPin(PD1_PIN) > THRESH) ? 1 : 0)
#pragma endregion Alan Detection Parameters


void setup() {
  Serial.begin(BaudRate);
  Serial.setTimeout(50);
  // delay(1500);

  // Calculate PD calib ratios
  // for(int ipd = 0; ipd < 4; ipd++) PD_Calib_ratio[ipd] = float(PD_Calib_data[0]) / float(PD_Calib_data[ipd]);
  for(int ipd = 0; ipd < 4; ipd++){ 
    PD_Calib_ratio[ipd] = float(PD_Calib_data[0]) / float(PD_Calib_data[ipd]);
    if(ipd == int(PD_DWDM) - 1) PD_Calib_ratio[ipd] = PD_Calib_ratio[ipd] / float(PD_DWDM_ratio);
  }  
  
  pinMode(QWSR_PIN, OUTPUT);
  delayMicroseconds(300);
  digitalWrite(QWSR_PIN, HIGH);

  DueAdcF.EnablePin(A0);
  // DueAdcF.EnablePin(A1);
  DueAdcF.EnablePin(A2);
  DueAdcF.EnablePin(A3);
  DueAdcF.Start1Mhz(); 

  for(QWNC_idx = 0; QWNC_idx < N_QWNC_c; QWNC_idx++){
    S3_QWNC[QWNC_idx] = uint16_t(0);
    S4_QWNC[QWNC_idx] = uint16_t(0);
  }
}


void ADC_Handler() {
  DueAdcF.adcHandler();
}


void loop() {
  if(Serial.available() > 0){
    mode = Serial.read() - '0';
  }

  if(mode == 1){
    /*
    This is the code used by the classical receiver. It recognizes all the information (also skipping the RPC part).
    */

    while(1){
      if(start_loop == 0){
        start_loop = 1;
        Rx_read = 0;
        Inf_idx = 0;

        // S_init = DueAdcF.ReadAnalogPin(Rx_PIN);
        delayMicroseconds(100);
        QWN_S1 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];

        // Reset the Data buffer to accept new data.
        for(Rx_Data_idx = 0; Rx_Data_idx < DataBufferSize; Rx_Data_idx++){
          Data[Rx_Data_idx] = 0x00;
        }
        Rx_Data_idx = 0;
        Rx_Data_Reg = 0x00;
        Rx_Data_Reg_idx = 0;
        QPD_byte_idx = 0;

        // Monitor_idx = 0;
        // Monitor_length = 0
      }

      if(Rx_read == 0){
        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];

        if(QWN_S1 < Rx_Sdes && QWN_S2 > Rx_Sdes) {
          // If the signal goes from low to high, then the zerosth CDR pulse is recorded.
          Rx_CDR_idx = 0;
          Rx_read = 1;
          
        }
        else if(QWN_S1 > Rx_Sdes && QWN_S2 < Rx_Sdes) {
          // If the signal goes from high to low, then the first CDR pulse is recorded.
          Rx_CDR_idx = 1;
          Rx_CDR_t1 = micros();
          Rx_read = 1;
        }

        QWN_S1 = QWN_S2;
      }
      else if(Rx_read == 1){
        // Read mode 1: CDR
        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];;

        if(QWN_S1 >= Rx_Sdes){
          if(QWN_S2 < Rx_Sdes){
            Rx_CDR_idx++;
            if(Rx_CDR_idx == 1) Rx_CDR_t1 = micros();
            else if(Rx_CDR_idx == Tx_CDR_num){
              Rx_read = 2;
              Rx_CDR_t2 = micros();
              Rx_DataPeriod_us = float(Rx_CDR_t2 - Rx_CDR_t1)/float(Tx_CDR_num - 1)/2.0;
              // if(num_loop > 0){
                // Serial.println(Rx_CDR_t2);
                // Serial.println(Rx_CDR_t1);
              // }
              Rx_Data_t1 = int(Rx_CDR_t2 + Rx_DataPeriod_us + Rx_CDR_phase);
              Rx_Data_idx = 0;
              Rx_Data_Reg = 0x00;
              Rx_Data_Reg_idx = 0;
              Rx_Data_Length = 0;
              Inf_idx++;  // This marks the finish of CDR process
              Inf_RPC_marker = 0;
            }
          }
        }

        QWN_S1 = QWN_S2;
      }
      else if(Rx_read == 2){
        // Read mode 2: True Data to be transmitted
        while(micros() < Rx_Data_t1 + int(float(Rx_Data_idx) * Rx_DataPeriod_us)){}
        Rx_Data_idx++;

        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];

        if(QWN_S2 > Rx_Sdes) Rx_Data_bit = 1;
        if(QWN_S2 < Rx_Sdes) Rx_Data_bit = 0;

        Rx_Data_Reg |= (Rx_Data_bit << Rx_Data_Reg_idx);
        
        Rx_Data_Reg_idx++;

        if(Rx_Data_Reg_idx == 8){
          if(Inf_idx == Inf_ID_idx){
            // Serial.println(Rx_Data_Reg, HEX);
            if(Rx_Data_Reg != 0x32){
              // Rx_Data_Reg != 0x32 ("2") means this is not the information supposed to be received.
              start_loop = 0;
              delayMicroseconds(int(Rx_DataPeriod_us * 5));
              num_loop++;
            }
            else if(Rx_Data_Reg == 0x32){
              // Rx_Data_Reg == 0x32 ("2") means this is the information supposed to be received, and Inf_idx goes for IP
              Rx_Data_Reg = 0x00;
              Rx_Data_Reg_idx = 0;
              Inf_idx = Inf_IP_idx;
            }
          }
          else if(Inf_idx == Inf_IP_idx){
            // Determine Inf2 IP correctness
            // if(Rx_Data_Reg != Rx_ID){
            //   // Rx_Data_Reg != Rx_ID means IP is wrong, and the code should directly start looking for the terminator
            //   Inf_idx = Inf_TER_idx;

            //   Rx_Data_Reg = 0x00;
            //   Rx_Data_Reg_idx = 0;
            // }
            // else if(Rx_Data_Reg == Rx_ID){
            //   // Rx_Data_Reg == Rx_ID means IP is wrong, start recording Quantum Payload Duration information
            //   Inf_idx = Inf_QPD_idx;

            //   Rx_Data_Reg = 0x00;
            //   Rx_Data_Reg_idx = 0;
            // }

            Inf_idx = Inf_RPC_idx;
            RPC_byte_idx = 0;
            Rx_IP = Rx_Data_Reg;
            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
          else if(Inf_idx == Inf_RPC_idx){
            // RPC stage, skip the next 3 bytes
            RPC_byte_idx++;
            if(RPC_byte_idx == RPC_byte_num) Inf_idx = Inf_QPD_idx;
            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
          else if(Inf_idx == Inf_QPD_idx){
            Data[QPD_byte_idx] = Rx_Data_Reg;
            QPD_byte_idx++;
            if(QPD_byte_idx == QPD_byte_num) Inf_idx = Inf_TER_idx;

            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
          else if(Inf_idx == Inf_TER_idx && Rx_Data_Reg == 0x00){
            Rx_read = 0;
            start_loop = 0;
            Rx_Data_Length = 0;
            Serial.println(Rx_IP, HEX);
            Serial.println((int)Data[0]);
            // Serial.println(Data[0], HEX);
            
            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
          else{
            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
        }
      }

      delayMicroseconds(Rx_period_us);
    }
  }
  else if(mode == 2){
    /*
    This is the code used by the switching unit. It recognizes ID=1 and the following IP to finish the switching. Then it directly goes for the terminator.
    */

    while(1){
      if(start_loop == 0){
        start_loop = 1;
        Rx_read = 0;
        Inf_idx = 0;

        // S_init = DueAdcF.ReadAnalogPin(Rx_PIN);
        delayMicroseconds(100);
        QWN_S1 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];

        // Reset the Data buffer to accept new data.
        for(Rx_Data_idx = 0; Rx_Data_idx < DataBufferSize; Rx_Data_idx++){
          Data[Rx_Data_idx] = 0x00;
        }
        Rx_Data_idx = 0;
        Rx_Data_Reg = 0x00;
        Rx_Data_Reg_idx = 0;
        QPD_byte_idx = 0;

        Monitor_idx = 0;
        // num_loop++;
        // delay(50);
      }

      if(Rx_read == 0){
        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];;

        if(QWN_S1 < Rx_Sdes && QWN_S2 > Rx_Sdes) {
          // If the signal goes from low to high, then the zerosth CDR pulse is recorded.
          Rx_CDR_idx = 0;
          Rx_read = 1;
        }
        else if(QWN_S1 > Rx_Sdes && QWN_S2 < Rx_Sdes) {
          // If the signal goes from high to low, then the first CDR pulse is recorded.
          Rx_CDR_idx = 1;
          Rx_CDR_t1 = micros();
          Rx_read = 1;
        }

        QWN_S1 = QWN_S2;
      }
      else if(Rx_read == 1){
        // Read mode 1: CDR
        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];;

        if(QWN_S1 >= Rx_Sdes){
          if(QWN_S2 < Rx_Sdes){
            Rx_CDR_idx++;
            if(Rx_CDR_idx == 1) Rx_CDR_t1 = micros();
            else if(Rx_CDR_idx == Tx_CDR_num){
              Rx_read = 2;
              Rx_CDR_t2 = micros();
              Rx_DataPeriod_us = float(Rx_CDR_t2 - Rx_CDR_t1)/float(Tx_CDR_num - 1)/2.0;
              // if(num_loop > 0){
              //   Serial.println(Rx_CDR_t2);
              //   Serial.println(Rx_CDR_t1);
              // }
              Rx_Data_t1 = int(Rx_CDR_t2 + Rx_DataPeriod_us + Rx_CDR_phase);
              Rx_Data_idx = 0;
              Rx_Data_Reg = 0x00;
              Rx_Data_Reg_idx = 0;
              Rx_Data_Length = 0;
              Inf_idx++;
            }
          }
        }

        QWN_S1 = QWN_S2;
      }
      else if(Rx_read == 2){
        // Read mode 2: True Data to be transmitted
        while(micros() < Rx_Data_t1 + int(float(Rx_Data_idx) * Rx_DataPeriod_us)){}
        Rx_Data_idx++;

        QWN_S2 = DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3];;

        if(QWN_S2 > Rx_Sdes) Rx_Data_bit = 1;
        if(QWN_S2 < Rx_Sdes) Rx_Data_bit = 0;

        Rx_Data_Reg |= (Rx_Data_bit << Rx_Data_Reg_idx);
        Rx_Data_Reg_idx++;

        if(Inf_idx == Inf_IP_idx){
          if(Rx_Data_Reg_idx == 2){
            if(Rx_Data_Reg == 0x01) QWSR_dir = 0;
            else if(Rx_Data_Reg == 0x02) QWSR_dir = 1;
          }
          else if(Rx_Data_Reg_idx == 5){
            if(QWSR_dir == 0) digitalWrite(QWSR_PIN, LOW);
            else if(QWSR_dir == 1) digitalWrite(QWSR_PIN, HIGH);
          }
          else if(Rx_Data_Reg_idx == 8){
            Inf_idx = Inf_TER_idx;
            Rx_Data_Reg = 0x00;
            Rx_Data_Reg_idx = 0;
          }
        }
        else{
          if(Rx_Data_Reg_idx == 8){
            if(Inf_idx == Inf_ID_idx){
              if(Rx_Data_Reg != 0x31){
                // Rx_Data_Reg != 0x31 ("2") means this is not the information supposed to be received. Directly look for the terminator.
                Inf_idx = Inf_TER_idx;

                Rx_Data_Reg = 0x00;
                Rx_Data_Reg_idx = 0;
              }
              else if(Rx_Data_Reg == 0x31){
                // Rx_Data_Reg == 0x32 ("2") means this is the information supposed to be received, and Inf_idx goes for IP
                Rx_Data_Reg = 0x00;
                Rx_Data_Reg_idx = 0;
                Inf_idx = Inf_IP_idx;
              }
            }
            else if(Inf_idx == Inf_TER_idx && Rx_Data_Reg == 0x00){
              Rx_read = 0;
              start_loop = 0;
              Rx_Data_Length = 0;
              Serial.println(QWSR_dir);
              
              Rx_Data_Reg = 0x00;
              Rx_Data_Reg_idx = 0;
            }
            else{
              Rx_Data_Reg = 0x00;
              Rx_Data_Reg_idx = 0;
            }
          }
        }
      }

      delayMicroseconds(Rx_period_us);
    }
  }
  else if(mode == 7){
    t1 = millis();
    while(1){
      S1 = 0;
      S2 = 0;
      S3 = 0;
      S4 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S1 += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN));
        S3 += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN));
        S4 += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN));
      }
      S1 = uint16_t(S1/PD_Repeat);
      S3 = uint16_t(S3/PD_Repeat);
      S4 = uint16_t(S4/PD_Repeat);
      S2 = (S3 + S4 < S1) ? 0 : S3 + S4 - S1;

      Serial.println(S1);
      Serial.println(S2);
      Serial.println(S3);
      Serial.println(S4);
      t2 = millis();
      dt = t2 - t1;
      Serial.println(dt);
      delay(Delay_ms);
    }
  }
  else if(mode == 8){
    // Fourier Transform measurement
    mode = -1;

    t1 = micros();
    t2 = micros();
    FT_num = 0;
    while(t2 - t1 < FT_delay * FT_buf){
      t2 = micros();
      FT_dt[FT_num] = (unsigned int)(t2 - t1);
      // FT_s[FT_num] = uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
      // FT_s[FT_num] = uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
      FT_s[FT_num] = float(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]) / float(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2] + DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
      FT_num++;

      delayMicroseconds(FT_delay);
    }

    Serial.println("Done.");
    Serial.println(FT_num);

    for(int i = 0; i < FT_num; i++){
      Serial.println(FT_dt[i]);
      Serial.println(FT_s[i]);
    }
    
  }
  else if(mode == 9){
    t1 = millis();
    while(1){
      S1 = 0;
      S2 = 0;
      S3 = 0;
      S4 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S1 += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN) * PD_Calib_ratio[0]);
        S3 += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
        S4 += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
      }
      S1 = uint16_t(S1/PD_Repeat);
      S3 = uint16_t(S3/PD_Repeat);
      S4 = uint16_t(S4/PD_Repeat);
      S2 = (S3 + S4 < S1) ? 0 : S3 + S4 - S1;

      Serial.println(S1);
      Serial.println(S2);
      Serial.println(S3);
      Serial.println(S4);
      t2 = millis();
      dt = t2 - t1;
      Serial.println(dt);
      delay(Delay_ms);
    }
  }
  else if(mode == int('a' - '0')){
    // Return the sum of S3 and S4
    t1 = millis();
    while(1){
      S3 = 0;
      S4 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S3 += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
        S4 += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
      }
      S3 = uint16_t(S3/PD_Repeat);
      S4 = uint16_t(S4/PD_Repeat);

      Serial.println(S3 + S4);
      t2 = millis();
      dt = t2 - t1;
      Serial.println(dt);
      delay(Delay_ms);
    }
  }
  else if(mode == int('b' - '0')){
    // Return S3 and S4
    t1 = millis();
    while(1){
      S3 = 0;
      S4 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S3 += uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN) * PD_Calib_ratio[2]);
        S4 += uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN) * PD_Calib_ratio[3]);
      }
      S3 = uint16_t(S3/PD_Repeat);
      S4 = uint16_t(S4/PD_Repeat);

      Serial.println(S3);
      Serial.println(S4);
      t2 = millis();
      dt = t2 - t1;
      Serial.println(dt);
      delay(Delay_ms);
    }
  }
  else if(mode == int('i' - '0')){
    // QWNC test
    QWNC_idx = 0;
    QWNC_t0 = micros();
    while(QWNC_idx < N_QWNC_c){
      S3_QWNC[QWNC_idx] = uint16_t(DueAdcF.ReadAnalogPin(PD3_PIN));
      S4_QWNC[QWNC_idx] = uint16_t(DueAdcF.ReadAnalogPin(PD4_PIN));
      QWNC_idx++;
      delayMicroseconds(Delay_QWNC_us_c);
    }
    QWNC_t1 = micros() - QWNC_t0;

    Serial.println("QWNC");
    Serial.println(N_QWNC_c);
    Serial.println(QWNC_t1);

    int N_return = int(N_QWNC_c / 500);
    for(int ir = 0; ir < N_return; ir++){
      while (Serial.available() == 0) {}
      Serial.parseInt();
      Serial.println("Copy.");
      for(QWNC_idx = 0; QWNC_idx < 500; QWNC_idx++){
        Serial.println(S3_QWNC[QWNC_idx + 500 * ir]);
        Serial.println(S4_QWNC[QWNC_idx + 500 * ir]);
      }
    }
  }
  else if(mode == int('q' - '0')){
    while (Serial.available() == 0) {}
    // long t1 = millis();
    Serial.parseInt();
    // long t2 = millis();

    
    S1 = 0;
    for(int idx=0; idx<1; idx++){
      S1 += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN));
    }
    // S1 = uint16_t(S1/PD_Repeat);
    Serial.println(S1);
    
    // Serial.println(t2 - t1);
  }
  else if(mode == int('z' - '0')){
    t1 = millis();
    while(1){
      S1 = 0;
      for(int idx=0; idx<PD_Repeat; idx++){
        S1 += uint16_t(DueAdcF.ReadAnalogPin(PD1_PIN));
      }
      S1 = uint16_t(S1/PD_Repeat);

      Serial.println(S1);
      t2 = millis();
      dt = t2 - t1;
      // Serial.println(dt);
      delay(Delay_ms);
    }
  }
  else if(mode == int('p' - '0')) {
    int last = READ_BIT();
    while (last != 0) { last = READ_BIT(); }          // ensure LOW first

    for (;;) {
      int b = READ_BIT();
      if (b != last && b == 1) {
        if (EDGE_CONFIRM_US) { delayMicroseconds(EDGE_CONFIRM_US); if (READ_BIT() != 1) { last = b; continue; } }
        last = 1;
        break;                                        // first rising edge found
      }
      last = b;
    }
    uint32_t t_start = micros();                      // time of first rising edge

    // ===== 2) Consume the rest of the preamble and land on the LAST falling edge =====
    // 4×'U' -> 32 bits -> 31 intervals between edges. We already saw the 1st edge.
    const uint16_t EDGES_TO_MEASURE = (PREFIX_COUNT * 8) - 1;

    for (uint16_t i = 0; i < EDGES_TO_MEASURE; ++i) {
      int b;
      do {
        b = READ_BIT();
      } while (b == last);

      if (EDGE_CONFIRM_US) {                          // deglitch
        int b2 = READ_BIT();
        if (b2 == last) { --i; continue; }           // transient; re-wait for this edge
        b = b2;
      }

      last = b;
    }

    // At this point, last_edge_t is the time of the FINAL falling edge of the preamble
    uint32_t t_end   = micros();
    uint32_t span_us = (t_end - t_start);            // time covering 31 bit intervals

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
    uint8_t totalLenBytes = 0;
    for (uint8_t i = 0; i < 8; i++) {
      while ((int32_t)(micros() - t_sample) < 0) { /* wait */ }
      int b = READ_BIT();
      if (b) totalLenBytes |= (1u << i);     // LSB-first
      t_sample += bit_us;
    }

    int payloadLen = (int)totalLenBytes;
    if (payloadLen < 0) {
      Serial.print(F("Bad length: ")); Serial.println(totalLenBytes);
      return;  // resync on next packet
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
