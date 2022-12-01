#include <SPI.h>
#include "pwm_lib.h"
#include <PeakDetection.h>
#include <filters.h>

#define PIN_RESET  9
#define PIN_DRDY   2
#define SS 10
#define SlaveSelectPin 10
#define PERIODS 16 

using namespace arduino_due::pwm_lib;

PeakDetection peakDetection;

const float cutoff_freq   = 15.0;  // Frequencia de corte
const float sampling_time = 0.005; // Amostra de tempo em segundos
IIR::ORDER  order  = IIR::ORDER::OD3; // Ordem (OD1 to OD4)
Filter f(cutoff_freq, sampling_time, order); // Filtro passa baixa

uint32_t periods[PERIODS]=
{
  10,      // 0.1 usecs. 
  100,     // 1 usec.
  1000,    // 10 usecs. 
  10000,   // 100 usecs. 
  100000,  // 1000 usecs.
  1000000, // 10000 usecs.
  10000000,// 100000 usecs.
  50000000,// 500000 usecs.
  50000000,// 500000 usecs.
  10000000,// 100000 usecs.
  1000000, // 10000 usecs.
  100000,  // 1000 usecs.
  10000,   // 100 usecs. 
  1000,    // 10 usecs. 
  100,     // 1 usec.
  10,      // 0.1 usecs. 
};

uint32_t period=1;

// Definindo pwm usando pin 35
pwm<pwm_pin::PWMH0_PC3> pwm_pin35;


//Variaveis globais
double timefrequency;
boolean flagE = true;
double frequenciaRespiracao = 0;
int valAFE;
bool flag;
double valAFEV;

void resetAFE4300();
void writeRegister(unsigned char address, unsigned int data); //Escrita do registrador do AFE4300
int readRegister(unsigned char address); //Leitura do registrador do AFE4300
void initAFE4300(); // Inicializa AFE4300
void initBCM(); // Inicializa modulo BCM

void readData(); //  Le do registrador ADC_DATA_RESULT


/***************************
Posição dos registradores AFE4300
****************************/
#define ADC_DATA_RESULT         0x00
#define ADC_CONTROL_REGISTER    0x01
#define MISC1_REGISTER          0x02
#define MISC2_REGISTER          0x03
#define DEVICE_CONTROL_1        0x09
#define ISW_MATRIX              0x0A
#define VSW_MATRIX              0x0B
#define IQ_MODE_ENABLE          0x0C
#define WEIGHT_SCALE_CONTROL    0x0D
#define BCM_DAC_FREQ            0x0E
#define DEVICE_CONTROL_2        0x0F
#define ADC_CONTROL_REGISTER_2  0x10
#define MISC3_REGISTER          0x1A

void setup()
{
  pinMode(PIN_RESET, OUTPUT); // Pino de reset
  pinMode(SS, OUTPUT);

  pinMode(PIN_DRDY, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), readData, RISING);

  Serial.begin(115200);
  Serial.flush();
   
  SPI.begin();
  SPI.setClockDivider(SlaveSelectPin,21);
  SPI.setDataMode(SlaveSelectPin, SPI_MODE1);

  peakDetection.begin(30, 2, 0.15); // Parametrização da função de detecção de picos
  
  resetAFE4300();
  initAFE4300();
  initBCM();

  pwm_pin35.start(periods[period],(periods[period]>>1)); // Gerador de 1MHZ no pino 35 para funcionamento interno AFE4300         
}
void loop() {  
  double valAFE;
  
  //Leitura das informações 
  if(flag) {
    valAFE =  readRegister(ADC_DATA_RESULT); 
    double filteredval = f.filterIn(valAFE); // Utilizando o filtro passa baixa de terceira ordem
    valAFEV= filteredval*0.000058; // Ganho para tensão

    double convertDigitalDataResult_2_3 = convertDigitalData_2_3(valAFEV); // Utilização da equação de posicionamento dos eletrodos de tensão 2/3
    
    peekDetectionFunction(convertDigitalDataResult_2_3); // Função de detecção de pico
    
    flag = 0;  
  }
} 
/**
* @brief  Resets the AFE4300 device
*/
void resetAFE4300(){
  digitalWrite(PIN_RESET ,LOW);
  delay(100);
  digitalWrite(PIN_RESET ,HIGH);
  writeRegister(MISC1_REGISTER,0x0000);
  writeRegister(MISC2_REGISTER,0xFFFF);
  writeRegister(MISC3_REGISTER,0x0030);
}

/**
* @brief  Writes to a register on the AFE4300
**/
void writeRegister(unsigned char address, unsigned int data)
{
  unsigned char firstByte = (unsigned char)(data >> 8);
  unsigned char secondByte = (unsigned char)data;
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  digitalWrite(SS,LOW);
  SPI.transfer(address);

  SPI.transfer(firstByte);
  SPI.transfer(secondByte);
  digitalWrite(SS,HIGH);
  
  SPI.endTransaction();
}      

/**
* @brief Reads from a register on the AFE4300
*/
int readRegister(unsigned char address)
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));
  
  int spiReceive = 0;
  unsigned char spiReceiveFirst = 0;
  unsigned char spiReceiveSecond = 0;
  
  address = address & 0x1F; // Os ultimos 5 bits são para especificar a direção
  address = address | 0x20; // Os 3 primeiros 3 bits são para a operação de leitura
  
  digitalWrite(SS,LOW);  
  SPI.transfer(address);
  spiReceiveFirst  = SPI.transfer(0x00);
  spiReceiveSecond = SPI.transfer(0x00);
  digitalWrite (SS, HIGH);

  SPI.endTransaction();

  // A combinação dos dois bytes gera o sinal 
  spiReceive = (spiReceiveFirst << 8); 
  spiReceive |= spiReceiveSecond;
  return spiReceive;
}

/**
* @brief  Initializes the AFE4300 device
*/
void initAFE4300()
{
  writeRegister(ADC_CONTROL_REGISTER,0x5140);
  writeRegister(DEVICE_CONTROL_1,0x6004);//,0x6006
  writeRegister(ISW_MATRIX,0x0000);//0x0804
  writeRegister(VSW_MATRIX,0x0000);//0x0804
  writeRegister(IQ_MODE_ENABLE,0x0000);
  writeRegister(WEIGHT_SCALE_CONTROL,0x0000);
  writeRegister(BCM_DAC_FREQ,0x0010);//0x0010
  writeRegister(DEVICE_CONTROL_2,0x0000);
  writeRegister(ADC_CONTROL_REGISTER_2,0x0011);//0x0063
  writeRegister(MISC1_REGISTER,0x0000);
  writeRegister(MISC2_REGISTER,0xFFFF);
  writeRegister(MISC3_REGISTER,0x0030);//0x00C0//0x0030  
}

/**
* @brief  Initializes the BCM Module
*/
void initBCM()
{
  writeRegister(ADC_CONTROL_REGISTER,0x4120); // Modo de medição diferencial, 32 SPS
  writeRegister(DEVICE_CONTROL_1,0x0006); // Escrita do modo BCM
  writeRegister(ISW_MATRIX,0x8040); // Canais de saida IOUTP5 e IOUTN4 
  writeRegister(VSW_MATRIX,0x8040); // canais de entrada VSENSEP5 and VSENSEN4
  writeRegister(ADC_CONTROL_REGISTER_2,0x0063); // Seleção do output do BCM
  writeRegister(WEIGHT_SCALE_CONTROL,0x0000); // Ganho = 1 Offset = 0
  writeRegister(BCM_DAC_FREQ,0x0010);// Frequencia BCM
  writeRegister(DEVICE_CONTROL_2,0x0006);
} 

/**
* @brief  Reads the ADC data register
*/
void readData()
{
  flag = 1;
}    

/**
* @brief PeekDetection function
*/
void peekDetectionFunction(double data){
  double timer = millis(); // Inicia a contagem para controle da frequencia respiratoria
  peakDetection.add(data); // adição de um novo ponto
  int peak = peakDetection.getPeak(); // returns 0, 1 or -1


  Serial.print(data); // print data
  if (peak>=0 && data > 4) { // Sinal de corte para picos
    peak = 1;
    if (timefrequency != 0 && flagE){    
      frequenciaRespiracao = 60/(timer-timefrequency)*1000; // Função de conversão para frequencia respiratoria
    } 
    timefrequency = timer;

    flagE = false;
  }
  else {
    peak = 0;
    flagE = true;
  }
  Serial.print(",");
  Serial.print(frequenciaRespiracao);
  Serial.print(",");
  Serial.print(peak*10); // print peak status
  Serial.println("\r\n");
}

/**
* @brief Conversion equation for the tension electrodes in the positions 2/3
*/
double convertDigitalData_2_3(double data) {
  double anm = 1.58e10;
  double bnm = -1.99e8;
  double cnm = 8.39e5;
  double dnm = -1181;

  return (anm * pow(data,3)) + (bnm * pow(data,2)) + (cnm * data) + dnm;
}

/**
* @brief Conversion equation for the tension electrodes in the positions 2/4
*/
double convertDigitalData_2_4(double data) {
  double anm = 3.27e9;
  double bnm = -1.03e8;
  double cnm = 1.08e6;
  double dnm = -3760;

  return (anm * pow(data,3)) + (bnm * pow(data,2)) + (cnm * data) + dnm;
}

/**
* @brief Conversion equation for the tension electrodes in the positions 2/6
*/
double convertDigitalData_2_6(double data) {
  double anm = 1.11e9;
  double bnm = -4.18e7;
  double cnm = 5.27e5;
  double dnm = -2212;

  return (anm * pow(data,3)) + (bnm * pow(data,2)) + (cnm * data) + dnm;
}

/**
* @brief Conversion equation for the tension electrodes in the positions 2/7
*/
double convertDigitalData_2_7(double data) {
  double anm = 1.13e10;
  double bnm = -1.43e8;
  double cnm = 6.06e5;
  double dnm = -858.7;

  return (anm * pow(data,3)) + (bnm * pow(data,2)) + (cnm * data) + dnm;
}

/**
* @brief Conversion equation for the tension electrodes in the positions 2/8
*/
double convertDigitalData_2_8(double data) {
  double anm = 3.14e8;
  double bnm = 9.06e5;
  double cnm = 2.02e3;
  double dnm = 4.6;

  return (anm * pow(data,3)) + (bnm * pow(data,2)) + (cnm * data) + dnm;
}
