#include <Wire.h>
#include <math.h>
#include <NeoSWSerial.h>
#include <SPI.h>
#include <SD.h>
#include "SparkFunMPL3115A2.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "DEV_Config.h"
#include "L76X.h"


//Defines LORA

#define loraRXD 3
#define loraTXD 2
#define lora_aux 3

//Defines SD

#define cardDetect 7
#define chipSelect 6

//Defines Ejecção

#define sendPin 10
#define Minimo_acc 2.0                                                              //g's
#define Minimo_temp 300.0                                                           // milisegundos
//#define it_v_ejecao 3.0                                                           //apagar??
#define tempo_v_ejecao 300.0 //depende muito da freq, 3Hz impossibilita 300ms       //tempo de amostragem velocidade para ejeção
#define tempo_h_ejecao 1500.0                                                       //tempo de amostragem altitude máxima para ejeção
#define tempo_seguranca 10000.0                                                     //milisegundos
#define period 26800.0                                                              // Periodo de Ejeção em Milisegundos


//NeoSWSerial loraSerial(loraRXD,loraTXD);                                               // RXD, TXD  Ao contrário nos dispositivos
Adafruit_BNO055 bno = Adafruit_BNO055(00, 0x28);
MPL3115A2 altimetro;
