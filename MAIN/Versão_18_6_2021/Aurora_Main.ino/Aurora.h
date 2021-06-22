#include <Wire.h>
#include <math.h>

#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <BMP280_DEV.h> 
#include "wiring_private.h"    //sercom 
#include <Arduino.h>

//Defines LORA

#define loraRXD 3
#define loraTXD 2


//Defines SD

#define cardDetect 9
#define chipSelect 10

//Defines Ejecção

#define sendPin 5
#define Minimo_acc 18          //m/s2 acelaracao minimia 9 m/s acima do zero para flight mode 
#define Minimo_temp_flight 1100.0       // milisegundos
#define tempo_v_ejecao 1100.0          //tempo de amostragem velocidade para ejeção
#define tempo_h_ejecao 1800.0       //tempo de amostragem altitude máxima para ejeção
#define tempo_seguranca 15000.0   //milisegundos
#define period 26800.0               // Periodo de Ejeção em Milisegundos

//gps stuff
char const Temp[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
#define SET_POS_FIX_100MS   "$PMTK220,100"
#define SET_NMEA_BAUDRATE_115200    "$PMTK251,115200"
#define SET_NMEA_OUTPUT "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
#define lora_aux 4
#define TEMPO_APOGEU 20000  //milliiis

long int tempo_delay_ejecao;
//bno stuff
Adafruit_BNO055 bno = Adafruit_BNO055(00, 0x28);
BMP280_DEV bmp280;

//vars init!
int resetPin = 2;
unsigned long startMillis; 
unsigned long starttime=0;
byte ejecao = 0;
byte counter = 0;
byte counter_acel = 0;
String nome_ficheiro; //Valores Cartão SD
double AcXf, AcYf, AcZf,rot_x,rot_y,rot_z; //Valores BNO
sensors_event_t orientationData , accelerometerData;
double latitude; //gps
double longitude;
String tempo;
//Altimetro
int Flag_flight = 0;
float altitude_medida;
String leitura;
float m=7,m0=5.8;
float t = 0;
float v=0, h=0, P = 1000, P2=1000;
float delta;
float mfr;
float acel_vert;
float acel_corrigida;
float haux;
float tempo_aux_v = 0;
float tempo_aux_h = 0;
float altitude_max = -999999;
