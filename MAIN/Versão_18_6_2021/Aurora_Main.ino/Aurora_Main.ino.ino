#include "Aurora.h"
#include <BMP280_DEV.h> 
/*###--------------------------------------------------------------Merida C++ code----------------------------------------------------------------###
 *                                                                       2021
 *                                                                       
 * this is the final code done for Aurora 3.0 during the year 2021 by the eletronics team
 * you can see the fow diagram in ##### to better understand how does it works
 * 
 *                                                           Teamwork makes the dream work!
 */


/*###------------------------------------------------------------------Inits----------------------------------------------------------------------###
 * 
 */

//serial for the arduino <-> gps communication
Uart ss (&sercom0, 6, 7, SERCOM_RX_PAD_0, UART_TX_PAD_2);
TinyGPSPlus gps;

void SERCOM0_Handler()
{
    ss.IrqHandler();
}



/*###--------------------------------------------------------------INITial setup----------------------------------------------------------------------###
 * after all initializations necessary for all the sensors and the opening of the serial comms for the gps and loras
 * the module enters standby mode
 * 
 */
void setup()
{
   digitalWrite(resetPin, HIGH);
  Wire.begin();  //Começa a comunicação i2c
  Serial.begin(9600);
//while(!Serial){}
  pinMode(resetPin, OUTPUT);     
  pinMode(5, OUTPUT); //pino ejecao
  Serial.println("Hello Mérida!!");
  delay(500);
  pinPeripheral(6, PIO_SERCOM_ALT);
  delay(50);
  pinPeripheral(7, PIO_SERCOM_ALT);
  delay(50);
  incializa_BNO();
  delay(50);
  inicializa_LORA();
  delay(50); 
  inicializa_GPS();
  delay(50);
  inicializar_bmp();
  delay(50);
  nome_ficheiro = inicializa_SD();
  envia_LORA("INITIAL SETUP DONE!!!!");
  
/*###--------------------------------------------------------------Standby Mode---------------------------------------------------------------------###  
 * here the module is listening to a msg from ground wich change the mode to LAUNCH MODE
 * while that doesnt happen it reads all the data and sends it to ground
 * 
 * When the data is good enought and the conditions for launch are met, the msg can be sent trough lora and the module will automaticaly switch to LAunch mode
 * 
 */
 
   unsigned long  ms= 400;
int MODE_FLAG = 0;
unsigned long tempo_lora = millis(); 
Serial.println("heyyy");
while(MODE_FLAG==0)                                                                                                                
{
 unsigned long startlora = millis(); 
  do {
    
    while (Serial1.available()>0)
  
  {
    
    leitura = Serial1.readStringUntil('\n');Serial.print(leitura);
  if(leitura== "RED"){guardaSD("Entrei em Launch Mode!");delay(500);envia_LORA("Entrei em Launch Mode!");leitura=""; MODE_FLAG = 1; break;}

  
  if(leitura == "RESET"){ guardaSD("REINICIA");delay(500);envia_LORA("VAI REINICAR");leitura=""; digitalWrite(resetPin, LOW);}
  
  }

  
  } while (millis() - startlora < ms);
  //if(Serial1.available()>0){leitura = Serial1.readStringUntil('\n');Serial.print(leitura);if(leitura== "RED"){Serial.println("OLAA");guardaSD("Entrei em Launch Mode!");delay(500);envia_LORA("Entrei em Launch Mode!");leitura="";break;}} //listen to lora msgs
  
  faz_leitura_BNO( &accelerometerData, &orientationData);
  retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
  retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z); 
  retira_dados_GPS(&latitude, &longitude);
  bmp280.getCurrentAltitude(altitude_medida);
  tempo=get_gps_time();
  
  leitura = String(AcXf, 3 ) + " " + String(AcYf, 3) + " " + String(AcZf,3) + " " + String(latitude, 8) + " " + String(longitude, 8) + " " + String(altitude_medida, 3) + " " + String(0)+" " + String (rot_x, 3)+ " " + String (rot_y,3 )+ " " + String (rot_z,3)+" " +String(millis());
  
  guardaSD(leitura);  //Guarda os dados no cartão SD
  envia_LORA(leitura);  // Envia os dados para o Lora  

  if (Flag_flight == 1){envia_LORA("Encontrou Flight no SD");break;}

  
}

startMillis = millis();
tempo_aux_v = millis();
tempo_aux_h = millis();
t = millis();

/*###--------------------------------------------------------------LAUNCH MODE----------------------------------------------------------------###
after receiving a msg from ground the module enters in Launch mode
 - reads data from BNO
 - if there is an acelaration > threshold for a periodo of time enters in FLIGHT MODE
 - reads gps data
 - reads altitude data
 - send data to ground
 - saves data in SD card 

 */
 envia_LORA("LAUNCH MODE!!!!");
 if (Flag_flight != 1){
  while(1) 
  {   
             
    faz_leitura_BNO( &accelerometerData, &orientationData);
    retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
    retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z);
    tempo=get_gps_time();   
    if((millis() - startMillis) > Minimo_temp_flight) { guardaSD("FLIGHT  " + tempo);envia_LORA("Entrei no Flight Mode!");starttime=1;counter=1;  break;}else if (accelModule( AcXf, AcYf, AcZf) == false){startMillis = millis();  tempo_aux_v = millis();tempo_aux_h = millis();t = millis();
}
    retira_dados_GPS(&latitude, &longitude);    
    bmp280.getCurrentAltitude(altitude_medida);  
    leitura = String(AcXf, 3 ) + " " + String(AcYf, 3) + " " + String(AcZf,3) + " " + String(latitude, 8) + " " + String(longitude, 8) + " " + String(altitude_medida, 3) + " " + String(0)+" " + String (rot_x, 3)+ " " + String (rot_y,3 )+ " " + String (rot_z,3)+" " +tempo; 
    envia_LORA(leitura);  // Envia os dados para o Lora   
    guardaSD("0 " + leitura);  //Guarda os dados no cartão SD
    unsigned long startlora = millis(); 
    do {while (Serial1.available()>0){leitura = Serial1.readStringUntil('\n');Serial.print(leitura);if(leitura== "RESET"){guardaSD("REINICIA");delay(500);envia_LORA("VAI REINICAR");leitura=""; digitalWrite(resetPin, LOW);}}} while (millis() - startlora < ms);
 
  }
}

else {
 
  unsigned long start = millis();
  Serial.print("tempo ejacaocaic");
  Serial.println(tempo_delay_ejecao);
  if (tempo_delay_ejecao < 0){
      ejecao = 1; iniciar_ejecao();
      envia_LORA("Ejectou tempo negativo");  
      Serial.println(tempo_delay_ejecao);
    Serial.println("negativo");
  }

  else{

    
  do { 

    Serial.println("cenass");
    faz_leitura_BNO( &accelerometerData, &orientationData);
    retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
    retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z);
     retira_dados_GPS(&latitude, &longitude);
    tempo=get_gps_time();   
     bmp280.getCurrentAltitude(altitude_medida);  
    leitura = String(AcXf, 3 ) + " " + String(AcYf, 3) + " " + String(AcZf,3) + " " + String(latitude, 8) + " " + String(longitude, 8) + " " + String(altitude_medida, 3) + " " + String(0)+" " + String (rot_x, 3)+ " " + String (rot_y,3 )+ " " + String (rot_z,3)+" " +tempo; 
    envia_LORA(leitura);  // Envia os dados para o Lora   
    guardaSD("1 " + leitura);  //Guarda os dados no cartão SD
    
  }while (millis() - start <tempo_delay_ejecao); 
  envia_LORA("Ejectou tempo positivo"); 
  ejecao = 1;     
  Serial.println("Positivo");
  iniciar_ejecao();}}}



void loop() 
{
  //Funcoes do main
  //Obter aceleracoes
  
  faz_leitura_BNO( &accelerometerData, &orientationData);
  retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
  retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z);
  retira_dados_GPS(&latitude, &longitude);//Valores de altitude longitude e latitude
  bmp280.getCurrentAltitude(altitude_medida); //Obtem o valor da altitude
  //Eliminar ruido, valores depois de serem filtrados pela forma canonica de kalman
  delta = (millis() - t)/1000;
  t=millis();
  if (m>m0)
  {                                                         //Se ainda houver combustivel (m > m0), a massa do rocket vai diminuindo
      mfr=0.33;                                             
      m=m-mfr*delta;                                        //À medida em que se vai queimando o combustivel         
  }
  else
  {                                                         //Se já não houver combustivel não há mass flow rate e a força do motor será 0
    mfr=0;
  }  
  acel_vert = obter_acel_vert(AcXf, AcYf, AcZf, rot_x, rot_y, rot_z);                                   //Calcura a aceleração vertical //Por aqui aquela cena da matriz rotacao
  v = filtro(acel_vert, mfr, m, &P, &P2, v,&h, altitude_medida, delta, haux);                          //v e' a velocidade corrigida. h e' a altitude corrigida.
  haux=h;                                                                                                 //Faz a correcção dos valores da velocidade com o filtro
  retira_dados_GPS(&latitude, &longitude);  //Obtem coordenadas do GPS

    if ((millis() - startMillis) >= period && ejecao == 0)     //Se ultrapassou o tempo
    {
      ejecao=1;iniciar_ejecao(); guardaSD("EJETOU condicao de tempo");  
    }
  
    //Caso o rocket não atinja a altitude estimada, verificar se está no processo de descida pelos valores do acelerómetro (10 instantes de amostragem)
    
    /*
    if(v<=0 && ejecao == 0 && ( millis()- startMillis) > tempo_seguranca) 
    {                                                                          //a escolha do eixo da condiçao depende de como o acelerometro estiver no circuito
    counter_acel++;
    }
    else 
    {  
      counter_acel=0;
    }
    if(ejecao == 0 && counter_acel== it_v_ejecao)
    {
      ejecao = 1;
      iniciar_ejecao();
    }
    */  
    if(v>5 || (millis()-startMillis)<=tempo_seguranca)     //se v<5 durante tempo_v_ejecao, então ejeta
    {    
      tempo_aux_v=millis();
    }
    if(ejecao == 0 &&(( millis()-tempo_aux_v) >= tempo_v_ejecao) && (millis()-startMillis)>tempo_seguranca)
    {  
      ejecao = 1;iniciar_ejecao();guardaSD("EJETOU condicao de velocidade");  
    }
    
    if((millis()-startMillis)<=tempo_seguranca)
    {
      tempo_aux_h=millis();
    }
    if(ejecao==0 && (millis()-startMillis)>tempo_seguranca)    //se h < altitude maxima ate aí durante tempo_h_ejecao, então ejeta
    {  
      if(h>altitude_max)
      {
        tempo_aux_h=millis();
        altitude_max=h;
      }
      if(ejecao == 0 && ((millis()-tempo_aux_h )>= tempo_h_ejecao))
      {
       
        ejecao = 1; iniciar_ejecao();guardaSD("EJETOU condicao altitude");  
      }
    }
  char tempo_string[5] ;
  sprintf(tempo_string, "%05d",millis()-startMillis);     
   
leitura = String(AcXf, 3 ) + " " + String(AcYf, 3) + " " + String(AcZf,3) + " " + String(latitude, 8) + " " + String(longitude, 8) + " " + String(altitude_medida, 3) + " " + String(v, 3)+" " + String (rot_x, 3)+ " " + String (rot_y,3 )+ " " + String (rot_z,3)+" " +tempo_string; 
guardaSD("1 "+ leitura);                                                        //Guarda os dados no cartão SD
envia_LORA(leitura);                                                       // Envia os dados para o Lora
}
