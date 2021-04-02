#include "Aurora.h"

//Valores Ejecção

unsigned long startMillis; 
unsigned long starttime=0;
byte ejecao = 0;
byte counter = 0;
byte counter_acel = 0;

//Valores Cartão SD

String nome_ficheiro;

//Valores BNO

double AcXf, AcYf, AcZf,rot_x,rot_y,rot_z;
sensors_event_t orientationData , accelerometerData;

//GPS

double latitude;
double longitude;

//Altimetro

float altitude_medida;
String leitura;
float m=7,m0=5.8;
float t = millis();
float v=0, h=0, P = 0, P2=0;
float delta;
float mfr;
float acel_vert;
float acel_corrigida;



void setup()
{
  Wire.begin();                                                                                                           //Começa a comunicação i2c
  incializa_BNO();
  inicializa_LORA();
  inicializa_altimetro();
  inicializa_GPS();
  //nome_ficheiro = inicializa_SD();
  
  
  while(1)                                                                                                                //Ciclo de StandBy
  {
    if(Serial.available())
    {
      if(Serial.readString() == "Launch Mode")
      {
        //guardaSD("Entrei em Launch Mode!");
        envia_LORA("Entrei em Launch Mode!");
        break;
      }
    }
  
      faz_leitura_BNO( &accelerometerData, &orientationData);
      retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
      retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z); 
      
      //Valores de altitude longitude e latitude
    
      retira_dados_GPS(&latitude, &longitude);
  
      //Obtem o valor da altitude
      
      get_altitude(&altitude_medida);  
    
      leitura = String(AcXf, 5 ) + " " + String(AcYf, 5) + " " + String(AcZf,5) + " " + String(latitude, 5) + " " + String(longitude, 5) + " " + String(altitude_medida, 3) + " " + String(0);
      
      envia_LORA(leitura);  // Envia os dados para o Lora    
    }

  startMillis = millis();
  
  while(1)                                                                                                                // Launch Mode!
  {            
    faz_leitura_BNO( &accelerometerData, &orientationData);
    retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
    retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z); 
  
    if((millis() - startMillis) > Minimo_temp)
    {
      //guardaSD("Entrei no Flight Mode!");
      envia_LORA("Entrei no Flight Mode!");
      starttime=1;
      counter=1; 
      
      break;
    }
    else if (accelModule( AcXf, AcYf, AcZf) == false)
    {
      startMillis = millis();
    }

    retira_dados_GPS(&latitude, &longitude);

    //Obtem o valor da altitude
    
    get_altitude(&altitude_medida);
  
    /*  leitura = String(AcXf, 5 ) + " " + String(AcYf, 5) + " " + String(AcZf,5) + " " + String(lati, 5) + " " + String(longi, 5) + " " + String(altitudeGps, 5) + " " + String(altitude_medida, 3) + " " + String(0);*/
    
    envia_LORA(leitura);  // Envia os dados para o Lora   
  }
}



void loop() 
{
  //Funcoes do main
  //Obter aceleracoes
  
  faz_leitura_BNO( &accelerometerData, &orientationData);
  retira_dados_acc(&accelerometerData, &AcXf,&AcYf, &AcZf);
  retira_dados_rota(&orientationData, &rot_x, &rot_y, &rot_z); 
  
  //Valores de altitude longitude e latitude
  
  retira_dados_GPS(&latitude, &longitude);
  
  //Obtem o valor da altitude
  
  get_altitude(&altitude_medida);
  
  //Eliminar ruido, valores depois de serem filtrados pela forma canonica de kalman
  
  delta = millis() - t;
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
      
  acel_vert = (-AcXf+sin((PI/2))*sin(PI/2));                                    //Calcura a aceleração vertical //Por aqui aquela cena da matriz rotacao
  v = filtro(acel_vert, mfr, m, &P, &P2, v,&h, delta);                          //Faz a correcção dos valores da velocidade com o filtro
  

  //Obtem coordenadas do GPS

  retira_dados_GPS(&latitude, &longitude);

  Serial.print(latitude,5);
  Serial.print("N\t");
  Serial.print(longitude,5);
  Serial.println("W");
    
  //Se ultrapassou o tempo
  
  if ( ejecao == -1)
  {
    if ((millis() - startMillis) >= period && ejecao == 0)
    {
      ejecao=1;
      iniciar_ejecao(); 
    }
  
    //Caso o rocket não atinja a altitude estimada, verificar se está no processo de descida pelos valores do acelerómetro (10 instantes de amostragem)
    
    if(v<=0 && ejecao == 0 && ( millis()- startMillis) > tempo_seguranca) 
    {                                                                          //a escolha do eixo da condiçao depende de como o acelerometro estiver no circuito
    counter_acel++;
    }
    else 
    {  
      counter_acel=0;
    }
    if(ejecao == 0 && counter_acel== numero_amostragens_ejecao)
    {
      ejecao = 1;
      iniciar_ejecao();
    }

    /*  leitura = String(AcXf, 5 ) + " " + String(AcYf, 5) + " " + String(AcZf,5) + " " + String(lati, 5) + " " + String(longi, 5) + " " + String(altitudeGps, 5) + " " + String(altitude_medida, 3)+ " "+ String(v,3) ; //Compila os dados numa só string
    */ 
    
    //guardaSD(leitura);                                                        //Guarda os dados no cartão SD
    envia_LORA(leitura);                                                       // Envia os dados para o Lora
  } 
}
