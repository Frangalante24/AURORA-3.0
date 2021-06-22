

/*---------------------------------------------------------------------------
                              Inicializa BNO
                              
Inicia a comunicação.
Verifica se a conecção foi bem sucedida.

---------------------------------------------------------------------------*/


void incializa_BNO(void)
{
  Serial.println("BNO init");
  
  if (!bno.begin())
  {
    // Problema a detectar o BNO055... Verificar as conecções
    Serial.print("Ooops, BNO055 nao detectado ... Verificar as coneccoes ou o endereco I2C!");
    incializa_BNO();
  }
  
  delay(1000);
}
/*---------------------------------------------------------------------------
                         Retira Dados da Aceleração
                         
Obtem os valores da aceleração para os eixos X, Y e Z.
Guarda os valoes.

---------------------------------------------------------------------------*/

void retira_dados_acc(sensors_event_t* event, double* a_x, double* a_y, double *a_z) 
{
*a_x = event->acceleration.x;*a_y= event->acceleration.y;*a_z= event->acceleration.z;
}
/*---------------------------------------------------------------------------
                         Retira Dados da Rotação
                         
Obtem os valores da rotação para os eixos X, Y e Z.
Guarda os valoes.

---------------------------------------------------------------------------*/

void retira_dados_rota(sensors_event_t* event, double* rot_x, double* rot_y, double *rot_z) 
{
    *rot_x = event->orientation.x;*rot_y = event->orientation.y;*rot_z = event->orientation.z;
}

/*---------------------------------------------------------------------------
                         Faz Leitura do BNO
                         
Le os valores da aceleração e da rotação obtidos pelo BNO.
Actualiza os valores.

---------------------------------------------------------------------------*/

void faz_leitura_BNO(sensors_event_t* accel_data, sensors_event_t* rot_data)
{
bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
}
/*---------------------------------------------------------------------------
                         Inicializa LORA
                         
Inicializa a comunicação entre o Arduino e o LORA.
Imprime uma mensagem de teste.

---------------------------------------------------------------------------*/
void inicializa_LORA(void)
{
  Serial1.begin(9600);pinMode(lora_aux, INPUT);envia_LORA("Lora Comms OK");delay(500);
}
/*---------------------------------------------------------------------------
                           Envia LORA
                         
Ordena o LoRa a enviar a string: a_enviar.

---------------------------------------------------------------------------*/
void envia_LORA(String a_enviar)
{
  if(digitalRead(lora_aux) != 0){Serial1.println(a_enviar);}
}
/*---------------------------------------------------------------------------
                           Inicializa GPS
                         
Inicializa a comunicação entre o Arduino e o GPS.
Inicializa o protocolo do GPS (GLL).

---------------------------------------------------------------------------*/
void inicializa_GPS (void)
{
  envia_LORA("Estou no GPS");
  delay(100);
  ss.begin(9600);
  delay(100);
  L76X_Send_Command(SET_POS_FIX_100MS);
  delay(100);
  L76X_Send_Command(SET_NMEA_OUTPUT);
  delay(100);
  L76X_Send_Command("$PMTK251,115200");
  delay(500);
  ss.begin(115200);
}
/*---------------------------------------------------------------------------
                            Retira Dados do GPS
                         
Retira os dados da latitude e da longitude obtidos pelo GPS.

---------------------------------------------------------------------------*/
static void retira_dados_GPS(double *localx,double *localy)
{ 
  unsigned long  ms= 150;
  unsigned long start = millis();
  do {while (ss.available()>0)if (gps.encode(ss.read())){*localx = gps.location.lat();*localy = gps.location.lng();}} while (millis() - start < ms);
}
/*---------------------------------------------------------------------------
                            Programa o gps
                         
funcoes para enviar comandos de programacao do gps

---------------------------------------------------------------------------*/

void L76X_Send_Command(char *data)
{
    char Check = data[1], Check_char[3]={0};
    uint8_t i = 0;
    ss.write('\r');
    ss.write('\n');
    for(i=2; data[i] != '\0'; i++){Check ^= data[i];}
    Check_char[0] = Temp[Check/16%16];
    Check_char[1] = Temp[Check%16];
    Check_char[2] = '\0';
    DEV_Uart_SendString(data);
    ss.write('*');
    DEV_Uart_SendString(Check_char);
    ss.write('\r');
    ss.write('\n');
}
void DEV_Uart_SendString(char *data)
{
  uint16_t i;
  for(i=0; data[i] != '\0'; i++){ss.write(data[i]);}
}

/*---------------------------------------------------------------------------
                           reinicia do arduino

---------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------
                          Inicializa o SD card

---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------
                          Inicializa o SD card

---------------------------------------------------------------------------*/
String inicializa_SD(void)
{
  String nome = "Teste_0.txt";                                  //Nome base do ficheiro
  int numero = 0;
  int flag = 0;
  File ficheiro; 
  pinMode(cardDetect, INPUT);                               //Bloqueia o Pino CardDetect do cartão SD no modo Input.
  envia_LORA("A inicializar SD");  
  while(!digitalRead(cardDetect))                           //Verifica se o  cartão está inserido no leitor.
  {
    envia_LORA("ERRO - Cartão SD NÃO detetado");
    delay(1000);
  }
  if(!SD.begin(chipSelect))
  {        
    envia_LORA("Incialização Falhada! - A tentar outra vez");
    digitalWrite(resetPin, LOW);
    return nome;    
   }
   
  while (SD.exists(nome))
  {
    nome = "Teste_";    
    ++numero;
    nome = String(nome + numero);
    nome = String (nome + ".txt");   
    ficheiro = SD.open(nome);
   
    int n=5; //nr de caracteres a ler a partir do fim
    long tempo_decorrido = 0; //miillliiii
    byte aux;
    
    if (flag ==0)
    {
      if (ficheiro) 
      {

        while (ficheiro.available()) 
        {
          String line =  ficheiro.readStringUntil('\n');
          
            if(line.indexOf("FLIGHT")!=-1)
          {

           ficheiro.seek(ficheiro.size()-n-2); //subtrai-se dois por causa dos caracteres de fim de linha e de ficheiro

             
            for(int k=1; k<=n ; k++)
            {
                    aux = ficheiro.read() - 48;
                     tempo_decorrido = tempo_decorrido*10 + aux;

          
            }


          
            tempo_delay_ejecao = (TEMPO_APOGEU - tempo_decorrido - 4000) ;
            
            Serial.println("ficheiiiro");
            Serial.println(tempo_delay_ejecao);
              
            flag=1;
            Flag_flight = 1;
            break;       
          }
          
           /*if(line.indexOf("FLIGHT")!=-1)
          {

            char * char_array2 = &line[0];
          
            long int tempo_FLIGHT_SD = converte_string_s(char_array2);
            long int tempo_SD_now =-1;
            
            while ((tempo_SD_now == -1) || (tempo_SD_now<60)){

              
                 retira_dados_GPS(&latitude, &longitude);
                 String aux_string = "FLIGHT   " + get_gps_time();
                 char * char_array = &aux_string[0];  
                 tempo_SD_now  = converte_string_s(char_array); 
                 Serial.print("tempo agora   "); 
                 Serial.print(aux_string); 
                      
            }
            tempo_delay_ejecao = (TEMPO_APOGEU - (tempo_SD_now - tempo_FLIGHT_SD)) ;
            
              
            flag=1;
            Flag_flight = 1;
            break;       
          }*/
        }
   
      ficheiro.close();
      } 
    else 
    {
        envia_LORA("Erro ao abrir ficheiro");
    }
    }
    }
   ficheiro = SD.open(nome, FILE_WRITE );
   if(ficheiro)
   {
     ficheiro.println("----------------AURORA 3.0 - Inicio de Dados-----------------\n");
     ficheiro.println("ax  ay  az latitute longitude altitude velocity rotx roty rotz time\n");
     
   }
   else{while(1){digitalWrite(resetPin, LOW);}} //deu erro reinicia o arduino
   envia_LORA("SD inicializado com Sucesso!");
   delay(500);
   ficheiro.close();
   return nome; 
}
/*---------------------------------------------------------------------------
                            Guarda SD
                              
Guarda uma string na proxima linha do ficheiro 

---------------------------------------------------------------------------*/

void guardaSD(String a_guardar)
{
  File ficheiro;
  if(digitalRead(cardDetect)){ficheiro = SD.open(nome_ficheiro, FILE_WRITE);if(ficheiro){ficheiro.println(a_guardar);ficheiro.close();}}
}
/*---------------------------------------------------------------------------
                            Modulo da acelaracao
                              

take_off, instantes seguidos em que o modulo dos paramentros do acelerometro é maior que 3    
---------------------------------------------------------------------------*/

bool accelModule(float AcXf, float AcYf, float AcZf) { //pode-se usar apenas um dos eixos do acelerometro
  bool take_off = false;
 
  if (sqrt(pow(AcXf,2)+pow(AcYf,2)+pow(AcZf,2)) > Minimo_acc) {take_off = true;}else {take_off= false; } return take_off;
}
/*---------------------------------------------------------------------------
                            Initializa altimetro bmp280
                              

---------------------------------------------------------------------------*/

void inicializar_bmp(void)
{
  Serial.println("bmp init");
  bmp280.begin(BMP280_I2C_ALT_ADDR);                                 // Default initialisation, place the BMP280 into SLEEP_MODE 
  bmp280.setPresOversampling(OVERSAMPLING_X4);    // Set the pressure oversampling to X4
  bmp280.setTempOversampling(OVERSAMPLING_X4);    // Set the temperature oversampling to X1
  bmp280.setIIRFilter(IIR_FILTER_16);              // Set the IIR filter to setting 4
  bmp280.setTimeStandby(TIME_STANDBY_05MS);     // Set the standby time to 2 seconds
  bmp280.setSeaLevelPressure(1010.01f);
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE  
}
/*---------------------------------------------------------------------------
                            Ejecao

Quando esta funcao e executada a carga de ejecao e libertada e o paraqueda e libertado
                           
---------------------------------------------------------------------------*/

void iniciar_ejecao()
{
  digitalWrite(sendPin, HIGH);   //Arde o Nicrómio
  ejecao = -1;
}






String get_gps_time(void){


    tempo="";
    if (gps.time.isValid())
    {
    if (gps.time.hour() < 10) tempo+="0";
    tempo+=String(gps.time.hour());
    tempo+=":";
    if (gps.time.minute() < 10) tempo+="0";
    tempo+=String(gps.time.minute());
    tempo+=":";
    if (gps.time.second() < 10)  tempo+="0";
    tempo+=String(gps.time.second());
    tempo+=":";
    if (gps.time.centisecond() < 10) tempo+="0";
    tempo+=String(gps.time.centisecond());
  } 
  //Serial.println(tempo);
  return tempo;
 
  
}









//Rotação do vetor aceleração
//return: aceleração vertical após rotaçãofloat obter_acel_vert(float acx, float acy, float acz, float rotx, float roty, float rotz){
  
/*---------------------------------------------------------------------------
                            acelaracao vertical


                           
---------------------------------------------------------------------------*/
float obter_acel_vert(float acx, float acy, float acz, float rotx, float roty, float rotz){
  
  float pi = 3.14159265359;
  float off_x=-pi/2;
  float off_y=0;      //offsets calculados com o inicializador (por fazer)
  float off_z=0;

  float off_acel_x=0;
  float off_acel_y=0;     //offsets a calcular conforme o comportamento do MPU
  float off_acel_z=0;

  float a_x=acx;
  float a_y=acz; //acel medidas pelo MPU a cada intervalo
  float a_z=-acy;

  float gyro_x=rotz*(pi/180);
  float gyro_y=roty*(pi/180); //gyro medidos pelo MPU a cada intervalo
  float gyro_z=rotx*(pi/180);

  float gyro[3];

  gyro[0] = gyro_x;
  gyro[1] = gyro_y;
  gyro[2] = gyro_z;

  float phi = gyro_x+off_x; //rot em torno de X em rads
  float theta = gyro_y+off_y; //rot em torno de Y
  float psi = gyro_z+off_z; //rot em torno de Z

  float R[3][3];

  R[0][0]= cos(theta)*cos(psi);
  R[0][1]= sin(theta)*sin(phi)*cos(psi)+cos(phi)*sin(psi);
  R[0][2]= sin(phi)*sin(psi)-cos(phi)*sin(theta)*cos(psi);

  R[1][0]= -cos(theta)*sin(psi);
  R[1][1]= cos(phi)*cos(psi)-sin(phi)*sin(psi)*sin(theta);
  R[1][2]= cos(phi)*sin(theta)*sin(psi)+sin(phi)*cos(psi);

  R[2][0]= sin(theta);
  R[2][1]= -sin(phi)*cos(theta);
  R[2][2]= cos(phi)*cos(theta);
  
  float a[3];
  a[0] = a_x;
  a[1] = a_y;
  a[2] = a_z;
  float b[3]; //rotacao do vetor aceleracao

  int i = 0;
  int j=0;
  float soma;

  for (i=0;i<3;i++)
  {
      soma=0;
      for (j=0;j<3;j++)
      {
          soma = soma+ R[i][j] * a[j];
      }
      b[i]=soma;
  }

  float acel_x_terra=b[0]-off_acel_x;  //atenção aos sinais dos offsets - verificar
  float acel_y_terra=b[1]-off_acel_y;  //aceleracoes no referencial da terra
  float acel_z_terra=b[2]-off_acel_z;  //aceleracao em z é a importante para os cálculos do apogeu - ejeção

  Serial.println("acelaraca");
  Serial.println(acel_z_terra -9.81);
  return (acel_z_terra-9.81);
  
}
/*---------------------------------------------------------------------------
                           Filtro de kalman

                 
---------------------------------------------------------------------------*/
float filtro(float acel_vert, float mfr, float m, float* P, float* P2, float v, float* alt, float h_m, float delta, float h_ant) {




    float v1,v2;
    float aux1,aux2,aux3,densidade;
    float Ve,L,M,p0,R0,temperatura;
    float g=9.81, A=0, C=1, K, K2;
    float Fa,Fm;
    float acel_corrigida,a_prediction, h_prediction, P_prediction,residual;
    float Q=800, R=1000, Q2 = 2, R2 = 400;//corrigir o R em funcao do ruido (quadrado do desvio padrao = cov)
    
    Ve=1186;
    L = 0.0065;
    M = 0.0289654;
    p0 = 101325;
    R0 = 8.31447;
    temperatura = 25 + 273.15;

    aux1 = (p0 * M) / (R0 * (temperatura));
    aux2 = 1 - ((L * h_m) / (temperatura));
    aux3 = ((g * M) / (R0 *L)) - 1;
    densidade = aux1 * (pow(aux2,aux3)); //E' suposto ser aux2^aux3;
  
    Fm = mfr * Ve; //*sin(pitch) ou assim
    Fa = 0.4*densidade*(v)*(v)* 0.0063617251*0.5; 
    if (v<0)
        Fa=-Fa;
  
    a_prediction = Fm/m - Fa/m - g;
    residual = acel_vert - C*a_prediction;
    if (residual>50)
        Q=800/(cosh(0.035*(residual-50)));
    else if (residual<-50)
        Q=800/(cosh(0.035*(residual+50)));
    else
        Q = 800;
  
    P_prediction = A*(*P)*A + Q;
    K = (P_prediction * C)/(C * P_prediction * C + R);
    acel_corrigida = a_prediction + K*residual;
    *P = (1 - K*C)*P_prediction; 

    v1 = v + acel_corrigida*delta;

    h_prediction = h_ant + v*delta + 0.5*delta*delta*acel_corrigida;
    residual = h_m - h_prediction;
    P_prediction = 1*(*P2) + Q2;
    K2 = (P_prediction)/(P_prediction + R2);
    *alt = h_prediction + K2*residual;
    *P2 = (1-K2)*P_prediction;

    v2 = (*alt - h_ant)/delta;

    v = 0.65*v1 + 0.35*v2;
  
   return v;
  
}
