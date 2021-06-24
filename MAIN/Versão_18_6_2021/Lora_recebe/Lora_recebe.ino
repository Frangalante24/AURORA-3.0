


void setup() 
{

Serial.begin(9600);
Serial1.begin(9600);
Serial.setTimeout(10);
Serial1.setTimeout(10);
delay(1000);
//Serial.print("leu");
//Serial.println(" ax     ay     az     latitute    longitude   H     V rotx    roty  rotz   time");
}
void loop() {

 //String leitura;

  if (Serial1.available())
 {
  // Serial.print("leu:");
  //leitura = Serial1.readString();
  
  Serial.print(Serial1.readString());
 
  
 }
 
  if (Serial.available())
 {
  Serial1.println(Serial.readString());
 
  
 } 
 
 
}
