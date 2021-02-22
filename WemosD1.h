//MODEL PAMETNE KUCE TEMELJEN NA MIKROKONTROLERU
//DIPLOMSKI RAD
//MATKO BUTKOVIC

//INICIJALIZACIJA KNJIZNICA POTREBNIH ZA RAD SUSTAVA
//Knjiznica za spajanje na WiFi
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Inicijalizacija Ticker knjiznice putem predprocesorskog poziva
#ifdef useTicker
  #include <Ticker.h> 
#endif

//Inicijalizacija knjiznice za I2C komunikaciju
#include <Wire.h>

//Postavljanje imena mreže i lozinke
const char* ssid = "WLANF928B6";
const char* password =  "mamatata14";
//Postavljanje MQTT protokola: ime servera i port, korsnicko ime i lozinka
const char* mqttServer = "farmer.cloudmqtt.com";
const int mqttPort = 18184;
const char* mqttUser = "ltdaniau";
const char* mqttPassword = "vWkhn8NvLHGl";

//Deklaracija varijabli za I2C komunikaciju
char receiveBuffer[255];
char sendingBuffer[20];
char sensorsBuffer[20];

//Deklaracija varijabli za statuse alarma vatre, pokreta i stanje ventilatora
int fanStatus = 0;
int fireStatus = 0;
int alarmStatus = 0;
int fanStatusOld = 0;
int fireStatusOld = 0;
int alarmStatusOld = 0;
int fireDetected = 0;
int moveDetected = 0;

//Deklaracija varijable za spremanje imena teme
String receivedTopic;

//Deklaracija varijable za temperaturu i vlagu
char tempByte[6];
char humByte[6];

/*
union u_tag {
   byte b[4];
   float fval;
} u;
*/
//Inicijalizacija Wemosa D1 mini kao client-a
WiFiClient espClient;
PubSubClient client(espClient);

//Vremenski okvir slanja i primanja poruka
#ifdef useTicker
  Ticker readSensors;
  Ticker sendSensors;
#endif
#ifndef useTicker
  unsigned long maxSensorsDelay = 10000;
  unsigned long maxSendDelay = 60000;
  unsigned long SensorsDelay;
  unsigned long SendDelay;
#endif

//FUNKCIJA SETUP
void setup() {

  //Inicijalizacija ugradene LED diode
  pinMode(LED_BUILTIN, OUTPUT);
  //Pocetak rada Serial monitora
  Serial.begin(115200);
  //Inizijalizacija I2C komunikacijske sabirnice
  Wire.begin(D1, D2); //Wemos D1 mini - SDA=D1 i SCL=D2
  //Metoda za pokretanje spajanja na WIFI parametrima: ime mreze, lozinka  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  //Obavijest da je Wemos D1 mini spojen na WIFI mrezu
  Serial.println("Connected to the WiFi network");
  
  //Spajanje na MQTT server parametrima: adresa MQTT servera i oznaaka porta
  client.setServer(mqttServer, mqttPort);
  //Postavljanje callback funkcije
  client.setCallback(callback);
  //Pokusaj spajanja na MQTT server
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
	//Obavijest da je Wemos D1 mini spojen na MQTT server
    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {
		
		Serial.println("connected");
	}	  
	
	//Obavijest da je Wemos D1 mini spojen na MQTT server
    else { 
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  #ifdef useTicker
    // Svakim 5 sekundi se salje zahtjev za očitanjem senzora prema MEGI2560
    readSensors.attach_ms(5000, sensorReading); 
    // Svake minute se salje očitanje senzora
    sendSensors.attach_ms(60000, mqttSendSensors);
  #endif
  
  #ifndef useTicker
    SendDelay = millis(); 
    SensorsDelay = millis(); 
  #endif
  
  //client.publish("sensors/tx/text", "Hello from ESP8266");
  //Wemos D1 mini se pretplaćuje na sve teme koje počinju sa sensors/tx/
  client.subscribe("sensors/rx/#");
  //Wemos D1 mini objavljuje poruke na sljedece teme - inicijalizacija početnih vrijednosti - sve ugaseno
  client.publish("sensors/rx/fireStatus", "0");
  client.publish("sensors/rx/fanStatus", "0");
  client.publish("sensors/rx/alarmStatus", "0");
}

//Funkcija za slanje vrijednosti temperature i vlage na MQTT Cloud
void mqttSendSensors(){
  client.publish("sensors/tx/temperature",tempByte);
  client.publish("sensors/tx/humidity",humByte);
  }

//Funkcija za citanje poruka sa MEGA2560 pločice
void sensorReading()
{
  //Svake sekunde se izvršava
  digitalWrite(LED_BUILTIN, !(digitalRead(LED_BUILTIN)));  //Promjena stanja ugrađene LED diode
  Wire.requestFrom(8, 15); // zahtjev za porukuom sa adrese 8 veličine 15 B
  if(Wire.available()>14){
    for(int i=0;i<15;i++){
     sensorsBuffer[i] = Wire.read();
     Serial.print("Data ");
     Serial.print(i);
     Serial.print(" : ");
     Serial.println(sensorsBuffer[i], HEX);
    }

    tempByte[0] = sensorsBuffer[0];
    tempByte[1] = sensorsBuffer[1];
    tempByte[2] = sensorsBuffer[2];
    tempByte[3] = sensorsBuffer[3];
    tempByte[4] = sensorsBuffer[4];
	//tempByte[5] = sensorsBuffer[5];
    
    humByte[0] = sensorsBuffer[5];
    humByte[1] = sensorsBuffer[6];
    humByte[2] = sensorsBuffer[7];
    humByte[3] = sensorsBuffer[8];
    humByte[4] = sensorsBuffer[9];

    fanStatus = sensorsBuffer[10] - 1;
    fireStatus = sensorsBuffer[11] - 1;
    alarmStatus = sensorsBuffer[12] - 1;
    fireDetected = sensorsBuffer[13] - 1;
    moveDetected = sensorsBuffer[14] - 1;
	
	//humByte[5] = sensorsBuffer[11];
    
    Serial.println(tempByte);
    Serial.println(humByte);    
  
   //If petlja koja provjerava ukoliko je detekcija pokreta aktivirana
   if(alarmStatus){
     //If petlja koja provjerava ukoliko je pokret primjećen na Arduino strani
     if(moveDetected){
        client.publish("sensors/tx/move", "1"); //Objavljuje se poruka na MQTT Cloud da je pokret detektiran
      }
      else{
        client.publish("sensors/tx/move", "0");
        }
    }
   //If petlja koja provjerava ukoliko je detekcija vatre aktivirana
   if(fireStatus){
     //If petlja koja provjerava ukoliko je vatra primjećena na Arduino strani
     if(fireDetected){
        client.publish("sensors/tx/fire", "1"); //Objavljuje se poruka na MQTT Cloud da je vatra detektirana
      }
      else{
        client.publish("sensors/tx/fire", "0");
        }
    }
  }
}
 
 //Postavljanje funkcije callback
void callback(char* topic, byte* payload, unsigned int length) {
 
  Serial.print("Pristigla poruka na temu: ");
  Serial.println(topic);
 
  Serial.print("Poruka:");
  for (int i = 0; i < length; i++) {
    receiveBuffer[i] = payload[i];
    Serial.print((char)payload[i]);
  }

  Serial.println();
  Serial.println("-----------------------");
  
  //Ovisno o temi u koju je stigla poruka izvrsi promjenu statusnih varijabli
  receivedTopic = topic;
  if(receivedTopic.equals("sensors/rx/alarmStatus")){
    Serial.print("alarmStatus: "); 
    alarmStatus = receiveBuffer[0] - '0';
    sendingBuffer[0] = 0x03;
    sendingBuffer[1] = alarmStatus + 1;
    
    Wire.beginTransmission (8);
    Wire.write (sendingBuffer,2);
    Wire.endTransmission ();
    
    Serial.println(alarmStatus);
    }
	
  else if(receivedTopic.equals("sensors/rx/fireStatus")){
    Serial.print("fireStatus: ");
    fireStatus = receiveBuffer[0] - '0';
    sendingBuffer[0] = 0x02;
    sendingBuffer[1] = fireStatus + 1;
    
    Wire.beginTransmission (8);
    Wire.write (sendingBuffer,2);
    Wire.endTransmission ();
    Serial.println(fireStatus);
    }
	
  else if(receivedTopic.equals("sensors/rx/fanStatus")){
    Serial.print("Fan: ");
    fanStatus = receiveBuffer[0] - '0';
    sendingBuffer[0] = 0x01;
    sendingBuffer[1] = fanStatus + 1;
    
    Wire.beginTransmission (8);
    Wire.write (sendingBuffer,2);
    Wire.endTransmission ();
    Serial.println(fanStatus);
    }   
}

void checkFanStatus(){
  if( fanStatusOld != fanStatus ){
    //Izvrsava se samo jednom ukoliko je došlo do promjene statusne varijable 
    if(fanStatus){
       client.publish("sensors/rx/fanStatus", "1"); //Ventilator aktiviran
      }
    else{
       client.publish("sensors/rx/fanStatus", "0");
      }
    fanStatusOld = fanStatus;
    }  
  }

void checkFireStatus(){
  if( fireStatusOld != fireStatus ){
    if(fireStatus){
       client.publish("sensors/rx/fireStatus", "1"); //Detektor vatre aktiviran
      }
    else{
       client.publish("sensors/rx/fireStatus", "0");
      }
    fireStatusOld = fireStatus;
    }
  }  

void checkAlarmStatus(){
  if( alarmStatusOld != alarmStatus ){
    if(alarmStatus){
       client.publish("sensors/rx/alarmStatus", "1"); //Detektor pokreta aktiviran
      }
    else{
       client.publish("sensors/rx/alarmStatus", "0");
      }
     alarmStatusOld = alarmStatus;
    }
  }
 
 //Glavna petlja programa
void loop() {
  client.loop();
  checkFanStatus();
  checkFireStatus();
  checkAlarmStatus();
  #ifndef useTicker
    if (millis()-SendDelay > maxSendDelay){
        SendDelay = millis();
        mqttSendSensors();
      }
    if (millis()-SensorsDelay > maxSensorsDelay){
        SensorsDelay = millis();
        sensorReading(); 
      }
  #endif
}
