//MODEL PAMETNE KUCE TEMELJEN NA MIKROKONTROLERU
//DIPLOMSKI RAD
//MATKO BUTKOVIC

//INICIJALIZACIJA KNJIZNICA POTREBNIH ZA RAD SUSTAVA
//Knjiznica za upravljanje LCD ekranom
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

//Knjiznica za Senzor DHT11 (Temperature i Vlage)
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>

//Deklaracija DHT11 pina kao A0 i modela senzora kao DHT 11
#define DHTPIN A0
#define DHTTYPE    DHT11 

//Deklaracija pinova ultrazvučnog senzora HCSR04
#define trigPin 12
#define echoPin 13

//Deklaracija pina tipkala i LED diode upravljanje tipkalom, mikrofonom i fotosenzorom
#define inPin 3
#define LIGHT 7
#define ledMic 8
#define LED 11

//Deklaracija pina za relej
#define outReleyPin 4

//Deklaracija pina za aktivnu zujalicu
#define buzzerPin 10

////Deklaracija pina senzora vatre
#define analogInPinFlame A1

//Deklaracija pina fotosenzora
#define analogInPinBright A2

//Deklaracija pina za mikrofon
#define microphonePin A3

//Deklaracija RGB LED diode
#define redpin A5   //Pin za crvenu vrijednost LED diode
#define bluepin A7  //Pin za plavu vrijednost LED diode
#define greenpin A6 //Pin za zelenu vrijednost LED diode

//INICIJALIZACIJA
//Inicijalizacija LCD ekrana
LiquidCrystal_I2C lcd(0x27,16,2);  // Kao parametre funkcija prima LCD adresu, broj znakova po liniji, broj linija

//Podesavanje pina DHT11 Senzora
DHT_Unified dht(DHTPIN, DHTTYPE); //Kao parametre funkcija prima DHT11 pin, i model senzora

//Inicijalizacija delay varijable u milisekundama
uint32_t delayMS = 0;

//Inizijalizacija varijabli za detektor pokreta-ultrazvucni senzor
float olddistance = 0;
float newdistance = 0;
float duration = 0;
float distance = 0;

//Inicijalizacija varijabli sensora plamena
int sensorFlameValue = 0;       
int outputFlameValue = 0;
int FireValue = 0;

//Inicijalizacija fotosenzora
int sensorBrightValue = 0;     
int outputBrightValue = 0;
int BrightValue = 0;

//Inicijalizacija mikrofona - SoundDetection varijabla i prag
int soundValue = 0;
int soundTrigger = 0; //Treshold vrijednost za aktivaciju

//Inizijalizacija varijabli za koritenje tipkala
int reading;
static bool led_state = false;    // trenutno stanje LED diode
static bool ledMic_state = false; // trenutno stanje LED diode

//I2C komunikacija i statusne varijable
char responseBuffer[20];
char receiveBuffer[20];
char humByte[6];
char tempByte[6];
int fanStatus = 0;
int fireStatus = 0;
int alarmStatus = 0;
int fanStatusOld = 0;
int fireStatusOld = 0;
int alarmStatusOld = 0;
int fireDetected = 0;
int moveDetected = 0;

//Inicijalizacija varijable potrebne za zujalicu
int i = 0;

//FUNKCIJA SETUP
void setup()
{
  //Pocetak rada Serial monitora
  Serial.begin(9600);
  
  lcd.init();                      //Inicijalizacija LCD ekrana
  dht.begin();                    //Inicijalizacija DHT11 senzora temperature i vlage

  //Inicijalizacija I2C komunikacija
  Wire.begin(8);                // Inicijalizacija I2C sabirnice sa adresom
  Wire.onReceive(receiveEvent); // Inicijalizacija funkcije za dolazne poruke
  Wire.onRequest(requestEvent); // Inicijalizacija funkcije za odlazne poruke
  
  //Ispis pozdravne poruke na LCD ekran
  lcd.backlight();
  lcd.print("Smart House");

  //Inizijalizacija pina kao output pin za LED diodu
  pinMode(LED, OUTPUT);
  
  //Inicijalizacija PINova RGB LEDice
  pinMode(redpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  
  //Inicijalizacija LEDice upravljane tipkalom
  pinMode(LIGHT,OUTPUT);

  //Inicijalizacija ultrazvučnog HCSR04 senzora
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //Inicijalizacija tipkala
  pinMode(inPin, INPUT);

  //Inicijalizacija releja
  pinMode(outReleyPin, OUTPUT);

  //Inicijalizacija zujalice
  pinMode(buzzerPin, OUTPUT);

  //Inicijalizacija mikrofona
  pinMode(microphonePin, INPUT);

  //Inicijalizacija LED diode upravljane mikrofonom
  pinMode(ledMic, OUTPUT);

  //Inicijalizacija pocetnih vrijednosti
  delayMS = 500;
  soundTrigger = 70;
  FireValue = 150;
  BrightValue = 650;
}

//Glavna petlja
void loop()
{
  //Delay između očitanja vrijednosti senzora
  delay(delayMS);

  //PROMIJENITA RGB LED DIODE OSNOVNO STANJE - plavo
  analogWrite(redpin, 0);
  analogWrite(bluepin, 255);
  analogWrite(greenpin, 0);

  //Provjera promjene statusa ventilatora
  if( fanStatusOld != fanStatus ){
    //Ispis na LCD ekran promjene statusa
     lcd.clear();
     lcd.print("Fan Status: ");
     lcd.print(fanStatus);
   //Spremanje novog statusa u old varijablu
     fanStatusOld = fanStatus;
   //Ispis varijable statusa na serial monitor radi provjere vrijednosti
     Serial.println("");
     Serial.print("fanStatus: ");
     Serial.print(fanStatus);
   //Signal RGB LED diode da je status promjenjen
     analogWrite(redpin, 143);
     analogWrite(bluepin, 255);
     analogWrite(greenpin, 0);
     delay(delayMS);
    }
  //Provjera promjene statusa senzora vatre
  if( fireStatusOld != fireStatus ){
   //Ispis na LCD ekran promjene statusa
     lcd.clear();
     lcd.print("Fire Status: ");
     lcd.print(fireStatus);
   //Spremanje novog statusa u old varijablu
     fireStatusOld = fireStatus;
   //Ispis varijable statusa na serial monitor radi provjere vrijednosti
     Serial.println("");
     Serial.print("fireStatus: ");
     Serial.print(fireStatus);
   //Signal RGB LED diode da je status promjenjen
     analogWrite(redpin, 255);
     analogWrite(bluepin, 0);
     analogWrite(greenpin, 0);
     delay(delayMS);
    }
  //Provjera promjene statusa detektora pokreta
  if( alarmStatusOld != alarmStatus ){
   //Ispis na LCD ekran promjene statusa
     lcd.clear();
     lcd.print("Alarm Status: ");
     lcd.print(alarmStatus);
   //Spremanje novog statusa u old varijablu
     alarmStatusOld = alarmStatus;
   //Ispis varijable statusa na serial monitor radi provjere vrijednosti
     Serial.println("");
     Serial.print("alarmStatus: ");
     Serial.print(alarmStatus);
   //Signal RGB LED diode da je status promjenjen
     analogWrite(redpin, 0);
     analogWrite(bluepin, 0);
     analogWrite(greenpin, 255);
     delay(delayMS);
    }
  
  //Ispis vrijednosti varijable fanStatus
     Serial.println("");
     Serial.print("fanStatus: ");
     Serial.print(fanStatus);
  //Kontrola ventilatora
  if(fanStatus){
    digitalWrite(outReleyPin, LOW); //Ventilator upaljen
    }
  else{
    digitalWrite(outReleyPin, HIGH); //Ventilator ugasen
    }

  //Citanje promjene vrijednosti signala na tipkalu
  delay(200);
  reading = digitalRead(inPin);
     Serial.println("");
     Serial.print("inPin: ");
     Serial.print(inPin);
  //Ukoliko je tipkalo pritisnuto, invertiraj LED diodu
  if(reading == 0){
  Serial.println(reading);
    if(led_state){
      led_state = false;
      digitalWrite(LIGHT, LOW);
    }
    else{
      led_state = true;
      digitalWrite(LIGHT,HIGH);
    }
    delay(50);
  }
  
  //digitalWrite(outPin, stat);
  //lcd.setBacklight(stat);
  //previous = reading;

  //Citanje sa mikrofona i upravljanje ledicom putem zvuka
  soundValue = analogRead(A3);
  Serial.println("");
  Serial.print("Očitanje mikrofona: ");
  Serial.print(soundValue);
  //Ukoliko je doslo do ocitanja senzora veceg od tresholda, i unutar granica promjeni stanje LED diode
   if (soundValue > soundTrigger && soundValue <100 && soundValue>50) {       
      if (ledMic_state) {
        ledMic_state = false;         
        digitalWrite(ledMic, LOW);
        Serial.println("LED dioda ugašena zvukom"); 
      }
      else {
        ledMic_state = true;
        digitalWrite(ledMic, HIGH);    
        Serial.println("LED dioda upaljena zvukom");
      }
      delay(50); 
  }

  //Funkcija za detekciju pokreta HCSR04 senzora
  if(alarmStatus){
  digitalWrite(trigPin, HIGH);
  //delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  //Promjenu izmedu dva mjerenja udaljenosti spremamo kao postotak
  newdistance = (distance/olddistance)*100;  
  olddistance = distance;
  //Detekcija pokreta nema ukoliko je promjena manja od 30% i udaljenost preko 200cm
  if (newdistance >= 30 || distance <= 0 || distance > 200){  
     Serial.println("Nema detekcije pokreta");
     Serial.println("");
     Serial.print("Distance: ");
     Serial.print(distance);
     moveDetected = 0;
  }
  //Funkcija else{} koja se izvrsava ukoliko je doslo do detekcije pokreta(promjene udaljenosti)
  else {
    while(newdistance <= 30){
     Serial.println("Detekcija pokreta");
     Serial.println("");
     Serial.print("Distance: ");
     Serial.print(distance);
     lcd.clear();
     //Alarm pokreta ispis na ekran
     lcd.print("DETEKCIJA POKRETA");
     //Promjena statusa moveDetected varijable
     moveDetected = 1;
     //Alarm pokreta - RGB ledica svijetli zeleno
     analogWrite(redpin, 0);
     analogWrite(bluepin, 0);
     analogWrite(greenpin, 255);
    //Ton zujalice prilikom detekcije pokreta
     for(i=0;i<120;i++){
        digitalWrite(buzzerPin,HIGH);
        delay(3);
        digitalWrite(buzzerPin,LOW);
        delay(3);
    }
  delay(delayMS);
  //Ponovna provjera ocitanja senzora
  digitalWrite(trigPin, HIGH);
  //delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  newdistance = (distance/olddistance)*100;  
  olddistance = distance;
      }
    }
  }
    
  //Fotosenzor
  sensorBrightValue = analogRead(analogInPinBright);
     Serial.println("");
     Serial.print("sensorBrightValue: ");
     Serial.print(sensorBrightValue);

   if(sensorBrightValue > BrightValue ){
    Serial.println("");
    Serial.print("NOC");
    digitalWrite(LED, HIGH);
    //lcd.setBacklight(1);
    //digitalWrite(outReleyPin, LOW);
    }
    else{
         Serial.println("");
         Serial.print("DAN");
         digitalWrite(LED, LOW);
         //lcd.setBacklight(0);
         //digitalWrite(outReleyPin, HIGH);
    }
      
  //Senzor vatre 
  if(fireStatus){
  //Ocitanje vrijednosti senzora vatre i ispis na Serial monitor
  sensorFlameValue = analogRead(analogInPinFlame);
  Serial.println("");
  Serial.print("sensorFlameValue = ");
  Serial.print(sensorFlameValue);
  fireDetected = 0;
        if(sensorFlameValue < FireValue ){
          fireDetected = 1;
          Serial.println("");
          Serial.print("Detekcija Vatre");
             lcd.clear();
             lcd.print("DETEKCIJA VATRE");
              //Alarm vatre RGB LED dioda svijetli crveno
             analogWrite(redpin, 255);
             analogWrite(bluepin, 0);
             analogWrite(greenpin, 0);
      //Generiranje zvuka na zujalici trajanja 200ms
             for(i=0;i<200;i++){
                digitalWrite(buzzerPin,HIGH);
                delay(1);//wait for 1ms
                digitalWrite(buzzerPin,LOW);
                delay(1);//wait for 1ms
             }
             delay(delayMS);
             /*while(sensorFlameValue < FireValue) {
        fireDetected = 1;
        sensorFlameValue = analogRead(analogInPinFlame);
             }*/
        }
  }
    
  //Ocitanje vrijednosti DHT11 senzora
  sensors_event_t event;
  //Spremanje vrijednosti temperature
  dht.temperature().getEvent(&event);
  //Ukoliko temperatura nije ocitana sa senzora
    if (isnan(event.temperature)) {
          Serial.println(F("Greska prilikom očitanja temperature!"));
          lcd.clear();
          lcd.write("Greska 102");
          delay(delayMS);
       }
  //Ispis temperature na Serial monitor i LCD ekran
      else {
        Serial.println("");
        Serial.println(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
        float temp = event.temperature;
        lcd.clear();
        //lcd.setCursor(0, 0);
        lcd.print("Temp.: ");
        lcd.print(temp);
        lcd.print(" C");
        //Ukoliko je teperatura iznad 35 stupnjeva automatski upali ventilator
        if(temp>35){
          fanStatus = 1;
        }
        //Ukoliko je teperatura ispod 10 stupnjeva automatski ugasi ventilator
        else if (temp<10){
           fanStatus = 0;
          }
        //Ukoliko je temperatura izmedu 10-35 stupnjeva od aplikacije dolazi vrijednost upali/ugasi ventilatora
        
    //I2C komunikacija
  //Pripremanje podataka temperature za slanje na Wemos D1 mini
    dtostrf(temp,0, 2, tempByte);
    for (int i=0;i<5;i++){
        responseBuffer[i] = tempByte[i];
      }
    Serial.print("Temp: ");
    Serial.print(responseBuffer[0], HEX); // -
    Serial.print(" ");
    Serial.print(responseBuffer[1], HEX); // 1
    Serial.print(" ");
    Serial.print(responseBuffer[2], HEX); // 1
    Serial.print(" ");
    Serial.print(responseBuffer[3], HEX); // .
    Serial.print(" ");
    Serial.print(responseBuffer[4], HEX); // 1
    Serial.print(" ");
    Serial.print(responseBuffer[5], HEX); // 1
    Serial.println("");
  }
  
  //Spremanje vrijednosti vlage
  dht.humidity().getEvent(&event);
   //Ukoliko blaga nije očitana javi gresku prilikom ocitanja
     if (isnan(event.relative_humidity)) {
          Serial.println(F("Greska prilikom očitanja vlage!"));
          lcd.clear();
          lcd.write("Greska 103");
          delay(delayMS);
      }
  //Ispis vrijednosti vlage na Serial monitor i LCD ekran
    else {
        Serial.println(F("Vlaga: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
        float hum = event.relative_humidity;
        lcd.setCursor(0, 1);
        lcd.print("Vlaga: ");
        lcd.print(hum);
        lcd.print(" %");

        //I2C komunikacija
    //Pripremanje podataka vlage za slanje na Wemos D1 mini
        dtostrf(hum,0, 2, humByte);
        for (int i=0;i<5;i++){
            responseBuffer[i+5] = humByte[i];
        }
        Serial.print("Humm: ");
        Serial.print(responseBuffer[5], HEX); // -
        Serial.print(" ");
        Serial.print(responseBuffer[6], HEX); // 1
        Serial.print(" ");
        Serial.print(responseBuffer[7], HEX); // 1 
        Serial.print(" ");
        Serial.print(responseBuffer[8], HEX); // .
        Serial.print(" ");
        Serial.print(responseBuffer[9], HEX); // 1
//      Serial.print(" ");
//      Serial.print(responseBuffer[11], HEX); //1
        Serial.println("");
  }
  responseBuffer[10] = fanStatus + 1;
  responseBuffer[11] = fireStatus + 1;
  responseBuffer[12] = alarmStatus + 1;
  responseBuffer[13] = fireDetected + 1;
  responseBuffer[14] = moveDetected + 1;
}

    //receiveEvent je funkcija koja se izvrsava svaki put kada Master uredaj-Wemos D1 mini posalje podatke
    void receiveEvent(int howMany) {
      //Unutar funkcije Serial.print metoda nije dozvoljena
      int i = 0;
        while (0 <Wire.available()) {
            char c = Wire.read();      //Primanje 8 bita podataka tipa char
            receiveBuffer[i] = c;
      //Serial.println(c,HEX);   //Ispis pročitane poruke
            i++;
    }
    //Serial.println();         
    if(receiveBuffer[0] == 0x01){
      fanStatus = int(receiveBuffer[1]) - 1;
    }
    else if(receiveBuffer[0] == 0x02){
      fireStatus = int(receiveBuffer[1]) - 1;
    }
    else if(receiveBuffer[0] == 0x03){
      alarmStatus = int(receiveBuffer[1]) - 1;
    }
    //    Serial.print("Fan status: ");
    //    Serial.println(fanStatus);
    //    Serial.print("Fire status: ");
    //    Serial.println(fireStatus);
    //    Serial.print("Alarm status: ");
    //    Serial.println(alarmStatus);
  }
        
    // Funkcija koja se izvrsava kada Master uredaj zahtjeva poruku 
    void requestEvent() {
        Wire.write(responseBuffer);  //Slanje stringa na Master
    }
    
