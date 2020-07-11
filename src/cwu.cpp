
/* 
*******************************************************************************************
  PROJECT: cwu3 (main.cpp)
  AUTHOR: Andrzej Łęgosz
  PURPOSE: IoT for Aroniowa
    URL:
    thanks to: 
  Relased to the  private domain
  HISTORY:  0.0: 2019-05-01 initial version
            0.2: 2019-05-05   SD form eMon
            0.3: client wifi, oled
            0.4: virtuino
            0.5: PlatformIO
            0.6: Dallas multi 18B20, BMP280 || BME280
            0.7: OLED multi screen, rebuild CWU steering
            0.8: fixed IP adress 10.14.0.20
            0.9: extra condition for cwu pump & floor pump
            1.0: 2019-06-28, AM2320 instead DHT11, modem nRF20
            1.1: reset Windows 10, changed plate Wemos to 80-7D-3A-7F-ED-1E, 10.14.0.23
            1.2: filling fireplace tank
            1.3: own library alTimers.h
            1.4: rearrangement FLOOR, ELV, ...
            1.5: divide into program section cwu_xxx.h 20190818
            1.6: czujniki temp na kolektorach podłogówki i na wymienniku ciepła z kominka
            1.7: 20190914 new VirtuinoCM
            1.8: domoticz
            1.9: hassio (mqtt) 2020-07-05, 2020-07-11
*******************************************************************************************
*/
/*
ESP.reset() is a hard reset and can leave some of the registers in the old state which can lead to problems,
            its more or less like the reset button on the PC.
ESP.restart() tells the SDK to reboot, so its a more clean reboot, use this one if possible.
*/

  #define COUNT_OF(x) 			((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

  #include <Arduino.h>
//------------------------------------------------------------------------------------
// enable Serial Debug, aby wyłaczyć komunikaty na PC, "zakomentuj" linię
//------------------------------------------------------------------------------------
  #define SERIAL_DEBUG
//#define DEBUG_PRINT_AM
//#define DEBUG_PRINT_BME
//#define DEBUG_PRINT_DALLAS
//#define DEBUG_VIRTUINO

//------------------------------------------------------------------------------------
// DEFINE CONSTANS
//------------------------------------------------------------------------------------
  #define _VERSION "CWU 1.9" //wersja programu
  #define CE_PIN 16     //Wemos D1 mini D0, nFR CE pin
  #define SCL_PIN 5     //Wemos D1 mini D1, I2C SCL
  #define SDA_PIN 4     //Wemos D1 mini D2, I2C SDA
  #define DALLAS_PIN 0  //Wemos D1 mini D3, oneWire pin
  #define CSN_PIN 2     //Wemos D1 mini D4, buildin LED, nFR CSN pin
  #define SCK_PIN 14    //Wemos D1 mini D5, SPI SCK
  #define MISO_PIN 12   //Wemos D1 mini D6, SPI MISO
  #define MOSI_PIN 13   //Wemos D1 mini D7, SPI MOSI
  #define SS_PIN 15     //Wemos D1 mini D8, SD Card CS pin

//------------------------------------------------------------------------------------
// DEFINES FOR WIFI
//------------------------------------------------------------------------------------
/*
 * płytka Wemos w kotłowni: home router DHCP Client: Client Name= ESP_7FED1E, MAC Address= 80-7D-3A-7F-ED-1E
 * rezerwacja w DHCP routera adresu klienta 10.14.0.23
 */
  #include <ESP8266WiFi.h>
  #include <ESP8266HTTPClient.h>
  #include <PubSubClient.h>

  #include <credentials.h>

  const int   watchdog = 60000; // Fréquence d'envoi des données à Domoticz - Frequency of sending data to Domoticz
  unsigned long previousMillis = millis(); 
  HTTPClient http;

  WiFiClient client;
  WiFiClient espClient;


  void callback(char* topic, byte* payload, unsigned int length);

  PubSubClient mqttClient(mqttServer_ip, mqttServer_port, callback, espClient);

//adresy serwerów z czujnikami
  IPAddress server[] =  
  { 
    IPAddress(10, 14, 0, 15),   //adres serwera - płytka Wemos w górnej łazience z czujnikiem PIR
    IPAddress(10, 14, 0, 16)    //do późniejszego wykorzystania
  };
  String answer;    //string odpowiedź z serwera 10.14.0.15
  String SubStr[5]; //składowe odpowiedzi z serwera
/*
  SubStr[0]: IP serwera (Wemos górna łazienka)
  SubStr[1]: sygnał WiFi serwera
  SubStr[2]: temperatura
  SubStr[3]: wilgotność
  SubStr[4]: PIR
 */

//------------------------------------------------------------------------------------
// DEFINES FOR TIME
//------------------------------------------------------------------------------------
  #include <TimeLib.h> // https://github.com/PaulStoffregen/Time

//------------------------------------------------------------------------------------
// DEFINES FOR RTC DS1307
//------------------------------------------------------------------------------------
  #include "D1_class_DS1307.h"
  #define TIMEZONE  1
  DS1307 ds1307;  

//------------------------------------------------------------------------------------
// DEFINE FOR PCF8574
//------------------------------------------------------------------------------------
  #include "PCF8574.h"  // https://github.com/xreef/PCF8574_library
  PCF8574 pcf8574(0x20);
  #define OutPin_first    0
  #define OutPin_last     4
  #define OutPin_hindmost 7 //najdalszy, ostatni
  #define VALVE_PIN       2 //elektrozawór uzupełniający wodę w zbiorniku kominka
  #define CWU_PIN         3 //pompa CWU
  #define LED_PIN         4 //led info
  #define FLOOR_PIN       7 //pompa podłogówki

//------------------------------------------------------------------------------------
// DEFINE TIMERS_AL
//------------------------------------------------------------------------------------
//patrz komentarze w pliku alTimers.h
  uint64_t timeTbl[] = 
  {
    59999UL   //timer[0]: czas do następnego zapisu na karcie SD
    ,2150UL   //timer[1]: czas do następnego odczytu czujników
    ,2351UL   //timer[2]: czas do następnego połączenia z serwerem w górnej łazience
    ,1030UL   //timer[3]: czas do następnego wyświetlenia na ekranie OLED
    ,10000UL  //timer[4]: czas do zmiany numeru wyświetlanego ekranu OLED
    ,400000UL //timer[5]: czas załączenie pompy cwu [milisek]
    ,900000UL //timer[6]: czas blokady załączenie pompy cwu [milisek]
    ,500UL    //timer[7]: opóźnienie do wykonania próby ponownego połączenia WiFi
    ,18000UL  //timer[8]: timeout na następny telegram z nFR
    ,300000UL //timer[9]: timeout napełniania zbiornika wody nad kominkiem
    ,60000UL  //timer[10]: czas do następnej wysyłki do domoticza
  };

  const uint8_t amount_timers = COUNT_OF(timeTbl);  //ilość timers_al

//------------------------------------------------------------------------------------
// FUNCTION DECLARATIONS
//------------------------------------------------------------------------------------
  void RELY();

//------------------------------------------------------------------------------------
// GLOBAL VARIABLES
//------------------------------------------------------------------------------------
  time_t LOOP_EXEC_TIME = 0;

//zmienne domoticza//cwu
  boolean PIR, inWorkingHour, PIR_holdStateOn;
  boolean CWU_pump, CWU_oldPump = false;
  unsigned long CWU_startTime, CWU_duration, CWU_total;
  unsigned int CWU_count;

  byte enabled_hours[][2] = //zakresy godzin, dla których PIR może załączać pompę cwu
  {
    //{5,6}     //<5, 6>
    {12,17}
    //,{8,10}  //test
    ,{18,20}  //<18, 20>
    ,{21,23}  //<21, 23>
  };
  const int enabled_hours_rows = sizeof(enabled_hours) / sizeof(byte) / 2;  //ile wierszy ma tabela enable_hours, dzielenie przez 2 albowiem są dwa "byte" w wierszu!

  float sanitary_temp = 60.0; //temperatura "antylegionelloza"
  boolean DisablePIR;         //flaga blokująca ponowne załączenie pompy cwu przez PIR - przez czas timer[6]

//"podłogówka"
  float floor_start = 40.0F;                    //uruchomienie pompy "podłogówki"
  float floor_hist = 5.0F;                      //histereza "podłogówki"
  float floor_stop = floor_start - floor_hist;  //wyłączenie "podłogówki"
  boolean FLOOR_pump = false;                   //pompa "podłogówki"
  unsigned long FLOOR_startTime, FLOOR_duration, FLOOR_total;
  unsigned int FLOOR_count;

//zbiornik wyrównawczy kominka
  uint16_t tank_deep = 250U;  //od powierzchni czujki ultradźwiękowej do dna zbiorniczka [mm]
  float Level_min = 50.0F;
  float Level_max = 100.0F;
  boolean oldELV, ELV = false, ELV_timeout_error = false;

//pomiar ciśnienia CO
  float pressureSamples = 20.0F; //uśrednianie odczytu


//byte SDCARD_ERR = 0;


//------------------------------ koniec nagłówka programu ------------------------------------------------------





                      //------------------------------------------------------------------------------------
                      // streaming for PRINT
                      //------------------------------------------------------------------------------------
  template<class T> inline Print &operator <<(Print &obj, T arg)
  {
    obj.print(arg);
    return obj;
  }

  #define endl '\n'

  #ifdef SERIAL_DEBUG
    #define SLN(x)  Serial.println(x)
    #define SLNF(x) Serial.println(F(x))
    #define SS(x)   Serial.print(x)
    #define SSF(x)  Serial.print(F(x))
  #else
    #define SLN(x)
    #define SLNF(x)
    #define SS(x)
    #define SSF(x)
  #endif

// ********************** sekcje programu *****************************
  #include <Timers_al.h>

  #include <cwu_OTA.h>
  #include <cwu_SENSORS.h>
  #include <cwu_NRF.h>
  #include <cwu_SD.h>
  #include <cwu_OLED.h>
  #include <cwu_NTP.h>
  #include <cwu_VIRTUINO.h>
  //#include <cwu_domoticz.h>
  #include <cwu_mqtt.h>
  #include <cwu_WIFI.h>


void SETUP_PRINT()
{
  SSF("\n");                          SLN(ARDUINO_BOARD);  
  SSF("Connected to wifi (mySSID): ");SLN(WiFi.SSID());       //nazwa sieci WiFi
  SSF("Status: ");                    SLN(WiFi.status());     // Network parameters
  SSF("IP: ");                        SLN(WiFi.localIP());
  SSF("MAC: ");                       SLN(WiFi.macAddress());
  SSF("Host name: ");                 SLN(WiFi.hostname());   //nazwa Wemos'a
  SSF("Subnet: ");                    SLN(WiFi.subnetMask());
  SSF("Gateway: ");                   SLN(WiFi.gatewayIP());
  SSF("Signal: ");                    SLN(WiFi.RSSI());
  SSF("Networks: ");                  SLN(WiFi.scanNetworks());
  SLN();
}//SETUP_PRINT()

void GET_FROM_SERVER()
{//string pobrany z serwera
  SSF("answer= ");  SS(answer);
  byte len=answer.length();
  SSF("\tlen= ");   SLN(len);
  byte i, last_i = 0, j = 0;
  /*
  dane zawarte w stringu odebranym z serwera
  SubStr[0]: IP serwera
  SubStr[1]: wartość sygnału serwera
  SubStr[2]: temperatura z czujnika serwera
  SubStr[3]: wilgotność z czujnika serwera
  SubStr[4]: sygnał PIR
   */
  for(i = 0; i < len; i++)//wyszukiwanie substringów przedzielonych przecinkami
  {
    if(answer.charAt(i) == ',')//czy bieżący znak to przecinek?
    {
      SubStr[j] = answer.substring(last_i, i);
      last_i = i + 1;
      SSF("\t"); SS(SubStr[j]);
      j++;
    }
  }
  SubStr[j] = answer.substring(last_i);
  SSF("\t");  SLN(SubStr[j]);
  PIR = (SubStr[4].charAt(0) == '1');//stan czujnika PIR z górnej łazienki
  arySensor[5] = SubStr[2].toFloat();//temperatura z górnej łazienki
  arySensor[6] = SubStr[3].toFloat();//wilgotność z górnej łazienki
}//GET_FROM_SERVER()

void CONN_SERVER1()//wywoływany przez Timer[2]
{
    client.connect(server[0], 80);          // connects to the server
    SSF("\nconnecting to server: "); SLN(server[0]);
    pcf8574.digitalWrite(LED_PIN, LOW);     // to show the communication only (inverted logic)
    client.println("Haliho server!\r");     // trigger message to the server, its value is scrapped
    answer = client.readStringUntil('\r');  // received the server's answer
    client.flush();
    GET_FROM_SERVER();

    RELY();//sterowanie wyjściem pompy CWU
//SSF("\nStan wy PCF CWU= ");SLN(pcf8574.digitalRead(CWU_PIN));
    SSF("LOOP_EXEC_TIME= "); SS(LOOP_EXEC_TIME);  SLN();
    pcf8574.digitalWrite(LED_PIN, HIGH);
}//CONN_SERVER1()

                      //------------------------------------------------------------------------------------
                      // SECTION RTC
                      //------------------------------------------------------------------------------------
time_t DS1307_GET() //dla setSyncProvider(DS1307_GET);
{
  return ds1307.getSecs1970();
}


                      //------------------------------------------------------------------------------------
                      // SECTION SENSORS READ AND DEBUG_PRINT
                      //------------------------------------------------------------------------------------
void AM2320_PRINT()
{
  AM2320_READ();
  #ifdef DEBUG_PRINT_AM
    SSF("AM2320\ttemperature= "); SS(String(arySensor[0], 1));
    SSF(" *C\thumidity= "); SS(String(arySensor[1], 1));  SSF(" %");
  #endif
/*
  SSF("\tHeatIndex= ");
  SS(String(dht.computeHeatIndex(arySensor[0], arySensor[1], false), 1));
  SSF("\tDewPoint= ");
  SS(String(dht.computeDewPoint(arySensor[0], arySensor[1], false), 1));
  
  SSF("\tAbsHumidity= ");
  SS(String(dht.computeAbsoluteHumidity(arySensor[0], arySensor[1], false), 1));
  */
  /*
  SSF("\tComfortRatio= ");
  float cr = dht.getComfortRatio(cf, temperature, humidity, false);
  SS(cr);
  SSF("\tPerception= ");
  SS(String(dht.computePerception(temperature, humidity, false), 1));
  */
  SLN();
}

void BME_PRINT() 
{
  BME_READ();
  #ifdef DEBUG_PRINT_BME
    SSF("BME280\ttemperature = "); SS(arySensor[2]); SSF(" *C,");
    #ifdef __BME280_H__
      SSF("\thumidity = "); SS(arySensor[3]); SLNF(" %,");
    #endif
    SSF("\tpressure = "); SS(arySensor[4]); SLNF(" hPa");
  #endif
}

void DALLAS_PRINT()
{
  DALLAS_READ();
  #ifdef DEBUG_PRINT_DALLAS
    SLNF("Temperature sensors Dallas 18B20:");
    for(byte i = 0; i < ds18count; i++)
    {
      SSF("\t");  if(i < 10) SSF(" ");
      SS(i);
      SSF(": "); SS(ds18float[i]); SSF(" C");
      if(!((i + 1) % 3)) SLN();
    }
    SLN();
  #endif
}//DALLAS_PRINT()

void PRESSURE_CO()
{
  //0.5V -> 0psi, 4.5V -> 100psi (6,9bar)
  //Pressure[psi] = 25[psi] * (Uanalog[V] - 0.5[V]), e.g.: Uanalog=3V -> Pressure[bar]=62.5[psi]=4.31[bar]
  //float currPressure = (float)map(analogRead(A0), 153, 967, 0, 431) / 100.0;
  int CO_sensor = map(analogRead(A0), 153, 967, 0, 370);
  if(CO_sensor < 0) //jeśli wartość <0: to błąd
  {
    CO_sensor = 0;
  }
  //uśrednianie
  arySensor[14] -= (arySensor[14] - (float)CO_sensor / 100.0F) / pressureSamples;
}//PRESSURE_CO()

void SENSORS_PRINT()
{
  SSF("\n***** Sensors READ & DEBUG_PRINT: ");
  SLN(ds1307.getDateTime2());
  AM2320_PRINT();
  bme.takeForcedMeasurement(); // has no effect in normal mode
  BME_PRINT();
  DALLAS_PRINT();
  PRESSURE_CO();  //odczyt ciśnienia wody CO
}

                      //------------------------------------------------------------------------------------
                      // SECTION CWU
                      //------------------------------------------------------------------------------------
/*
char* handleCmd(char cmd) {
  Serial.print("recived command= ");Serial.print(cmd); Serial.print(": ");
  switch(cmd) {
    case '1':
        digitalWrite(RELAY_PIN, LOW);
        return "Relay pin set to high\n";
    case '0':
        digitalWrite(RELAY_PIN, HIGH);
        return "Relay pin set to low\n";
    default:
        return "Send ASCII 1 for on, and 0 for off.\n";
  }
}//char* handleCmd(uint8_t cmd)
*/

void CWU_ON() //załączenie pompy cwu
{
  pcf8574.digitalWrite(CWU_PIN, LOW);
  if(CWU_pump == 0)
  {
    CWU_startTime = millis();
    CWU_count += 1; //zwiększenie licznika załączeń pompy cwu
    CWU_pump = 1;
    TIMER[6].active = true; //timer disable cwu pump after activity
    DisablePIR = true;
    CWU_oldPump = CWU_pump;
  }
  TIMER[5].active = true; //timer cwu

}//CWU_ON()

void CWU_OFF()//wyłączenie pompy cwu
{
  pcf8574.digitalWrite(CWU_PIN, HIGH);
  CWU_pump = 0;
  CWU_oldPump = CWU_pump;
  CWU_duration = millis() - CWU_startTime;  //ostatni czas pracy pomy cwu
  CWU_total += CWU_duration;                //suma czasów pracy pompy cwu
  SSF("CWU pump duration= "); SS(CWU_duration); SLNF(" ms");
  TIMER[5].active = false;  //timer cwu
  vRly = false;
  vRly_clr = true;  //wyłaczenie rozkazu z Virtuino
}//CWU_OFF()

void CWU_DISABLE()
{
  TIMER[6].active = false;//timer cwu_disable
  DisablePIR = false;
}//CWU_DISABLE()

boolean WORKING_HOUR()//sprawdzenie, czy to godziny możliwej pracy cwu
{
  boolean ret = false;
  for(byte i = 0; i < enabled_hours_rows; i++)
  {
    ret = ret || ((hour() >= enabled_hours[i][0]) && (hour() <= enabled_hours[i][1]));
  }
  //ret = ret && !DisablePIR;
  return ret;
}//WORKING_HOUR()

void RELY()//sterowanie pompą cwu, wywołanie z CONN_SERVER1
{
  inWorkingHour = WORKING_HOUR();  
  boolean cwu = (ds18float[0] > sanitary_temp) || (PIR && inWorkingHour && !DisablePIR);

  if(!vRly && vRly_old && CWU_pump)
  {
    CWU_OFF();
  }
  if((cwu || vRly) && !CWU_pump)
  {
    CWU_ON();
  }
  vRly_old = vRly;
}//RELY()

                      //------------------------------------------------------------------------------------
                      // SECTION FLOOR
                      //------------------------------------------------------------------------------------
void FLOOR_ON()
{
  pcf8574.digitalWrite(FLOOR_PIN, LOW);
  SS(ds1307.getDateTime2()); SLNF("\tPodłogówka załączona ");
  if(FLOOR_pump == 0)
  {
    FLOOR_startTime = millis();
    FLOOR_count += 1; //zwiększenie licznika załączeń pompy podłogówki
    FLOOR_pump = 1;
  }
}//FLOOR_ON()

void FLOOR_OFF()
{
  pcf8574.digitalWrite(FLOOR_PIN, HIGH);
  SS(ds1307.getDateTime2()); SLNF("\tPodłogówka wyłączona ");
  if(FLOOR_pump == 1)
  {
    FLOOR_pump = 0;
    FLOOR_duration = millis() - FLOOR_startTime;  //ostatni czas pracy pomy cwu
    FLOOR_total += FLOOR_duration;                //suma czasów pracy pompy cwu
    SSF("Floor pump duration= "); SS(FLOOR_duration); SLNF(" ms");
  }
}//FLOOR_OFF()

void FLOOR()  //pompa podłogówki
{
  //tempState=1 po wzroście temp powyżej zadanej aż do spadku poniżej min (histereza)
  bool tempState = (ds18float[6] > (floor_start * !FLOOR_pump) + (floor_stop * FLOOR_pump));
  if((tempState || vFloor) && !FLOOR_pump)
  {
    FLOOR_ON();
  }
  if(!tempState && !vFloor && FLOOR_pump)
  {
    FLOOR_OFF();
  }
}//FLOOR()

                      //------------------------------------------------------------------------------------
                      // SECTION FIREPLACE
                      //------------------------------------------------------------------------------------
void FIREPLACE_ELV_TIMEOUT()//TIMER[9]
{
  ELV_timeout_error = true; //był timeout napełniania zbiornika nad kominkiem
  TIMER[9].active = false;
  SS(ds1307.getDateTime2()); SLNF("\t**** Timeout Elektrozaworu");
}//FIREPLACE_ELV_TIMEOUT()

void FIREPLACE_ELV()
{
  //levelState=1 po spadku poziomu poniżej min aż do wzrostu poziomu powyżej max (histereza)
  boolean levelState = ((arySensor[9] < ((Level_min * !ELV) + (Level_max * ELV))) && radioNRFStatus);

  if(!ELV_timeout_error && levelState)
  {
    ELV = true;
  }
  else
  {
    ELV = false;
  }

  if(ELV!=oldELV)
  {
    SS(ds1307.getDateTime2()); SSF("\t**** Elektrozawór = ");SLN(ELV);
    TIMER[9].active = ELV;  //wyłączenia timera timeout'u otwarcia elektrozaworu
    oldELV=ELV;
  }

  //sterowanie stanem przekaźnika z Virtuino
  if(vElv==1)
  {
    ELV=!ELV;
  }
  else
  {
    ELV=ELV;
  }
  
//załączenie elektrozaworu poziomem niskim
  pcf8574.digitalWrite(VALVE_PIN, !ELV);


}//FIREPLACE_ELV()



                      //------------------------------------------------------------------------------------
                      // SECTION MAIN
                      //------------------------------------------------------------------------------------
void setup()
{
//serial
  #ifdef SERIAL_DEBUG
    Serial.begin(115200);     //begin Serial comm
  #endif
  SLNF("\n");

//CS karty SD
  pinMode(SS_PIN, OUTPUT);

  VIRTUINO_INI();
  
//PCF dodatkowe we/wy binarne
  pcf8574.begin();
  //PCF piny jako wyjścia  
  for(uint8_t i = OutPin_first; i <= OutPin_last; i++)
  {
    pcf8574.pinMode(i, OUTPUT);
    pcf8574.digitalWrite(i, HIGH);
  }
    pcf8574.pinMode(OutPin_hindmost, OUTPUT);
    pcf8574.digitalWrite(OutPin_hindmost, HIGH);
  //PCF piny jako wejścia
  for(uint8_t i = OutPin_last + 1; i < OutPin_hindmost; i++)
  {
    pcf8574.pinMode(i, INPUT);
  }
//wstępne zamknięcie elektrozaworu
  NFR_RESET_STATUS();
  DATA_FROM_NFR.Sensor.id = 0;
  vElv = 0;

//RTC - ustawienie czasu RTC płytki  
  setTime(ds1307.getSecs1970());
  SLN(ds1307.getDateTime2());
  
//timery
  TIMER[0].handler = SD_CARD;               TIMER[0].time_const = timeTbl[0]; TIMER[0].active = true;
  TIMER[1].handler = SENSORS_PRINT;         TIMER[1].time_const = timeTbl[1]; TIMER[1].active = true;
  TIMER[2].handler = CONN_SERVER1;          TIMER[2].time_const = timeTbl[2]; TIMER[2].active = true;
  TIMER[3].handler = OLED_PRINT;            TIMER[3].time_const = timeTbl[3]; TIMER[3].active = true;
  TIMER[4].handler = OLED_NEXT_SCR;         TIMER[4].time_const = timeTbl[4]; TIMER[4].active = false;
  TIMER[5].handler = CWU_OFF;               TIMER[5].time_const = timeTbl[5]; TIMER[5].active = false;
  TIMER[6].handler = CWU_DISABLE;           TIMER[6].time_const = timeTbl[6]; TIMER[6].active = false;
  TIMER[7].handler = RECONNECT;             TIMER[7].time_const = timeTbl[7]; TIMER[7].active = false;
  TIMER[8].handler = NFR_RESET_STATUS;      TIMER[8].time_const = timeTbl[8]; TIMER[8].active = false;
  TIMER[9].handler = FIREPLACE_ELV_TIMEOUT; TIMER[9].time_const = timeTbl[9]; TIMER[9].active = false;
//  TIMER[10].handler = toDomoticz;           TIMER[10].time_const = timeTbl[10]; TIMER[10].active = true;
  TIMER[10].handler = toHassio;             TIMER[10].time_const = timeTbl[10]; TIMER[10].active = true;

//czujniki
  Wire.begin();
  AM_2320.begin();

  bool statusBME = bme.begin(0x76);
  if (!statusBME) 
  {
    SLNF("Could not find a valid BME280 sensor, check wiring!");
  }
  //scenario weather monitoring
  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                  Adafruit_BME280::SAMPLING_X1, // temperature
                  Adafruit_BME280::SAMPLING_X1, // pressure
                  Adafruit_BME280::SAMPLING_X1, // humidity
                  Adafruit_BME280::FILTER_OFF   );

/*
//BMP280 Default settings from datasheet.
  bme.setSampling(Adafruit_BMP280::MODE_NORMAL,     // Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     // Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      // Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); // Standby time.
*/

  DALLAS_INI();

//pierwszy odczyt czujnika ciśnienia CO [bar]
  arySensor[14] = (float)map(analogRead(A0), 153, 967, 0, 431) / 100.0;
  if(arySensor[14] < 0)//jeśli wartość <0: to błąd
  {
    arySensor[14] = 0;
  }

//wyświetlacz OLED
  OLED_INI();

//komunikacja
  WIFI_CONNECTING();
//UDP
  udp.begin(localPort);
  SSF("\nStarting UDP, local port: "); SLN(udp.localPort());

//odczyt NTP
  NTP_READ();
  setSyncProvider(DS1307_GET); //ustawienie aktaulizacji RTC płytki z DS1307

//radio nFR
  RadioNRF24.begin(address, CHANNEL_NR, CSN_PIN, CE_PIN);

//pozostałe procesy
  TIMER[4].active = true; //uruchominie sekwencji wyświetlań OLED
  SETUP_PRINT();  //wydruk OLED 1 ekranu
  OTA_SERVICES(); //inicjalizacja funkcji OTA
  delay(0);       //delay(0) causes a yeild for ESP8266 (wydajność)

//inicjalizacja mqtt
  setup_Mqtt();

}//setup()



void loop()
{
  time_t start = millis();  //do pomiaru czasu wykonania pętli LOOP
  delay(0);

  ArduinoOTA.handle();      //obsługa OTA

  timer_al();               //polling timerów, funkcja timer() z pliku <Timers_al.h>, na początku loop()
  
  VIRTUINO_VAR();           //obsługa zmiennych virtuino

  RECONNECT();              //czy utracono WiFi?

  FLOOR();                  //sterowanie pompą podłogówki

  RADIO_NRF();              //odczyt danych z modułu nRF20

  FIREPLACE_ELV();          //sterowanie elz zbiorniczka kominka

  shortStatesForDomoticz();

  LOOP_EXEC_TIME = millis() - start;  //czas wykonania pętli LOOP

}//loop()

