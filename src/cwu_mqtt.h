/* cwu_mqtt.h

*/

const char* mqttClient_name = F("boiler_room");

void callback(char* topic, byte* payload, unsigned int length)
{
    /*
  SSF("Message arrived [");
  SS(topic);
  SSF("] ");
  for (int i = 0; i < length; i++) 
  {
    SS((char)payload[i]);
  }
  SLNF();
  */
}

#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
char topic[MSG_BUFFER_SIZE];
long qty_mqttReconnect;

void shortStatesForDomoticz()
//"zatrzaśnięcie" stanu HIGH czujnika PIR do momentu wysłania do domoticza. po "wysyłce" stan zostaje uaktualniony
{
  if(PIR) PIR_holdStateOn=PIR;
}


boolean mqttReconnect() {
  if (mqttClient.connect(mqttClient_name, mqttServer_name, mqttServer_pass)) 
  {
    // Once connected, publish an announcement...
    snprintf (msg, MSG_BUFFER_SIZE, "Mqtt connected %s", ds1307.getDateTime2().c_str());
    SLN(msg);
  }
  return mqttClient.connected();
}

void setup_Mqtt()
{
  SSF("Attempting MQTT connection... ");
  if (mqttClient.connect(mqttClient_name, mqttServer_name, mqttServer_pass))
  {
      snprintf (msg, MSG_BUFFER_SIZE, "Mqtt connected %s", ds1307.getDateTime2().c_str());
      SLN(msg);
      mqttClient.publish(F("Cwu/status/connected"), ds1307.getDateTime2().c_str(), true);
      mqttClient.publish(F("Cwu/status/rssi"), String(WiFi.RSSI()).c_str(), true);//wifi RSSI
      mqttClient.subscribe(F("Cwu/inTopic"));
  }
  else
  {
    SSF("failed :(, rc="); SLN(mqttClient.state());    
  }
  
}


void toHassio()
{
  //czy klient mqqtt podłączony?
  if (!mqttClient.connected()) 
  {
    //próba ponownego połączenia klienta mqtt
      if (mqttReconnect()) 
      {
        qty_mqttReconnect++;
      }
  } 
  SSF("Total Qry MQTT reconnected:");SLN(qty_mqttReconnect);
  if (mqttClient.connected()) 
  {
    //klient mqtt podłączony
    mqttClient.loop();
    mqttClient.subscribe(F("Cwu/inTopic"));

/*
formatowanie ciądu dla snprintf:
 * %d = signed integer               %f = floating point number  
 * %s = string                     %.1f = float to 1 decimal place
 * %c = character                  %.3f = float to 3 decimal places
 * %e = scientific notation          %g = shortest representation of %e or %f                
 * %u = unsigned integer             %o = unsigned octal
 * %x = unsigned hex (lowercase)     %X = unsigned hex (uppercase)
 * %hd = short int                  %ld = long int
 * %lld = long long int
*/
    mqttClient.publish(F("Cwu/status/connected"), ds1307.getDateTime2().c_str(), true);
    mqttClient.publish(F("Cwu/status/reconnected"), String(qty_mqttReconnect).c_str(), true);//status karty SD
    mqttClient.publish(F("Cwu/status/rssi"), String(WiFi.RSSI()).c_str(), true);//wifi RSSI
    mqttClient.publish(F("Cwu/status/SD"), String(SDCARD_ERR).c_str(), true);//status karty SD
    mqttClient.publish(F("Cwu/status/SD_info"), String(SD_ERR[SDCARD_ERR]).c_str(), true);//info karta SD

    mqttClient.publish(F("Cwu/boiler_room/temp/AM2320"), String(arySensor[0]).c_str(), true);//AM2320 temp
    mqttClient.publish(F("Cwu/boiler_room/hum/AM2320"), String(arySensor[1]).c_str(), true);//AM2320 hum
    mqttClient.publish(F("Cwu/boiler_room/temp/BME280"), String(arySensor[2]).c_str(), true);//BME280 temp
    mqttClient.publish(F("Cwu/boiler_room/hum/BME280"), String(arySensor[3]).c_str(), true);//BME280 hum
    mqttClient.publish(F("Cwu/boiler_room/press/BME280"), String(arySensor[4]).c_str(), true);//BME280 pressure
    mqttClient.publish(F("Cwu/boiler_room/temp/tank"), String(ds18float[0]).c_str(), true);//kotłownia cwu temp.wody w zasobniku
    mqttClient.publish(F("Cwu/boiler_room/temp/tank_inlet"), String(ds18float[1]).c_str(), true);//kotłownia cwu temp.zasil.zasobnika
    mqttClient.publish(F("Cwu/boiler_room/temp/tank_outlet"), String(ds18float[2]).c_str(), true);//kotłownia cwu temp.powrotu zasobnika
    mqttClient.publish(F("Cwu/boiler_room/temp/pump_circulation"), String(ds18float[3]).c_str(), true);//kotłownia cwu temp.na pompie obiegowej
    mqttClient.publish(F("Cwu/boiler_room/temp/stove_inlet"), String(ds18float[4]).c_str(), true);//kotłownia cwu temp.zasilania pieca
    mqttClient.publish(F("Cwu/boiler_room/temp/stove_outlet"), String(ds18float[5]).c_str(), true);//kotłownia cwu temp.powrotu pieca
    mqttClient.publish(F("Cwu/boiler_room/temp/co_inlet"), String(ds18float[6]).c_str(), true);//kotłownia cwu temp.co zasilania
    mqttClient.publish(F("Cwu/boiler_room/temp/fireplace_exchanger_inlet"), String(ds18float[9]).c_str(), true);//kotłownia cwu zasilanie z kominka wymiennika ciepłą
    mqttClient.publish(F("Cwu/boiler_room/press/co"), String(arySensor[14]).c_str(), true);//kotłownia ciśnienie co
    mqttClient.publish(F("Cwu/boiler_room/state/pump_cwu"), String(CWU_pump).c_str(), true);//kotłownia pompa cwu
    mqttClient.publish(F("Cwu/floor_heating/temp/behind_heat_exchanger"), String(ds18float[7]).c_str(), true);//kotłownia cwu podłogówka temp.za wymiennikiem ciepła
    mqttClient.publish(F("Cwu/floor_heating/temp/lower_collector"), String(ds18float[8]).c_str(), true);//kotłownia cwu podłogówka temp.dolnego kolektora
    mqttClient.publish(F("Cwu/floor_heating/temp/upper_collector"), String(ds18float[10]).c_str(), true);//kotłownia cwu podłogówka temp.górnego kolektora
    mqttClient.publish(F("Cwu/floor_heating/state/pump"), String(FLOOR_pump).c_str(), true);//kotłownia cwu pompa podłogówki
    mqttClient.publish(F("Cwu/upper_bathroom/temp/AM2320"), String(arySensor[5]).c_str(), true);//ub AM2320 temp
    mqttClient.publish(F("Cwu/upper_bathroom/hum/AM2320"), String(arySensor[6]).c_str(), true);//ub AM2320 hum
    mqttClient.publish(F("Cwu/upper_bathroom/state/PIR"), String(PIR_holdStateOn).c_str(), true);//ub PIR
    PIR_holdStateOn=PIR;      
    mqttClient.publish(F("Cwu/upper_bathroom/state/PIR_disable"), String(DisablePIR).c_str(), true);//ub blokada PIR
    mqttClient.publish(F("Cwu/upper_bathroom/state/PIR_inWorkingHour"), String(inWorkingHour).c_str(), true);//ub PIR inWorkingHour
    mqttClient.publish(F("Cwu/fireplace/voltage/akku"), String(arySensor[8]).c_str(), true);//kominek napięcie akumulatora
    mqttClient.publish(F("Cwu/fireplace/level/water"), String(arySensor[9]).c_str(), true);//kominek poziom wody w zbiorniku wyrównawczym [mm]
    mqttClient.publish(F("Cwu/fireplace/temp/under_tank"), String(arySensor[10]).c_str(), true);//kominek temp.pow. nad zbiorniczkiem wyrównawczym
    mqttClient.publish(F("Cwu/fireplace/temp/tank"), String(arySensor[11]).c_str(), true);//kominek temp.wody w zbiorniczku wyrównawczym
    mqttClient.publish(F("Cwu/fireplace/number/nRF_transmission"), String(arySensor[12]).c_str(), true);//nRF nr transmisji
    mqttClient.publish(F("Cwu/fireplace/state/elv"), String(ELV).c_str(), true);//fp stan elz uzupełniania wody
    mqttClient.publish(F("Cwu/fireplace/state/elv_timeout"), String(ELV_timeout_error).c_str(), true);//fp stan elz timeout
  }
}
