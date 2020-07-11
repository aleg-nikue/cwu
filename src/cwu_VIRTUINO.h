                      //------------------------------------------------------------------------------------
                      // SECTION VIRTUINO
                      //------------------------------------------------------------------------------------
WiFiServer virtServ(8000);                     // for virtuino
#include "VirtuinoCM.h"
VirtuinoCM virtuino; 

#define V_memory_count 50 // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];  // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

boolean debug;            // set this variable to false on the finale code to decrease the request time.

boolean vRly, vRly_old, vRly_clr, vElv, vFloor;                          

//============================================================== onReceived
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
 void onReceived(char variableType, uint8_t variableIndex, String valueAsText)
 {     
    if ((variableType == 'V') && (variableIndex < V_memory_count))
    {
      float value = valueAsText.toFloat();  // convert the value to float. The valueAsText have to be numerical
      V[variableIndex] = value;             // copy the received value to arduino V memory array
    }
}

//============================================================== onRequested
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex)
{
  if ((variableType == 'V') && (variableIndex < V_memory_count))
  {
    return String(V[variableIndex]);  // return the value of the arduino V memory array
  }
  return "";
}

//============================================================== virtuinoRun
void virtuinoRun()
  {
    WiFiClient client = virtServ.available();
    if (!client) return;
    if (debug) SLNF("Virtuino Connected");
    unsigned long timeout = millis() + 3000;
    while (!client.available() && millis() < timeout) delay(1);
    if (millis() > timeout)
    {
      SLNF("timeout");
      client.flush();
      client.stop();
      return;
    }
    virtuino.readBuffer = "";     // clear Virtuino input buffer. The inputBuffer stores the incoming characters
      while (client.available() > 0)
      {
        char c = client.read();   // read the incoming data
        virtuino.readBuffer += c; // add the incoming character to Virtuino input buffer
        if (debug && (Serial.availableForWrite() >= 1)) Serial.write(c);
      }
     client.flush();
     if (debug)
     {
       SSF("\nReceived data: ");
       SLN(virtuino.readBuffer);
     }
     String* response = virtuino.getResponse();    // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
     if (debug)
     {
       SSF("Response : ");
       SLN(*response);
     }
     client.print(*response);
     client.flush();
     delay(10);
     client.stop(); 
    if (debug) SLNF("Disconnected\n");
}//virtuinoRun()


//============================================================== vDelay
void vDelay(int delayInMillis)
{
  long t = millis() + delayInMillis;
  while (millis() < t) virtuinoRun();
}//vDelay(int delayInMillis)

//==============================================================
//============================================================== end Ilias Lamprou


//============================================================== VIRTUINO_INI
void VIRTUINO_INI()
{
/* Start Virtuino. Set the buffer to 256. 
 * With this buffer Virtuino can control about 28 pins (1 command = 9bytes).
 * The T(text) commands with 20 characters need 20+6 bytes. */
  virtuino.begin(onReceived,onRequested,256);

  debug = false;             // set this variable to false on the finale code to decrease the request time.

  virtuino.key = F("1234");  // Set a password to your web server for more protection 
  virtServ.begin();
}//VIRTUINO_INI

//============================================================== VIRTUINO_VAR
void VIRTUINO_VAR()
/*
  V[32]: cnt_SD_write (uint8_t)
  V[33]: CWU stan pompy
  V[34]: CWU wymuszenie zmiany stanu pompy z Virtuino
  V[35]: CWU blokada PIR po zaistniałym załączeniu pompy
  V[36]: nFR Status komunikacji
  V[37]: PIR stan
  V[38]: ELV wymuszenie zmiany stanu elektrozaworu z Virtuino
  V[39]: ELV stan
  V[40]: ELV timeout
  V[41]: ELV reset timeout error
  V[42]: FLOOR stan pompy
  V[43]: FLOOR wymuszenie zmiany stanu pompy z Virtuino
  V[44]: SDCARD status
  V[45]: inWorkingHour godziny, w których PIR może załaczyć pompę cwu

  V[0]..V[14]: czujniki z arySensor
    V[0]: arySensor[0]: AM2320 temp w obudowie
    V[1]: arySensor[1]: AM2320 wilgotność w obudowie
    V[2]: arySensor[2]: BME temp
    V[3]: arySensor[3]: BME wilgotność
    V[4]: arySensor[4]: BME ciśn
    V[5]: arySensor[5]: górna łaz temp
    V[6]: arySensor[6]: górna łaz wilgotność
    V[7]: arySensor[7]: uint16_t  DATA_FROM_NFR.Sensor.Vcc
    V[8]: arySensor[8]: uint16_t  DATA_FROM_NFR.Sensor.battery
    V[9]: arySensor[9]: uint16_t  DATA_FROM_NFR.Sensor.waterLevel
    V[10]: arySensor[10]: float     DATA_FROM_NFR.Sensor.d18_1
    V[11]: arySensor[11]: float     DATA_FROM_NFR.Sensor.d18_2
    V[12]: arySensor[12]: uint8_t   DATA_FROM_NFR.Sensor.seq
    V[13]: arySensor[13]: uint8_t   DATA_FROM_NFR.Sensor.retry
    V[14]: arySensor[14]: uint8_t   ciśnienie wody w CO    

  V[17]: czas otwarcia elz
  V[18]: czas do wyłączenia cwu
  V[19]: czas do wyłączenia blokady cwu

  V[31]..V[25]: temperatura 18B20
    V[31]: BROWN-VIOLET  zasobnik cwu
    V[30]: RED           zasilanie zasobnika cwu
    V[29]: YELLOW        powrót zasobnika cwu
    V[28]: VIOLET        pompa obiegowa cwu
    V[27]: GREEN         piec zasilanie
    V[26]: BROWN         piec powrót
    V[25]: BLUE-VIOLET   zasilanie CO
    V[24]: vi-vi          za wymiennikiem ciepła - odbiór ciepła z kominka
    V[23]: rd-vi          dolny kolektor podłogówki
    V[22]: gn-vi          wlot do wymiennika ciepła - z kominka
    V[21]: ye-vi          górny kolektor podłogówki
 */
{
  virtuinoRun(); //komunikacja Virtuino

  V[32] = cnt_SD_write; //(uint8_t)
  V[33] = CWU_pump;     //DV1: stan pompy cwu
  if(CWU_pump) V[33] = 1;
  else
  {
    if(DisablePIR || !inWorkingHour) V[33] = -2;
    else V[33] = 0;
  }
  
  if(vRly_clr) //kasowanie rozkazu załączenia pompy cwu w Virtuino
  {
    V[34] = 0;
    vRly_clr = false;
  }
  vRly = V[34];              //DV2: zmiana stanu pompy cwu z Virtuino
  V[35] = DisablePIR;        //DV3: blokada PIR
  V[36] = radioNRFStatus;    //DV4: radioNRFStatus: czy jest transmisja
  if(PIR) V[37] = 1;
  else
  {
    if(!inWorkingHour) V[37] = -2;
    else V[37] = 0;
  }

//elektrozawór napełniania wody do zbiornika wyrównawczego kominka
  vElv = V[38];              //DV6: zmiana stanu elektrozaworu z Virtuino
  if(ELV_timeout_error)
  {
    V[38] = 0;               //kasowanie rozkazu zał elz w Virtuino
    vElv = 0;
  }
  V[39] = ELV;               //DV7: stan elektrozaworu
  V[40] = ELV_timeout_error; //DV8: czy był timeout elzaworu
  if(V[41])                  //DV9: reset elv timeout error
  {
    ELV_timeout_error = false;
  }

//pompa podłogówki
  V[42] = FLOOR_pump;       //DV10: stan pompy podłogówki
  vFloor = V[43];           //DV11: zmiana stanu pompy podłogówki z Virtuino

  V[44] = SDCARD_ERR;       //DV12: status SDCARD
  V[45] = inWorkingHour;    //DV13: godziny, w których PIR może załączyć pompę cwu

//test pcf out 20190910
  /*
  pcf8574.digitalWrite(0, virtuino.vDigitalMemoryRead(31));
  pcf8574.digitalWrite(1, virtuino.vDigitalMemoryRead(30));
  pcf8574.digitalWrite(2, virtuino.vDigitalMemoryRead(29));
  pcf8574.digitalWrite(3, virtuino.vDigitalMemoryRead(28));
  pcf8574.digitalWrite(4, virtuino.vDigitalMemoryRead(27));
  pcf8574.digitalWrite(7, virtuino.vDigitalMemoryRead(26));
  */

//V[0]..V[arySensor_count]
  for(byte i = 0; i < arySensor_count; i++)//odczyt czujników AM2320, BMP/E i z serwera górnej łazienki
  {
    V[i] = arySensor[i]; //V[0], V[1], ...
  }

//V[31]..V[31 - ds18count]
  for(byte i = 0; i < ds18count; i++)//odczyt czujników temperatury Dallas 18B20
  {
    V[31 - i] = ds18float[i]; //V[31], V[30], ...
  }
  
  V[17] = (TIMER[9].time_current);              //czas otwarcia elz
  V[18] = (timeTbl[5] - TIMER[5].time_current); //czas do wyłączenia cwu
  V[19] = (timeTbl[6] - TIMER[6].time_current); //czas do wyłączenia blokady cwu
}//VIRTUINO_VAR()
