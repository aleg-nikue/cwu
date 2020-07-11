//------------------------------------------------------------------------------------
// DEFINES FOR OLED
//------------------------------------------------------------------------------------
#include <Adafruit_GFX.h>           // OLED
#include <Adafruit_SSD1306.h>       // OLED
#include <Fonts/Picopixel.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#define OLED_RESET -1//brak resetu
Adafruit_SSD1306 oled(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 48)
  #error("OLED height incorrect, please check Adafruit_SSD1306.h!");
#endif

#define amountOledScr 8 //ilość ekranów OLED
/*
  screen #0: info WiFi
  screen #1: temp & humidity from BME/P 280
  screen #2: temp & humidity in case from AM2320
  screen #3: temp from Dallas 18B20 set #1
  screen #4: temp from Dallas 18B20 set #2
  screen #5: from server 10.14.0.15
  screen #6: from nFR tank fireplace
  screen #7: pressure CO
 */
byte currScrNr = 0;     //aktualny nr ekranu OLED - od 0


                      //------------------------------------------------------------------------------------
                      // SECTION OLED
                      //------------------------------------------------------------------------------------
void OLED_INI()
{
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);   // I2C address of my blue OLED
  oled.setTextSize(1);
  oled.setFont(&Picopixel);
  oled.setTextColor(WHITE);
  oled.clearDisplay();
  oled.setCursor(0, 6); oled.print(F("Wemos Client"));
  oled.display();
}//OLED_INI

void OLED_NEXT_SCR()  //invoke by alTimer#4
{
  currScrNr++;
  if(currScrNr > amountOledScr)
  {
    currScrNr = 0;
  }
}//OLED_NEXT_SCR

void OLED_PRINT()
{
  oled.setFont(&Picopixel); oled.setTextSize(1);
  oled.clearDisplay();
  
//jeśli błąd karty SD - to inwersja ekranu OLED
  if (SDCARD_ERR) oled.invertDisplay(true);
  else oled.invertDisplay(false);
  
  oled.setCursor(60, 6); oled.print(currScrNr);//nr ekranu
  oled.setCursor(0, 6);

  switch (currScrNr)
  {
    case 1:
    //BME280
      oled.println(F("from BME280"));
      oled.setFont(); oled.setTextSize(1);
      oled.setCursor(0, 10); oled.print(arySensor[3], 1); oled.print(F("% "));
      oled.print(arySensor[4], 0);  //ciśnienie
      oled.setTextSize(2);
      oled.setCursor(0, 22); oled.print(arySensor[2], 1); oled.println(F("C"));
      break;

    case 2:
    //AM2320
      oled.println(F("from AM2320"));
      oled.setFont(); oled.setTextSize(1);
      oled.setCursor(0, 10); oled.print(arySensor[1], 1); oled.print(F("%"));
      oled.setTextSize(2);
      oled.setCursor(0, 22); oled.print(arySensor[0], 1); oled.println(F("C"));
      break;

    case 3:
    case 4:
    case 5:
    //DALLAS 18B20 sensors
      oled.println(F("from 18B20"));
      oled.setFont(); oled.setTextSize(1);
      for(byte i = 0; i < 4; i++)
      {
        byte item = (currScrNr - 3) * 4 + i;
        /*
          dla ekranu #3: item = 0 .. 3
          dla ekranu #4: item = 4 .. 7
          dla ekranu #5: item = 8 .. 11
        */
        if(item < ds18count)
        {
          oled.setCursor(0, 9 + 8 * i);
          if(item<10)oled.print(F(" "));
          oled.print(item); oled.print(F(": "));
          oled.print(ds18float[item], 1); oled.println(F("C"));
        }
      }
      break;

    case 6:
    //server 10.14.0.15 górna łazienka
      oled.print(F("from "));oled. println(server[0]);
      oled.setFont(); oled.setTextSize(1);
      oled.setCursor(0, 10); oled.print(arySensor[6], 0); oled.print(F("% "));
      oled.print(F("cwu=")); oled.print(CWU_pump);
      oled.setFont(); oled.setTextSize(2);
      oled.setCursor(0, 22); oled.print(arySensor[5], 1); oled.println(F("C"));
      break;

    case 7:
    //nFR
      oled.print(F("from nFR,seq")); oled.println(arySensor[12], 0);
      oled.setFont(); oled.setTextSize(1);
      oled.setCursor(0, 10); oled.print(arySensor[8], 2); oled.print(F("V "));
      oled.print(arySensor[10], 1); oled.setCursor(55, 18); oled.print(F("C"));
      oled.setFont(); oled.setTextSize(2);
      oled.setCursor(0, 22); oled.print(arySensor[9], 0); oled.println(F("mm"));
      break;

    case 8:
    //pressure sensor
      oled.print(F("press.sensor"));
      oled.setFont(); oled.setTextSize(1);
      oled.setCursor(0, 10); oled.print(F("AI0= ")); oled.print(analogRead(A0));
      oled.setFont(); oled.setTextSize(2);
      oled.setCursor(0, 22); oled.print(arySensor[14], 1); oled.println(F("ba"));
      break;

    default:
    //info
      oled.println(ssid);//nazwa sieci WiFi
      oled.print(F("Host ")); oled.println(WiFi.hostname());
      oled.println(WiFi.macAddress());
      oled.print(F("IP ")); oled.println(WiFi.localIP());
      oled.print(F("Signal ")); oled.println(WiFi.RSSI());
      break;
  }
  //stopka: bieżąca data/czas
  oled.setFont(&Picopixel); oled.setTextSize(1);
  oled.setCursor(0, 47); oled.println(ds1307.getDateTime2());
  oled.display();
}//OLED_PRINT()
