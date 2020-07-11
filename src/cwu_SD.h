
//------------------------------------------------------------------------------------
// DEFINES FOR SD CARD
//------------------------------------------------------------------------------------
#include <SPI.h>
#include "SdFat.h"
#define SDCARD_OFF   digitalWrite(SS_PIN, HIGH)
#define SDCARD_ON    digitalWrite(SS_PIN, LOW)
SdFat sd;
SdFile file;
char SD_fileName[] = "cwu2.txt";
byte SDCARD_ERR = 0;
uint8_t cnt_SD_write = 0;

/*
  0: OK
  1: błąd otwarcia pliku
  2: set write timestamp failed
  3: set access timestamp failed
  4: Not saving log to SD Card (błąd dostępu do SD Card)
 */
char *SD_ERR[] = 
{
  F("SD write OK "),
  F("SD ERROR: opening file "),
  F("SD ERROR: set write timestamp failed "),
  F("SD ERROR: set access timestamp failed "),
  F("SD ERROR: Not saving log to SD Card ")
};

                      //------------------------------------------------------------------------------------
                      // SECTION SD
                      //------------------------------------------------------------------------------------
//przygotowanie ciągu znakowego do zapisu na karte SD
String SD_DATA_STRING()
{
  if(CWU_pump)
  {
    CWU_duration = millis() - CWU_startTime;//bieżący czas pracy pomy cwu
  }

  if(FLOOR_pump)
  {
    FLOOR_duration = millis() - FLOOR_startTime;//bieżący czas pracy pomy podłogówki
  }

  String dt1 = ds1307.getDateTime2();     //dd.mm.yyyy HH:MM:SS
  dt1 += ","; dt1 += SubStr[0];           //ip serwera
  dt1 += ","; dt1 += SubStr[1];           //poziom sygnału WiFi serwera
  dt1 += ","; dt1 += CWU_count;           //ilość załączeń pompy cwu
  dt1 += ","; dt1 += CWU_duration / 1000; //czas ostatniego załączenia pompy cwu [s]
  dt1 += ","; dt1 += CWU_total / 1000;    //całkowity czas załączeń pompy cwu [s]

  //czujniki (float)
  for(byte j = 0; j < arySensor_count; j++)
  {
    dt1 += ","; dt1 += String(arySensor[j], arySensor_precision[j]); //float na string z określoną ilością cyfr po przecinku
  }

  //czujniki temperatury DALLAS 18B20 (float)
  for(byte j = 0; j < ds18count; j++)
  {
    dt1 += ","; dt1 += String(ds18float[j], 1); //float na string z jedną cyfrą po przecinku
  }

  SLNF("Data string to SD CARD:"); SLN(dt1);
  int len = dt1.length();
  //byte len=answer.length();
  SSF("String length= "); SLN(len);
  return dt1;
}//String SD_DATA_STRING()

byte SAVE_LOG(char* fileName) //ał zapis danych do pliku log na karcie SD
{
  byte err = 0;
  cnt_SD_write++;
  SS(ds1307.getDateTime2()); SLNF(": <SAVE_LOG>");
  
  SDCARD_ON;
  if (!file.open(fileName, O_CREAT | O_WRITE | O_AT_END)) //error opening SD_fileName
  {
    err = 1;//błąd otwarcia pliku
    return err;
  }//byte SAVE_LOG(char* fileName)
  
  //opened OK, write data to SD logfile
  file << SD_DATA_STRING() << endl;
  
  time_t local = now();
  if (!file.timestamp(T_WRITE, year(local), month(local), day(local), hour(local), minute(local), second(local)))
  {
    err = 2;
    return err;
  }
  // set access date time to now
  if (!file.timestamp(T_ACCESS, year(local), month(local), day(local), hour(local), minute(local), second(local)))
  {
    err = 3;
    return err;
  }
  
  file.close();

  SDCARD_OFF;
  return err;
}//byte SAVE_LOG()

void SD_CARD()//wywoływany przez Timer[0]
{
  SDCARD_ON;
  if (sd.begin(SS_PIN))  //zapis pomiarów na kartę SD
  {
    SDCARD_ERR = SAVE_LOG(SD_fileName);//ewentualny błąd 1..3
  }
  else//błąd dostępu do SDCARD
  {
    SDCARD_ERR = 4;
    SSF(SD_ERR[SDCARD_ERR]);SSF("#"); SLN(SS_PIN);
  }
  SSF(SD_ERR[SDCARD_ERR]); SLN(SD_fileName);
  SDCARD_OFF;
}//SD_CARD()
