//------------------------------------------------------------------------------------
// DEFINE FOR RADIO nRF20
//------------------------------------------------------------------------------------
/* 
  płytka TinyBrd nad zbiornikiem wyrównawczym kominka
 */
#include "RadioNRF24.h"
uint8_t address[3] = {0, 0, 3};
#define CHANNEL_NR 100

struct SensorData
{
  uint8_t id;
  uint16_t Vcc;
  uint16_t battery;
  uint16_t waterLevel;
  float d18_1;
  float d18_2;
  uint8_t seq;
  uint8_t retry;
} __attribute__((packed));  //wymuszenie poprawnej alokacji struktury

const int BUFF_SIZE = sizeof(SensorData);

union UNION_DTA
{
  SensorData Sensor;
  uint8_t rec[BUFF_SIZE];
} __attribute__((packed));  //wymuszenie poprawnej alokacji struktury

UNION_DTA DATA_FROM_NFR;

boolean radioNRFStatus; //status połączenia z nFR



                      //------------------------------------------------------------------------------------
                      // SECTION RADIO nRF20
                      //------------------------------------------------------------------------------------
boolean NFR_GET_DATA()
{
  uint8_t byteArray[BUFF_SIZE];
  uint8_t readStatus = RadioNRF24.available();//odczyt ilości odebranych bajtów
  if(readStatus > 0)
  {
    SSF("\nRadio available amount bytes: ");SLN(readStatus);
    RadioNRF24.read(byteArray);
    for (byte k = 0; k < BUFF_SIZE; k++)
    { 
      DATA_FROM_NFR.rec[k] = byteArray[k];
      SS(k); SSF(" -> "); SS(DATA_FROM_NFR.rec[k]); SSF(", ");
    }
    SLN();
    bool OK = (DATA_FROM_NFR.Sensor.id == 6);
    return OK;  //odebrano poprawnie z nFR
  }//end got packet
  else return false; // No Packet received
}// NFR_GET_DATA()

void RADIO_NRF()
{
  static uint8_t old_seq;
  bool radioStatus = NFR_GET_DATA();//odczyt z radia nRF24

  if ((DATA_FROM_NFR.Sensor.seq != old_seq) && radioStatus)//czy przyszedł nowy telegram z nFR? tzn. nowy nr seq i poprawne id
  {
    radioNRFStatus = true;
    old_seq = DATA_FROM_NFR.Sensor.seq;
  
    if(TIMER[8].active)
    {
      TIMER[8].last_state = false;//restart timera[8] - timeout nFR
    }
    else
    {
      TIMER[8].active = true;//aktywacja timera[8] - timeout nFR
    }
    
  }
  
  if (radioStatus)
  {
    //arySensor[7] = (float)DATA_FROM_NFR.Sensor.id;
    arySensor[7] = (float)DATA_FROM_NFR.Sensor.Vcc / 1000.0F;
    arySensor[8] = (float)DATA_FROM_NFR.Sensor.battery / 100.0F;
    arySensor[9] = (float)(tank_deep - DATA_FROM_NFR.Sensor.waterLevel);//poziom wody w zbiorniczku
    arySensor[10] = DATA_FROM_NFR.Sensor.d18_1;
    arySensor[11] = DATA_FROM_NFR.Sensor.d18_2;
    arySensor[12] = (float)DATA_FROM_NFR.Sensor.seq;
    arySensor[13] = (float)DATA_FROM_NFR.Sensor.retry;
  }
}//RADIO_NRF()

void NFR_RESET_STATUS()//timeout: brak transmisji z nRF
{
  radioNRFStatus = false;
  TIMER[8].active = false; //timeout transmisji z radia nFR
}//RESET_STATUS_NFR()
