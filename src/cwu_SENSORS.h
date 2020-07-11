//------------------------------------------------------------------------------------
// DEFINES FOR AM2320 sensor on AM2320 shield inside in case
//------------------------------------------------------------------------------------

//#include "Adafruit_AM2320.h"
//Adafruit_AM2320 AM_2320 = Adafruit_AM2320(); 20190912
#include <AM2320.h> //20190912 by hibikiledo: https://github.com/hibikiledo/AM2320
AM2320 AM_2320;

//------------------------------------------------------------------------------------
// DEFINES FOR BME/P 280
//------------------------------------------------------------------------------------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
Adafruit_BME280 bme; // I2C
//#include <Adafruit_BMP280.h>
//Adafruit_BMP280 bme; // I2C

// For more details on the following scenarious, see chapter 3.5 "Recommended modes of operation" in the datasheet
/*
    // weather monitoring
    Serial.println("-- Weather Station Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 1x pressure oversampling,");
    Serial.println("filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );
                      
    // suggested rate is 1/60Hz (1m)
    delayTime = 60000; // in milliseconds
*/

/*    
    // humidity sensing
    Serial.println("-- Humidity Sensing Scenario --");
    Serial.println("forced mode, 1x temperature / 1x humidity / 0x pressure oversampling");
    Serial.println("= pressure off, filter off");
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_NONE, // pressure
                    Adafruit_BME280::SAMPLING_X1,   // humidity
                    Adafruit_BME280::FILTER_OFF );
                      
    // suggested rate is 1Hz (1s)
    delayTime = 1000;  // in milliseconds
*/

/*    
    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    
    // suggested rate is 25Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5) + (2 * H_ovs + 0.5)
    // T_ovs = 2
    // P_ovs = 16
    // H_ovs = 1
    // = 40ms (25Hz)
    // with standby time that should really be 24.16913... Hz
    delayTime = 41;
    */
    
    /*
    // gaming
    Serial.println("-- Gaming Scenario --");
    Serial.println("normal mode, 4x pressure / 1x temperature / 0x humidity oversampling,");
    Serial.println("= humidity off, 0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X1,   // temperature
                    Adafruit_BME280::SAMPLING_X4,   // pressure
                    Adafruit_BME280::SAMPLING_NONE, // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
                      
    // Suggested rate is 83Hz
    // 1 + (2 * T_ovs) + (2 * P_ovs + 0.5)
    // T_ovs = 1
    // P_ovs = 4
    // = 11.5ms + 0.5ms standby
    delayTime = 12;
*/

//------------------------------------------------------------------------------------
// DEFINES FOR DALLAS (18B20)
//------------------------------------------------------------------------------------
#include <OneWire.h>
#include <DallasTemperature.h>
OneWire oneWire(DALLAS_PIN);
DallasTemperature ds18(&oneWire);
const int ds18Resolution = 12;
DeviceAddress ds18addr[] = 
{
  { 0x28, 0xFF, 0x22, 0xF5, 0x01, 0x17, 0x05, 0xD5 }, //BROWN-VIOLET   zasobnik cwu
  { 0x28, 0x28, 0x19, 0x94, 0x97, 0x0C, 0x03, 0x94 }, //RED            zasilanie zasobnika cwu
  { 0x28, 0xBC, 0x0D, 0x94, 0x97, 0x03, 0x03, 0x82 }, //YELLOW         powrót zasobnika cwu
  { 0x28, 0xDB, 0x02, 0x94, 0x97, 0x0C, 0x03, 0xB3 }, //VIOLET         pompa obiegowa cwu
  { 0x28, 0x36, 0x4C, 0x94, 0x97, 0x0C, 0x03, 0x9B }, //GREEN          piec zasilanie
  { 0x28, 0x90, 0x27, 0x94, 0x97, 0x0E, 0x03, 0x85 }, //BROWN          piec powrót
  { 0x28, 0xFF, 0xBA, 0x69, 0x02, 0x17, 0x03, 0x96 }, //BLUE-VIOLET    zasilanie CO
  { 0x28, 0xE1, 0xF2, 0x79, 0x97, 0x05, 0x03, 0x51 }, //vi-vi          za wymiennikiem ciepła - odbiór ciepła z kominka
  { 0x28, 0xEB, 0xBD, 0x79, 0x97, 0x05, 0x03, 0x18 }, //vi-gn          wlot do wymiennika ciepła - z kominka
  { 0x28, 0x0D, 0x1E, 0x79, 0x97, 0x08, 0x03, 0x77 }, //vi-rd          dolny kolektor podłogówki
  { 0x28, 0x9B, 0xA8, 0x79, 0x97, 0x07, 0x03, 0xBF }  //vi-ye          górny kolektor podłogówki
};
const int ds18count = sizeof(ds18addr)/sizeof(DeviceAddress);
//const uint8_t amount_timers = COUNT_OF(timeTbl);  //ilość timers_al

uint8_t ds18index[ds18count]; //index of this matches ds18addr and the value is the index the library sees
unsigned int ds18delay;
unsigned long ds18lastreq;
float ds18float[ds18count];

float arySensor[] =  //macierz wartości odczytanych z czujników
{
  0.0,//arySensor[0]: AM2320 temp w obudowie
  0.0,//arySensor[1]: AM2320 wilgotność w obudowie
  0.0,//arySensor[2]: BME temp
  0.0,//arySensor[3]: BME wilgotność
  0.0,//arySensor[4]: BME ciśn
  0.0,//arySensor[5]: górna łaz temp
  0.0,//arySensor[6]: górna łaz wilgotność
  0.0,//arySensor[7]: uint16_t  DATA_FROM_NFR.Sensor.Vcc
  0.0,//arySensor[8]: uint16_t  DATA_FROM_NFR.Sensor.battery
  0.0,//arySensor[9]: uint16_t  DATA_FROM_NFR.Sensor.waterLevel
  0.0,//arySensor[10]: float     DATA_FROM_NFR.Sensor.d18_1
  0.0,//arySensor[11]: float     DATA_FROM_NFR.Sensor.d18_2
  0.0,//arySensor[12]: uint8_t   DATA_FROM_NFR.Sensor.seq
  0.0,//arySensor[13]: uint8_t   DATA_FROM_NFR.Sensor.retry
  0.0 //arySensor[14]: uint8_t   ciśnienie wody w CO
};
const int arySensor_count = sizeof(arySensor)/sizeof(float);//ilość elementów w tablicy arySensor
byte arySensor_precision[arySensor_count] = //ile miejsc po przecinku przy zapisie na kartę SD
{
  1,//arySensor[0]: AM2320 temp w obudowie
  1,//arySensor[1]: AM2320 wilgotność w obudowie
  1,//arySensor[2]: BME temp
  1,//arySensor[3]: BME wilgotność
  1,//arySensor[4]: BME ciśn
  1,//arySensor[5]: górna łaz temp
  1,//arySensor[6]: górna łaz wilgotność
  3,//arySensor[7]: uint16_t  DATA_FROM_NFR.Sensor.Vcc
  2,//arySensor[8]: uint16_t  DATA_FROM_NFR.Sensor.battery
  0,//arySensor[9]: uint16_t  DATA_FROM_NFR.Sensor.waterLevel
  1,//arySensor[10]: float     DATA_FROM_NFR.Sensor.d18_1
  1,//arySensor[11]: float     DATA_FROM_NFR.Sensor.d18_2
  0,//arySensor[12]: uint8_t   DATA_FROM_NFR.Sensor.seq
  0,//arySensor[13]: uint8_t   DATA_FROM_NFR.Sensor.retry
  2 //arySensor[14]: uint8_t   ciśnienie wody w CO
};


                      //------------------------------------------------------------------------------------
                      // SECTION AM2320
                      //------------------------------------------------------------------------------------
void AM2320_READ()//wywoływany przez Timer[1]
{
  static byte qty_read = 0;
//  arySensor[0] = AM_2320.readTemperature();//20190912
//  arySensor[1] = AM_2320.readHumidity();

  if(qty_read > 4)//odczyt co 2*5 = 10sek
  {
    qty_read = 0;
    if (AM_2320.measure())  //Tell sensor to perform both temperature and humidity acquisitions
    {
      arySensor[0] = AM_2320.getTemperature();
      arySensor[1] = AM_2320.getHumidity(); 
    }
    else
    {  // error has occured
      int errorCode = AM_2320.getErrorCode();
      switch (errorCode)
      {
        SSF("AM2320 ERR: ");
        case 1: SLNF("Sensor is offline"); break;
        case 2: SLNF("CRC validation failed."); break;
        default: SLNF("Unknown error");
      }
    }
  }
  else qty_read++;
}

                      //------------------------------------------------------------------------------------
                      // SECTION BME280
                      //------------------------------------------------------------------------------------
void BME_READ()
{
    arySensor[2]=bme.readTemperature();
    #ifdef __BME280_H__
    arySensor[3]=bme.readHumidity();
    #else
    arySensor[3]=0.0;
    #endif
    arySensor[4]=bme.readPressure() / 100.0F;
}

                      //------------------------------------------------------------------------------------
                      // SECTION DALLAS (18B20)
                      //------------------------------------------------------------------------------------
void DALLAS_INI()
{
  ds18.begin(); //dallas 18B20
  for(byte j = 0; j < ds18count; j++)
  {
    ds18index[j] = 255; //prime for failure
  }
  //loop the devices detected and match them with sensors we care about
  for (byte i = 0; i < ds18.getDeviceCount(); i++)
  {
    DeviceAddress taddr;
    if(ds18.getAddress(taddr, i))
    {
      ds18.setResolution(taddr, ds18Resolution); //also set desired resolution
      for(byte j = 0; j < ds18count; j++)
      {
        if(oneWire.crc8(taddr, 7) == ds18addr[j][7]) //found it
        {
          SS(i); SSF(" matches"); SLN(j);
          ds18index[j] = i; //store the lib index in the array
          break; //stop the j loop
        }
      }
    }
  }
  ds18.setWaitForConversion(false); //this enables asyncronous calls
  ds18.requestTemperatures(); //fire off the first request
  ds18lastreq = millis();
  ds18delay = 750 / (1 << (12 - ds18Resolution)); //delay based on resolution
}//DALLAS_INI()

void DALLAS_READ()
{
  for(byte i = 0; i < ds18count; i++)
  {
    ds18float[i] = ds18.getTempC(ds18addr[i]);
  }
  ds18.requestTemperatures(); 
}

