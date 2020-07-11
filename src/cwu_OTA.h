//------------------------------------------------------------------------------------
// DEFINES FOR OTA
//------------------------------------------------------------------------------------
#include <ArduinoOTA.h>

void OTA_SERVICES()
{
// Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  // ArduinoOTA.setHostname("myesp8266");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() 
  {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) 
    {
      type = "sketch";
    }
      else  // U_SPIFFS
    {
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    SSF("Start updating ");SLN(type);
  });
  ArduinoOTA.onEnd([] () {SLNF("\nEnd");} );
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  {
    #ifdef SERIAL_DEBUG
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    #endif
  });
  ArduinoOTA.onError([](ota_error_t error) 
  {
    #ifdef SERIAL_DEBUG
      Serial.printf("Error[%u]: ", error);
    #endif
    if (error == OTA_AUTH_ERROR) 
    {
      SLNF("Auth Failed");
    } 
    else if (error == OTA_BEGIN_ERROR) 
    {
      SLNF("Begin Failed");
    } 
    else if (error == OTA_CONNECT_ERROR) 
    {
      SLNF("Connect Failed");
    } 
    else if (error == OTA_RECEIVE_ERROR) 
    {
      SLNF("Receive Failed");
    } 
    else if (error == OTA_END_ERROR) 
    {
      SLNF("End Failed");
    }
  });

  ArduinoOTA.begin();
  SSF("\nReady OTA IP address: ");SLN(WiFi.localIP());
}//OTA_SERVICES

