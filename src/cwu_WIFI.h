                      //------------------------------------------------------------------------------------
                      // SECTION WiFi
                      //------------------------------------------------------------------------------------
  // We start by connecting to a WiFi network

void WIFI_CONNECTING()
{
  SSF("Connecting to "); SLN(ssid);//nazwa sieci WiFi
  WiFi.mode(WIFI_STA);  
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    SSF(".");
    oled.print(".");
    oled.display();  
  }
  oled.print("\nIP: "); oled.println(WiFi.localIP());//adres IP Wemos'a
  oled.setCursor(0, 47);
  oled.println(ds1307.getDateTime2());
  oled.display();
  delay(0);
}//WIFI_CONNECTING()

void RECONNECT() 
{
  if (WiFi.status() != WL_CONNECTED)//po rozłączeniu z WiFi restart WEMOS'a
  {
    SSF("\nReconnecting");
    WiFi.mode(WIFI_STA);  
    WiFi.begin(ssid, pass);  
    while (WiFi.status() != WL_CONNECTED) 
    {
      delay(500);  
      SSF(".");
    }  
    SLNF("Connected!\n");
  }
  delay(0);
}//RECONNECT()

