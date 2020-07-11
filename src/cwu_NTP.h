//------------------------------------------------------------------------------------
// DEFINES FOR WiFi UDP
//------------------------------------------------------------------------------------
#include <WiFiUdp.h>
WiFiUDP udp;
unsigned int localPort = 2390;      // local port to listen for UDP packets

//------------------------------------------------------------------------------------
// DEFINES FOR NTP
//------------------------------------------------------------------------------------
/* 
 * IPAddress timeServer(129, 6, 15, 28); // time.nist.gov NTP server
 * const char* ntpServerName = "time.nist.gov";
 */ 
IPAddress timeServerIP;             // time.nist.gov NTP server address
const char* ntpServerName = "pl.pool.ntp.org";
const int NTP_PACKET_SIZE = 48;     // NTP time stamp is in the first 48 bytes of the message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming and outgoing packets



                      //------------------------------------------------------------------------------------
                      // SECTION NTP
                      //------------------------------------------------------------------------------------
// send an NTP request to the time server at the given address
void NTP_SEND_PACKET(IPAddress &address) 
{
  SSF("**** sending NTP packet... ");
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  udp.beginPacket(address, 123); //NTP requests are to port 123
  udp.write(packetBuffer, NTP_PACKET_SIZE);
  udp.endPacket();
}//NTP_SEND_PACKET

void NTP_READ()
{
  //get a random server from the pool
  WiFi.hostByName(ntpServerName, timeServerIP);

  NTP_SEND_PACKET(timeServerIP); // send an NTP packet to a time server
  // wait to see if a reply is available
  delay(1000);

  int cb = udp.parsePacket();
  if (!cb) 
  {
    SLNF("no packet yet");
  }
  else 
  {
    SSF("packet received, length= ");
    SLN(cb);
    // We've received a packet, read the data from it
    udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    //the timestamp starts at byte 40 of the received packet and is four bytes, or two words, long. First, esxtract the two words:
    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    SSF("Seconds since Jan 1 1900 = ");SLN(secsSince1900);
    // now convert NTP time into everyday time: Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    SSF("Unix time = "); SLN(epoch);

    /*
    // print the hour, minute and second:
    Serial.print("The UTC time is ");       // UTC is the time at Greenwich Meridian (GMT)
    Serial.print((epoch  % 86400L) / 3600); // print the hour (86400 equals secs per day)
    Serial.print(':');
    if (((epoch % 3600) / 60) < 10) {
      // In the first 10 minutes of each hour, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.print((epoch  % 3600) / 60); // print the minute (3600 equals secs per minute)
    Serial.print(':');
    if ((epoch % 60) < 10) {
      // In the first 10 seconds of each minute, we'll want a leading '0'
      Serial.print('0');
    }
    Serial.println(epoch % 60); // print the second
    */
    int ret=ds1307.setSecs1970(epoch);
    SSF("Status set DS1307 data/time: "); SLN(ret);
    SSF("from DS1307 UTC data/time: ");
    SLN(ds1307.getDateTime2());

    bool DST=ds1307.isSummertime(TIMEZONE);
    SSF("isSummerTime? "); SLN(DST);
    ret=ds1307.setSecs1970(epoch + 3600 * (TIMEZONE + DST));
    SSF("finally from DS1307: ");
    SS(ds1307.getDateTime2()); SLNF("\n");
  }
}//NTP_READ()
