/*  NETPIE ESP8266 basic sample                            */
/*  More information visit : https://netpie.io             */

#include <TimeLib.h>
#include <ESP8266WiFi.h>
#include <MicroGear.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
#define PIN D3
#include <DHT.h>c
#include <MCP3008.h>
#define DHTTYPE DHT11
#define DHTPIN D4

#define CS_PIN D8
#define CLOCK_PIN D5
#define MOSI_PIN D7
#define MISO_PIN D6

// put pins inside MCP3008 constructor
MCP3008 adc(CLOCK_PIN, MOSI_PIN, MISO_PIN, CS_PIN);
DHT dht(DHTPIN,DHTTYPE);

//#define NUM_LEDS D3

#define pixelCount D1
#define colorSaturation 128
#define Pump D0
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);
time_t getNtpTime();

const char* ssid     = "RICH_2G4r";
const char* password = "27/14GtechRICH";

#define APPID   "PlantPots"
#define KEY     "wd4YlkkHGs4FNed"
#define SECRET  "PXxOq4MHnCG8VCVBRwwVVCQNx"
#define ALIAS   "/esp8266"

WiFiClient client;
int timer = 0;
MicroGear microgear(client);

// NTP Servers:
static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 7;     // Central European Time

WiFiUDP Udp;
unsigned int localPort = 8888;
time_t getNtpTime();
void digitalClockDisplay();
void printDigits(int digits);
void sendNTPpacket(IPAddress &address);
int k = 0 ; 

/* If a new message arrives, do this */
void onMsghandler(char *topic, uint8_t* msg, unsigned int msglen) {
    Serial.print("Incoming message --> ");
      char strState[msglen];
      for (int i = 0; i < msglen; i++) 
      {
        strState[i] = (char)msg[i];
        Serial.print((char)msg[i]);
      }
     
      Serial.println();
    
      String stateStr = String(strState).substring(0, msglen);
    
    //=========== ช่วงประมวลผลคำสั่ง =============
    
      if (stateStr == "PumpOn") 
      {
       digitalWrite(Pump, LOW);
       delay(5000);
       digitalWrite(Pump, HIGH);
      } 
}

/* When a microgear is connected, do this */
void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
    Serial.println("Connected to NETPIE...");
    /* Set the alias of this microgear ALIAS */
    microgear.setAlias(ALIAS);
}

void setup() {
    /* Add Event listeners */

    /* Call onMsghandler() when new message arraives */
     microgear.on(MESSAGE,onMsghandler);
  pinMode(Pump,OUTPUT);
  digitalWrite(Pump, HIGH);
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  dht.begin();
  //FastLED.addLeds<WS2812, DATA_PIN, GRB>(leds, NUM_LEDS);
  strip.begin();
  strip.show();
  microgear.on(CONNECTED,onConnected);
    Serial.begin(9600);
    Serial.println("Starting...");

    /* Initial WIFI, this is just a basic method to configure WIFI on ESP8266.                       */
    /* You may want to use other method that is more complicated, but provide better user experience */
    if (WiFi.begin(ssid, password)) {
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
    }
    Serial.println("WiFi connected");  
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    /* Initial with KEY, SECRET and also set the ALIAS here */
    microgear.init(KEY,SECRET,ALIAS);
    /* connect to NETPIE to a specific APPID */
    microgear.connect(APPID);
   // Serial.print("eiei");

  Serial.print("IP number assigned by DHCP is ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Udp.begin(localPort);
  Serial.print("Local port: ");
  Serial.println(Udp.localPort());
  Serial.println("waiting for sync");
  setSyncProvider(getNtpTime);
  setSyncInterval(300);
  
    int k = 0 ; 
    Serial.println(k);
}
time_t prevDisplay = 0; 

void loop() 
{
  int a = hour();
  int val = map(adc.readADC(0),0,1023,100,0);
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  Serial.println(a);
   String d1 = String(t);
   String d2 = String(h);
   String d3 = String(val);
   String d4 = d1+","+d2+","+d3+","+ "test1";
   if(WiFi.status() == WL_CONNECTED)
    {
      microgear.publish("/esp8266",d4);
    }
   if (a == 14 && k == 0)
    {
      digitalWrite(Pump, LOW);
      delay(5000);
      digitalWrite(Pump, HIGH);
      k = 1;
    }  
    if(a == 1)
    {
      k = 0;
    }
  if(val < 20)//สีแดง
  {
    colorWipe(strip.Color(255,0, 0), 50);
  }
  else//สีเขียว
  {
    colorWipe(strip.Color(0, 255, 0), 50);
  }
  /* To check if the microgear is still connected */
    microgear.loop();
    delay(100);
}
void colorWipe(uint32_t c, uint8_t wait) 
{
  for(uint16_t i=0; i<strip.numPixels(); i++) 
  {
    strip.setPixelColor(i, c);
    strip.show();
    delay(100);
  }
}

void digitalClockDisplay()
{
  // digital clock display of the time
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.print(" ");
  Serial.print(day());
  Serial.print(".");
  Serial.print(month());
  Serial.print(".");
  Serial.print(year());
  Serial.println();
}

void printDigits(int digits)
{
  // utility for digital clock display: prints preceding colon and leading 0
  Serial.print(":");
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}

/*-------- NTP code ----------*/

const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets

time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (Udp.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = Udp.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      Udp.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
