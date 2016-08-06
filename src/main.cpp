#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <lwip/def.h>
#include <WiFiManager.h>         //https://github.com/tzapu/WiFiManager

#include "platform.h"
#include "logic.h"

#include "MFRC522.h"

#include <math.h>

#include <Ticker.h>

#include <EEPROM.h>


#define SHOULD_BREATH 0
#define DEBUG_PROG 0
/* wiring the MFRC522 to ESP8266 (ESP-12)
RST     = GPIO15
SDA(SS) = GPIO2
MOSI    = GPIO13
MISO    = GPIO12
SCK     = GPIO14
GND     = GND
3.3V    = 3.3V
*/

#define LED_PIN 2
#define RST_PIN	5 // RST-PIN für RC522 - RFID - SPI - Modul GPIO15
#define SS_PIN	16 // 16  // SDA-PIN für RC522 - RFID - SPI - Modul GPIO2


#define RED_LED 15
#define GRN_LED 4
#define BLU_LED 0

MFRC522 mfrc522(SS_PIN, RST_PIN);

class EspTcpClient : public TcpClient{
public:

  virtual bool connected() override {return _client.connected() != 0;}
  virtual bool connect(const char* host, int port) override {return _client.connect(host, port);}
  virtual int  available() override {return _client.available();}
  virtual void stop() override {return _client.stop();}
  virtual int  read() override {return _client.read();}
  virtual size_t  write(const uint8_t *buf, size_t size) override {return _client.write(buf, size);}

private:
  WiFiClient _client;
};

class DefaultPlatform : public Platform {
public:
  virtual TcpClient& getClient() override;
  virtual RFID detectRfidId() override;

  virtual void getStickId(uint8_t* byteArray) override;

  virtual void println(const char* s )override { Serial.println(s);}

  virtual uint32_t hostToNetwork(uint32_t i) {return htonl(i);}
  virtual void  setLed(uint8_t r,uint8_t g,uint8_t b);
private:
  EspTcpClient _client;
} platform;

TcpClient& DefaultPlatform::getClient() {
  return   _client;
}

// Helper routine to dump a byte array as hex values to Serial
void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
  }
}

#define EVENT_THRESH 2000
unsigned long lastEventMillis;

RFID DefaultPlatform::detectRfidId() {
  static bool ledMode;
  // https://github.com/Jorgen-VikingGod/ESP8266-MFRC522/blob/master/ESP8266-MFRC522.ino
  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {

      // toggle led
      if (ledMode != HIGH) {
        ledMode = HIGH;
      } else {
        ledMode = LOW;
      }
  //    digitalWrite(LED_PIN, ledMode);

      unsigned long curMilis = millis();
      unsigned long diff = curMilis - lastEventMillis;
      lastEventMillis = curMilis;
      if (diff < EVENT_THRESH) {
          RFID id = {{0}};
          return id;
      }

    Serial.print("Got serial id! ");
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();

    RFID id;
    id.id[0] = mfrc522.uid.uidByte[0];
    id.id[1] = mfrc522.uid.uidByte[1];
    id.id[2] = mfrc522.uid.uidByte[2];
    id.id[3] = mfrc522.uid.uidByte[3];
    return id;
    }
  }

  RFID id = {{0}};
  return id;
}

int ledR, ledG, ledB;

void DefaultPlatform::setLed(uint8_t r,uint8_t g,uint8_t b) {
  Serial.printf("Setting led to #%02X%02X%02X\n", (uint32_t) r,(uint32_t) g,(uint32_t) b);
  noInterrupts();
  ledR = r;
  ledG = g;
  ledB = b;
  interrupts();

}

#define BREATH_TIME 3000

#define SetLedToColor(r,g,b) do { \
  analogWrite(RED_LED, map(r,0,255,0,1023)); \
  analogWrite(GRN_LED, map(g,0,255,0,1023)); \
  analogWrite(BLU_LED, map(b,0,255,0,1023)); \
} while(0)


void breath() {
#if ! SHOULD_BREATH
  SetLedToColor(ledR, ledG, ledB);
#else
  float r,g,b;

  unsigned long curMilis = millis()%BREATH_TIME;

  if (curMilis > (BREATH_TIME/2)) {
    curMilis = BREATH_TIME - curMilis;
  }

  float curPhase = (1.0*curMilis / (BREATH_TIME/2));

 // start with the phase this goes from 0 to 1
 // now we want to map  0 to min part of the sine and 255 to the max

  r = g = b = -PI/2;

  // now R is in [-pi/2,pi + pi/2]
  r += PI*(curPhase * ledR / 255.0);
  g += PI*(curPhase * ledG / 255.0);
  b += PI*(curPhase * ledB / 255.0);

  // http://sean.voisen.org/blog/2011/10/breathing-led-with-arduino/
  uint8_t bR = (exp(sin(r)) - 0.36787944)*108.0;
  uint8_t bG = (exp(sin(g)) - 0.36787944)*108.0;
  uint8_t bB = (exp(sin(b)) - 0.36787944)*108.0;

  analogWrite(BLU_LED, map(bR,0,255,0,1023));
  analogWrite(GRN_LED, map(bG,0,255,0,1023));
  analogWrite(RED_LED, map(bB,0,255,0,1023));
#endif
}

Ticker breather;


void DefaultPlatform::getStickId(uint8_t* byteArray) {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  byteArray[0] = mac[0];
  byteArray[1] = mac[1];
  byteArray[2] = mac[2];
  byteArray[3] = mac[3];
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  SetLedToColor(255, 255, 0);
}


char serverHostAddress[32] = "";

bool shouldSaveConfig = false;

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

const int RESET_PIN = 10;

void setup(void) {
  bool shouldStartConfigPortalAnyway = false;
  Serial.begin(9600);
  EEPROM.begin(512);
  Serial.println(F("Booting...."));
  delay(1000);
  SPI.begin();	         // Init SPI bus
  mfrc522.PCD_Init();    // Init MFRC522
  pinMode(RESET_PIN, INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(BLU_LED, OUTPUT);

  SetLedToColor(255, 0, 0);
  delay(1000);
  SetLedToColor(0, 255, 0);
  delay(1000);
  SetLedToColor(0, 0, 255);
  delay(1000);

  SetLedToColor(0, 255, 0);

  Serial.println("Loading config.");
  const int hostSize = sizeof(serverHostAddress)/sizeof(serverHostAddress[0]);

  {
    int i = 0;
    for ( i = 0; i < hostSize-1; i++) {
        serverHostAddress[i] = EEPROM.read(i);
    }
    serverHostAddress[i] = '\0';
  }
  Serial.println(serverHostAddress);


  Serial.println("wifi - trying to auto connect.");
  WiFiManagerParameter serverHostAdParam("server", "URB server", serverHostAddress, hostSize);
  WiFiManager wifiManager;
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&serverHostAdParam);


  // should we config anyway
  shouldStartConfigPortalAnyway = digitalRead(RESET_PIN) == LOW;

  if (Serial.available() && (Serial.read() == 'y') ){
    shouldStartConfigPortalAnyway = true;
  }

  if (!shouldStartConfigPortalAnyway) {

      SetLedToColor(255, 0, 0);

      wifiManager.setAPCallback(configModeCallback);

      if (!wifiManager.autoConnect()) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
      }
  } else {
      SetLedToColor(255, 255, 0);
      String ssid = "ESP" + String(ESP.getChipId());
      if (!wifiManager.startConfigPortal(ssid.c_str())) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        ESP.reset();
        delay(5000);
      }
  }


  if (shouldSaveConfig) {
     Serial.println("Saving config. ");
     Serial.println(serverHostAddress);
     // save config
     for (int i = 0; (i < hostSize) && serverHostAddress[i] != '\0' ; i++) {
         EEPROM.write(i, serverHostAddress[i]);
     }
     EEPROM.commit();
  }

  HOST = serverHostAddress;

  platform.setLed(255,255,255);
  breather.attach(0.07, breath);
}

void loop(void) {

#if DEBUG_PROG == 1

RFID id = platform.detectRfidId();
if ((id.id[0] != 0)||(id.id[1] != 0)||(id.id[2] != 0)||(id.id[3] != 0)) {

  Serial.print("found card!");
  platform.setLed(255,0,0);
  delay(1000);
  platform.setLed(255,255,255);
}

#else
    int ret = run(platform);

    if (ret < 0) {
      // client cant connect to host... try again in 5 seconds
      delay(5000);
    }
#endif
//    Serial.println("runned loop");
}
