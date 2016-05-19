#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <lwip/def.h>

#include "secrets.h"
#include "platform.h"
#include "logic.h"

#include "MFRC522.h"


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


#define RED_LED 0
#define GRN_LED 15
#define BLU_LED 4


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

void DefaultPlatform::setLed(uint8_t r,uint8_t g,uint8_t b) {
  Serial.printf("Setting led to #%02X%02X%02X\n", (uint32_t) r,(uint32_t) g,(uint32_t) b);
  analogWrite(BLU_LED, map(b,0,255,0,1023));
  analogWrite(GRN_LED, map(g,0,255,0,1023));
  analogWrite(RED_LED, map(r,0,255,0,1023));
}




void DefaultPlatform::getStickId(uint8_t* byteArray) {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  byteArray[0] = mac[0];
  byteArray[1] = mac[1];
  byteArray[2] = mac[2];
  byteArray[3] = mac[3];
}

void setup(void){

//  analogWriteRange(255);
// analogWriteFreq(1<<9);

//  pinMode(LED_PIN, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GRN_LED, OUTPUT);
  pinMode(BLU_LED, OUTPUT);

    delay(1000);
    digitalWrite(RED_LED, HIGH);
    delay(1000);
    digitalWrite(GRN_LED, HIGH);
    platform.setLed(0, 255, 0);
    delay(1000);
    digitalWrite(BLU_LED, HIGH);
    delay(1000);
  // blue hello
  platform.setLed(0, 0, 255);


  Serial.begin(9600);
  Serial.println(F("Booting...."));

  platform.setLed(255, 0, 0);
  delay(1000);
  platform.setLed(0, 255, 0);
  delay(1000);
  platform.setLed(0, 0, 255);
  delay(1000);

  SPI.begin();	         // Init SPI bus
  mfrc522.PCD_Init();    // Init MFRC522

  Serial.print("wifi - connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {

    platform.setLed(0, 0, 0);
    delay(500);
    platform.setLed(255, 0, 0);
    delay(500);

    Serial.print(".");
  }

  platform.setLed(128,128,128);
}

void loop(void) {
  //  Serial.println("Runnin' loop");
// debug..
// platform.detectRfidId();
    int ret = run(platform);

    if (ret < 0) {
      // client cant connect to host... try again in 5 seconds
      delay(5000);
    }

//    Serial.println("runned loop");
}
