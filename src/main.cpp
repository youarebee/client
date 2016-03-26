#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <lwip/def.h>

#include "secrets.h"
#include "platform.h"
#include "logic.h"

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
  virtual uint32_t detectRfidId() override;

  virtual void getStickId(uint8_t* byteArray) override;

  virtual unsigned long millis() override { return millis();}
  virtual void println(const char* s )override { Serial.println(s);}

  virtual uint32_t hostToNetwork(uint32_t i) {return htonl(i);}
private:
  EspTcpClient _client;
} platform;

TcpClient& DefaultPlatform::getClient() {
  return   _client;
}
uint32_t DefaultPlatform::detectRfidId() {
  return 0;
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
  Serial.begin(9600);

  Serial.print("connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

}

void loop(void) {

    Serial.println("Runnin' loop");
    run(platform);
    Serial.println("runned loop");
}
