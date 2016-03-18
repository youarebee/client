#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "secrets.h"



void setup(void){
  WiFi.begin(ssid, password);
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  MDNS.begin("flower");
  //  MDNS.queryService("tree", "tcp");

}

WiFiClient client;

void loop(void){

  uint8_t frame[16];
  if (!client.connected() != 0) {
    if (!client.connect("uival", 80)) {
      delay(5000);
      return;
    }

  }

  // read and handle frame:
  // server needs to ping us every 2 minutes; otherwise this might be a
  // half open connection and we timeout and reconnect.
  int timeout = millis() + 120*1000;

  while (client.available() == 0) {
    if (timeout - millis() < 0) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }

    // TODO add check RFID here, and send command to server if needed
  }

  // start to read frame, add a second to the timeout
  timeout += 1000;


  for (int i = 0;i < 16;i++) {

    if (timeout - millis() < 0) {
      Serial.println(">>> Client Timeout !");
      client.stop();
      return;
    }
  }

}
