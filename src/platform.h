#ifndef _URB_CLIENT_PLATFORM_H__
#define _URB_CLIENT_PLATFORM_H__

#include <stddef.h>
#include <stdint.h>

#ifndef ARDUINO
void delay(unsigned long);
unsigned long millis();
#else
#include "Arduino.h"
#endif

class TcpClient {
public:

  virtual bool connected() = 0;
  virtual bool connect(const char* host, int port) = 0;
  virtual int  available() = 0;
  virtual void stop() = 0;
  virtual int  read() = 0;
  virtual size_t write(const uint8_t *buf, size_t size) = 0;

private:

};


typedef struct {
  uint8_t id[4];
} RFID;

class Platform {
public:
  virtual TcpClient& getClient() = 0;
  virtual RFID detectRfidId() = 0;
  virtual void getStickId(uint8_t* byteArray) = 0;
  virtual uint32_t hostToNetwork(uint32_t) = 0;
  virtual void println(const char* s ) = 0;
  virtual void  setLed(uint8_t r,uint8_t g,uint8_t b) = 0;

  virtual ~Platform(){}
};


#endif
