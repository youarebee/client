#include "platform.h"

#include "logic.h"
#include <string.h>

const uint16_t PORT = 2323;
const char* HOST = "yuval";

typedef uint8_t NodeId[4];

const int FRAME_SIZE = 16;

struct __attribute__ ((packed)) Events   {
  union {
    uint8_t  data[FRAME_SIZE];
    struct {
      uint8_t  type;
      NodeId  from;
      NodeId  to;
      union {
        struct {
          NodeId  flowerId;
        } TouhcedPacket;
        struct {
          uint8_t  r;
          uint8_t  g;
          uint8_t  b;
        } SetColorPacket;

      } packets;

    } packetFrame;

  } rawPacket;
};

enum {
  CHELLO_PACKET,
  SHELLO_PACKET,
  TOUCHED_PACKET,
  SET_ANIM_PACKET,
  SET_COLOR_PACKET,
  HEARTBREAT_PACKET
};

void handlePacket(const Events& packet, Platform& platform);


void run(Platform& platform) {

    Events frame;
    memset(&frame, 0, sizeof(Events));
    platform.getStickId(frame.rawPacket.packetFrame.from);

    TcpClient& client = platform.getClient();

    if (!client.connected()) {
platform.println("Tryinge to connect to " );
platform.println(HOST);
      if (!client.connect(HOST, PORT)) {
platform.println("can't connect, waiting 0");
        delay(5000);
//platform.println("delayed");
        return;
      } else {
        platform.println("New connection, sending hello.");

        // send client hello
        frame.rawPacket.packetFrame.type = CHELLO_PACKET;
        client.write(frame.rawPacket.data, FRAME_SIZE);
      }
    }

    // read and handle frame:
    // server needs to ping us every 2 minutes; otherwise this might be a
    // half open connection and we timeout and reconnect.
    int32_t timeout = platform.millis() + 120*1000;

    while (client.connected() && (client.available() == 0)) {
      if ((timeout - int32_t(platform.millis())) < 0) {
platform.println(">>> Client Timeout !");
        client.stop();
        return;
      }

delay(0);
      uint32_t id = platform.detectRfidId();
      if (id != 0) {
      platform.println("got rfid, sending touch packet");
        // send touch event to server
        frame.rawPacket.packetFrame.type = TOUCHED_PACKET;
        id = platform.hostToNetwork(id);
        memcpy(frame.rawPacket.packetFrame.packets.TouhcedPacket.flowerId, &id, 4);
        client.write(frame.rawPacket.data, FRAME_SIZE);
      }
    }

    // start to read frame, add a second to the timeout
    timeout = platform.millis() + 1000;


    for (int i = 0; i < 16; i++) {

      platform.println("reading packet byte");

      while (client.connected() && (client.available() == 0)) {
        delay(0);

        if ((timeout - int32_t(platform.millis()))  < 0) {
          platform.println(">>> Client Timeout !");
          client.stop();
          return;
        }
      }
      frame.rawPacket.data[i] = client.read();
    }

    handlePacket(frame, platform);
  }


void handlePacket(const Events& packet, Platform& platform) {

switch (packet.rawPacket.packetFrame.type) {
  case CHELLO_PACKET:
  // can't happen
    break;
  case SHELLO_PACKET:
  // ignore - shouldn't happen
    break;
  case TOUCHED_PACKET:
  // can't happen
    break;
  case SET_ANIM_PACKET:
  // TODO...
    break;
  case SET_COLOR_PACKET:
  // set color!
    // TODO
    break;
  case HEARTBREAT_PACKET:
  // ignore
    break;
}

}
