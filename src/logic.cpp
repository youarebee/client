#include "platform.h"

#include "logic.h"
#include <string.h>

int PORT = 2323;
// const char* HOST = "192.168.1.100";
const char* HOST = "192.168.0.111";

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

int run(Platform& platform) {

    Events frame;
    memset(&frame, 0, sizeof(Events));
    platform.getStickId(frame.rawPacket.packetFrame.from);

    TcpClient& client = platform.getClient();

    if (!client.connected()) {
      platform.println("Trying to connect to " );
      platform.println(HOST);
      if (!client.connect(HOST, PORT)) {
        platform.println("can't connect.");
        return -1;
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
    int32_t timeout = millis() + 120*1000;

    while (client.connected() && (client.available() == 0)) {
      if ((timeout - int32_t(millis())) < 0) {
        platform.println(">>> Client Timeout !");
        client.stop();
        return 0;
      }
      // delay activates the wifi stack
      yield();
      RFID id = platform.detectRfidId();
      if ((id.id[0] != 0)||(id.id[1] != 0)||(id.id[2] != 0)||(id.id[3] != 0)) {
      platform.println("got rfid, sending touch packet");
        // send touch event to server
        frame.rawPacket.packetFrame.type = TOUCHED_PACKET;
        memcpy(frame.rawPacket.packetFrame.packets.TouhcedPacket.flowerId, id.id, 4);
        client.write(frame.rawPacket.data, FRAME_SIZE);

      platform.println("got rfid, touch packet sent");
      }
    }

    // start to read frame, add a second to the timeout
    timeout = millis() + 1000;

    platform.println("reading packet from network");
    for (int i = 0; i < 16; i++) {


      while (client.connected() && (client.available() == 0)) {
        yield();

        if ((timeout - int32_t(millis()))  < 0) {
          platform.println(">>> Client Timeout !");
          client.stop();
          return 0;
        }
      }
      frame.rawPacket.data[i] = client.read();
    }

    handlePacket(frame, platform);
    return 0;
  }


void handlePacket(const Events& packet, Platform& platform) {
uint8_t r,g,b;
  platform.println("handle packet");

switch (packet.rawPacket.packetFrame.type) {
  case CHELLO_PACKET:
    platform.println("got chello packet");
  // can't happen
    break;
  case SHELLO_PACKET:
    platform.println("got shello packet");
  // ignore - shouldn't happen
    break;
  case TOUCHED_PACKET:
    platform.println("got touched packet");
  // can't happen
    break;
  case SET_ANIM_PACKET:
    platform.println("got anim packet");
  // TODO...
    break;
  case SET_COLOR_PACKET:
  platform.println("setting led color");

     r = packet.rawPacket.packetFrame.packets.SetColorPacket.r;
     g = packet.rawPacket.packetFrame.packets.SetColorPacket.g;
     b = packet.rawPacket.packetFrame.packets.SetColorPacket.b;
  // set color!
  platform.setLed(r, g, b);


    break;
  case HEARTBREAT_PACKET:
    platform.println("got heartbeat packet");
  // ignore
    break;
  default:
    platform.println("got unknown packet");
}

}
