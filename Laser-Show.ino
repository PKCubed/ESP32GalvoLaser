/*
    This sketch shows the Ethernet event usage

*/

// Important to be defined BEFORE including ETH.h for ETH.begin() to work.
// Example RMII LAN8720 (Olimex, etc.)
#ifndef ETH_PHY_MDC
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#if CONFIG_IDF_TARGET_ESP32
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   23
#define ETH_PHY_MDIO  18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE  ETH_CLOCK_GPIO0_IN
#elif CONFIG_IDF_TARGET_ESP32P4
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   31
#define ETH_PHY_MDIO  52
#define ETH_PHY_POWER 51
#define ETH_CLK_MODE  EMAC_CLK_EXT_IN
#endif
#endif

#include <ETH.h>
#include <ESPAsyncE131.h>
#define UNIVERSE 1                      // First DMX Universe to listen for
#define UNIVERSE_COUNT 1                // Total number of Universes to listen for, starting at UNIVERSE

#include "SPI.h"

#define MCP4922_CS_PIN    32

uint32_t start, stop;

bool laser_state = false;

const int laser_red = 15;
const int laser_green = 14;
const int laser_blue = 13;

int red = 0;
int green = 0;
int blue = 0;

static bool eth_connected = false;

int point1[5] = {0,0,0,0,0}; // x, y, r, g, b
int point2[5] = {0,0,0,0,0};
int point3[5] = {0,0,0,0,0};
int point4[5] = {0,0,0,0,0};
int point5[5] = {0,0,0,0,0};
int point6[5] = {0,0,0,0,0};
int point7[5] = {0,0,0,0,0};
int point8[5] = {0,0,0,0,0};

int line1[7] = {0,0,0,0,0,0,0}; // x1, y1, x2, y2, r, g, b
int line2[7] = {0,0,0,0,0,0,0};
int line3[7] = {0,0,0,0,0,0,0};
int line4[7] = {0,0,0,0,0,0,0};

// ESPAsyncE131 instance with UNIVERSE_COUNT buffer slots
ESPAsyncE131 e131(UNIVERSE_COUNT);

// WARNING: onEvent is called from a separate FreeRTOS task (thread)!
void onEvent(arduino_event_id_t event) {
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      Serial.println("ETH Started");
      // The hostname must be set after the interface is started, but needs
      // to be set before DHCP, so set it from the event handler thread.
      ETH.setHostname("laser-1");
      break;
    case ARDUINO_EVENT_ETH_CONNECTED: Serial.println("ETH Connected"); break;
    case ARDUINO_EVENT_ETH_GOT_IP:
      Serial.println("ETH Got IP");
      Serial.println(ETH);
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
      Serial.println("ETH Lost IP");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
      Serial.println("ETH Disconnected");
      eth_connected = false;
      break;
    case ARDUINO_EVENT_ETH_STOP:
      Serial.println("ETH Stopped");
      eth_connected = false;
      break;
    default: break;
  }
}

void testClient(const char *host, uint16_t port) {
  Serial.print("\nconnecting to ");
  Serial.println(host);

  NetworkClient client;
  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  client.printf("GET / HTTP/1.1\r\nHost: %s\r\n\r\n", host);
  while (client.connected() && !client.available());
  while (client.available()) {
    Serial.write(client.read());
  }

  Serial.println("closing connection\n");
  client.stop();
}

void mcp4922(uint16_t value, uint8_t channel)  //  channel = 0, 1
{
  uint16_t data = 0x3000 | value;
  if (channel == 1) data |= 0x8000;
  digitalWrite(MCP4922_CS_PIN, LOW);
  SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0));
  SPI.transfer((uint8_t)(data >> 8));
  SPI.transfer((uint8_t)(data & 0xFF));
  SPI.endTransaction();
  digitalWrite(MCP4922_CS_PIN, HIGH);
}

void lcolor(int r, int g, int b) {
  if (r>=0) {
    red = r;
  } else {
    red = 0;
  }
  if (g>=0) {
    green = g;
  } else {
    green = 0;
  }
  if (b>=0) {
    blue = b;
  } else {
    blue = 0;
  }
}

void lon() {
  ledcWrite(laser_red, red);
  ledcWrite(laser_green, green);
  ledcWrite(laser_blue, blue);
  laser_state = true;
}

void loff() {
  ledcWrite(laser_red, 0);
  ledcWrite(laser_green, 0);
  ledcWrite(laser_blue, 0);
  laser_state = false;
}

void lcoords(uint16_t x, uint16_t y) {
  if (x >= 4095) {
    x = 4095;
  }
  if (y >= 4095) {
    y = 4095;
  }
  mcp4922(4095-x, 1);
  mcp4922(y, 0);
}

void lline(int Xi, int Yi, int Xf, int Yf, int steps, int duration, bool turnoff, int r, int g, int b) {
  lcolor(r,g,b);
  float mX = (Xf-Xi)/steps;
  float mY = (Yf-Yi)/steps;
  int x = Xi;
  int y = Yi;
  if (!laser_state) {
    lcoords(x,y);
    delayMicroseconds(2500);
    lon();
  }
  for (int i=0; i<=steps; i++) {
    if (x > 4095) {
      x -= 4095;
    }
    if (y > 4095) {
      y -= 4095;
    }
    if (x < 0) {
      x += 4095;
    }
    if (y < 0) {
      y += 4095;
    }
    lcoords(x,y);
    x += mX;
    y += mY;
    delayMicroseconds(duration/steps);
  }
  if (turnoff) {
    delayMicroseconds(400);
    loff();
    delayMicroseconds(400);
  }
}

void lpoint(int x, int y, int duration, int r, int g, int b) {
  if (laser_state) {
    loff();
    delayMicroseconds(400);
  }
  lcoords(x,y);
  delayMicroseconds(2500);
  lcolor(r,g,b);
  lon();
  delayMicroseconds(duration);
  loff();
  delayMicroseconds(400);
}

void setup() {
  Serial.begin(115200);
  Network.onEvent(onEvent);
  ETH.begin();

  delay(5000);

  Serial.println(ETH.localIP());

  if (e131.begin(E131_MULTICAST, UNIVERSE, UNIVERSE_COUNT))   // Listen via Multicast
    Serial.println(F("Listening for data..."));
  else 
    Serial.println(F("*** e131.begin failed ***"));

  pinMode(MCP4922_CS_PIN, OUTPUT);
  pinMode(0, OUTPUT);
  digitalWrite(MCP4922_CS_PIN, HIGH);
  digitalWrite(0, LOW);

  pinMode(laser_green, OUTPUT);
  pinMode(laser_blue, OUTPUT);
  digitalWrite(laser_green, LOW);
  digitalWrite(laser_blue, LOW);

  ledcAttach(laser_red, 100000, 8);
  ledcWrite(laser_red, 0);
  ledcAttach(laser_green, 100000, 8);
  ledcWrite(laser_green, 0);
  ledcAttach(laser_blue, 100000, 8);
  ledcWrite(laser_blue, 0);

  SPI.begin(33, 5, 16, 32);
}

void loop() {
  if (!e131.isEmpty()) {
    e131_packet_t packet;
    e131.pull(&packet);     // Pull packet from ring buffer
    
    if (htons(packet.universe) == 1) {
      point1[0] = (((packet.property_values[1]<<8) + (packet.property_values[2]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point1[1] = (((packet.property_values[3]<<8) + (packet.property_values[4]))>>4);
      point1[2] = packet.property_values[5];
      point1[3] = packet.property_values[6];
      point1[4] = packet.property_values[7];

      point2[0] = (((packet.property_values[9]<<8) + (packet.property_values[10]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point2[1] = (((packet.property_values[11]<<8) + (packet.property_values[12]))>>4);
      point2[2] = packet.property_values[13];
      point2[3] = packet.property_values[14];
      point2[4] = packet.property_values[15];

      point3[0] = (((packet.property_values[17]<<8) + (packet.property_values[18]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point3[1] = (((packet.property_values[19]<<8) + (packet.property_values[20]))>>4);
      point3[2] = packet.property_values[21];
      point3[3] = packet.property_values[22];
      point3[4] = packet.property_values[23];

      point4[0] = (((packet.property_values[25]<<8) + (packet.property_values[26]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point4[1] = (((packet.property_values[27]<<8) + (packet.property_values[28]))>>4);
      point4[2] = packet.property_values[29];
      point4[3] = packet.property_values[30];
      point4[4] = packet.property_values[31];

      point5[0] = (((packet.property_values[33]<<8) + (packet.property_values[34]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point5[1] = (((packet.property_values[35]<<8) + (packet.property_values[36]))>>4);
      point5[2] = packet.property_values[37];
      point5[3] = packet.property_values[38];
      point5[4] = packet.property_values[39];

      point6[0] = (((packet.property_values[41]<<8) + (packet.property_values[42]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point6[1] = (((packet.property_values[43]<<8) + (packet.property_values[44]))>>4);
      point6[2] = packet.property_values[45];
      point6[3] = packet.property_values[46];
      point6[4] = packet.property_values[47];

      point7[0] = (((packet.property_values[49]<<8) + (packet.property_values[50]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point7[1] = (((packet.property_values[51]<<8) + (packet.property_values[52]))>>4);
      point7[2] = packet.property_values[53];
      point7[3] = packet.property_values[54];
      point7[4] = packet.property_values[55];

      point8[0] = (((packet.property_values[57]<<8) + (packet.property_values[58]))>>4); // Bitshift for 16bit position values >> 12 bit position values
      point8[1] = (((packet.property_values[59]<<8) + (packet.property_values[60]))>>4);
      point8[2] = packet.property_values[61];
      point8[3] = packet.property_values[62];
      point8[4] = packet.property_values[63];

      line1[0] = packet.property_values[129]<<4; // x1, y1, x2, y2, r, g, b
      line1[1] = packet.property_values[131]<<4;
      line1[2] = packet.property_values[130]<<4;
      line1[3] = packet.property_values[132]<<4;
      line1[4] = packet.property_values[133];
      line1[5] = packet.property_values[134];
      line1[6] = packet.property_values[135];

      line2[0] = packet.property_values[137]<<4; // x1, y1, x2, y2, r, g, b
      line2[1] = packet.property_values[139]<<4;
      line2[2] = packet.property_values[138]<<4;
      line2[3] = packet.property_values[140]<<4;
      line2[4] = packet.property_values[141];
      line2[5] = packet.property_values[142];
      line2[6] = packet.property_values[143];

      line2[0] = packet.property_values[145]<<4; // x1, y1, x2, y2, r, g, b
      line2[1] = packet.property_values[147]<<4;
      line2[2] = packet.property_values[146]<<4;
      line2[3] = packet.property_values[148]<<4;
      line2[4] = packet.property_values[149];
      line2[5] = packet.property_values[150];
      line2[6] = packet.property_values[151];

      line2[0] = packet.property_values[153]<<4; // x1, y1, x2, y2, r, g, b
      line2[1] = packet.property_values[155]<<4;
      line2[2] = packet.property_values[154]<<4;
      line2[3] = packet.property_values[156]<<4;
      line2[4] = packet.property_values[157];
      line2[5] = packet.property_values[158];
      line2[6] = packet.property_values[159];
    }
    
    //Serial.printf("Universe %u / %u Channels | Packet#: %u / Errors: %u / CH1: %u\n",
    //  htons(packet.universe),                 // The Universe for this packet
    //  htons(packet.property_value_count) - 1, // Start code is ignored, we're interested in dimmer data
    //  e131.stats.num_packets,                 // Packet counter
    //  e131.stats.packet_errors,               // Packet error counter
    //  packet.property_values[1]);             // Dimmer data for Channel 1
  }
  if (point1[2] != 0 || point1[3] != 0 || point1[4] != 0) {
    lpoint(point1[0], point1[1], 1000, point1[2], point1[3], point1[4]);
  }
  if (point2[2] != 0 || point2[3] != 0 || point2[4] != 0) {
    lpoint(point2[0], point2[1], 1000, point2[2], point2[3], point2[4]);
  } 
  if (point3[2] != 0 || point3[3] != 0 || point3[4] != 0) {
    lpoint(point3[0], point3[1], 1000, point3[2], point3[3], point3[4]);
  } 
  if (point4[2] != 0 || point4[3] != 0 || point4[4] != 0) {
    lpoint(point4[0], point4[1], 1000, point4[2], point4[3], point4[4]);
  } 
  if (point5[2] != 0 || point5[3] != 0 || point5[4] != 0) {
    lpoint(point5[0], point5[1], 1000, point5[2], point5[3], point5[4]);
  }
  if (point6[2] != 0 || point6[3] != 0 || point6[4] != 0) {
    lpoint(point6[0], point6[1], 1000, point6[2], point6[3], point6[4]);
  } 
  if (point7[2] != 0 || point7[3] != 0 || point7[4] != 0) {
    lpoint(point7[0], point7[1], 1000, point7[2], point7[3], point7[4]);
  } 
  if (point8[2] != 0 || point8[3] != 0 || point8[4] != 0) {
    lpoint(point8[0], point8[1], 1000, point8[2], point8[3], point8[4]);
  } 

  if (line1[4] != 0 || line1[5] != 0 || line1[6] != 0) {
    lline(line1[0], line1[1], line1[2], line1[3], 100, 1000, true, line1[4], line1[5], line1[6]);
  }
  if (line2[4] != 0 || line2[5] != 0 || line2[6] != 0) {
    lline(line2[0], line2[1], line2[2], line2[3], 100, 1000, true, line2[4], line2[5], line2[6]);
  }
  if (line3[4] != 0 || line3[5] != 0 || line3[6] != 0) {
    lline(line3[0], line3[1], line3[2], line3[3], 100, 1000, true, line3[4], line3[5], line3[6]);
  }
  if (line4[4] != 0 || line4[5] != 0 || line4[6] != 0) {
    lline(line4[0], line4[1], line4[2], line4[3], 100, 1000, true, line4[4], line4[5], line4[6]);
  } 
}
