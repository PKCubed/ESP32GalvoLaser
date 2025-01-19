#include "SPI.h"

#define MCP4922_CS_PIN    32

uint32_t start, stop;


float theta = 0;
int value = 0;

int laser_pin = 13;

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);

  pinMode(MCP4922_CS_PIN, OUTPUT);
  pinMode(0, OUTPUT);
  digitalWrite(MCP4922_CS_PIN, HIGH);
  digitalWrite(0, LOW);

  pinMode(laser_pin, OUTPUT);
  digitalWrite(laser_pin, LOW);

  SPI.begin(33, 15, 16, 32);
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

void lon() {
  digitalWrite(laser_pin, HIGH);
}

void loff() {
  digitalWrite(laser_pin, LOW);
}

void lcoords(uint16_t x, uint16_t y) {
  mcp4922(4095-x, 1);
  mcp4922(y, 0);
}

void lline(uint16_t Xi, uint16_t Yi, uint16_t Xf, uint16_t Yf, int steps, long duration) {
  uint16_t mX = (Xf-Xi)/steps;
  uint16_t mY = (Yf-Yi)/steps;
  uint16_t x = Xi-mX;
  uint16_t y = Yi-mY;
  lcoords(x,y);
  delayMicroseconds(1000);
  lon();
  for (int i=0; i<=steps; i++) {
    lcoords(x,y);
    x += mX;
    y += mY;
    delayMicroseconds(duration/steps);
  }
  loff();
}

int timing = 50000;

void loop()
{
  lline(0,1000,0,3000,100,timing);
  lline(0,3000,500,3000,100,timing);
  lline(500,3000,500,2000,100,timing);
  lline(500,2000,0,2000,100,timing);

  lline(1000,1000,1000,3000,100,timing);
  lline(1500,3000,1000,2000,100,timing);
  lline(1000,2000,1500,1000,100,timing);
}