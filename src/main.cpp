#include <Arduino.h>
#include <CAN.h>

CANhandler can;

void setup()
{
  Serial.begin(115200);
  can.initCAN();
}

void loop()
{
  can.runCAN();
}