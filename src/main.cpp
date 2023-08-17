#include <Arduino.h>
#include <CAN.h>

void setup()
{
  Serial.begin(115200);
  initCAN();
}

void loop()
{
  runCAN();
}