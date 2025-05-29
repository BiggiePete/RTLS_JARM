#include <Arduino.h>

HardwareSerial Serial_Debug(PA15, PB3);

void setup()
{
  Serial_Debug.begin(115200);
  Serial_Debug.println("Hello World");
}
unsigned int x = 0;
void loop()
{
  Serial_Debug.printf("Hello %d\n", x++);
  delay(1000);

  // put your main code here, to run repeatedly:
}
