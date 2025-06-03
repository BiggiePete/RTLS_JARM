#include <Arduino.h>

// Define your LED pin if it's not the default LED_BUILTIN (PA5 for nucleo_f411re)
// If you defined LED_BUILTIN in platformio.ini, you can just use LED_BUILTIN here.
// Otherwise, define it manually:
// #define MY_LED_PIN PA5 // Example, if your LED is on PA5

// We will use Serial1 because we configured PIN_SERIAL1_TX and PIN_SERIAL1_RX
// USART1 is typically mapped to Serial1 in STM32duino

unsigned long previousMillis = 0;
const long interval = 1000; // Interval at which to send message (milliseconds)
uint32_t counter = 0;

void setup()
{
// Initialize the LED pin as an output (optional)
#ifdef MY_LED_PIN
  pinMode(MY_LED_PIN, OUTPUT);
#else
  pinMode(LED_BUILTIN, OUTPUT); // Use the default or platformio.ini defined LED_BUILTIN
#endif

  // Initialize Serial1 at 115200 baud.
  // The pins PA15 (TX) and PB3 (RX) were defined in platformio.ini
  Serial1.begin(115200);

  // Optional: Wait for serial port to connect. Needed for some boards.
  // You can remove this or adjust the timeout if it causes issues.
  // while (!Serial1 && millis() < 5000) {
  //   delay(100);
  // }

  Serial1.println("\nSTM32F411RE Custom Board - Arduino Framework");
  Serial1.print("Serial1 (USART1) on PA15 (TX), PB3 (RX) initialized.\n");
  Serial1.print("SystemCoreClock: ");
  Serial1.print(SystemCoreClock); // SystemCoreClock is updated by the Arduino core
  Serial1.println(" Hz");
}

void loop()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis; // Save the last time a message was sent

    char msg[60];
    sprintf(msg, "Hello from STM32! Count: %lu", counter++);
    Serial1.println(msg);

// Toggle LED (optional)
#ifdef MY_LED_PIN
    digitalWrite(MY_LED_PIN, !digitalRead(MY_LED_PIN));
#else
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
  }

  // You can also check for incoming serial data if needed
  // if (Serial1.available() > 0) {
  //   char receivedChar = Serial1.read();
  //   Serial1.print("Received: ");
  //   Serial1.println(receivedChar);
  // }
}