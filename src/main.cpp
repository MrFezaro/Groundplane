#include <Arduino.h>

const int LED_PIN = 8; // Define the GPIO for the LED on SuperMini

void setup()
{
    Serial.begin(115200);            // Start serial communication at 115200 baud rate
    Serial.println("Hello, ESP32!"); // Print message to serial monitor
    pinMode(LED_PIN, OUTPUT);        // Set LED_PIN as output
}

void loop()
{
    Serial.println("loop()");
    digitalWrite(LED_PIN, HIGH); // Turn LED on
    delay(500);                 // Wait 1 second
    digitalWrite(LED_PIN, LOW);  // Turn LED off
    delay(500);                 // Wait 1 second
}