#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>
#include <Servo.h>
#include "PIDino.hpp"


Servo M1, M2;
QTRSensors qtr;

// Create a PIDino object with the following parameters:
PIDino pid1(500, 0, 0, 10, 0.01, 0.01);

constexpr uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

void setup()
{
    Serial.begin(115200);

    M1.attach(0);
    M2.attach(2);

    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t[]){1, 3}, SensorCount);

    delay(500);
    pinMode(8, OUTPUT);
    digitalWrite(8, LOW);

    for (uint16_t i = 0; i < 200; i++)
    {
        qtr.calibrate();
    }
    digitalWrite(8, HIGH);
}

void loop(){
    uint16_t input = qtr.readLineBlack(sensorValues);

    Serial.println(input);

    const float output = pid1.compute(input); // Compute the PID output
    Serial.println(output); // Print the output to the serial monitor

    M1.writeMicroseconds(output);
    M2.writeMicroseconds(-output);

    delay(10);
}