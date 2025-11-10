#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>
#include <Servo.h>

Servo M1, M2;
QTRSensors qtr;

constexpr uint8_t SensorCount = 2;
uint16_t sensorValues[SensorCount];

void setup()
{
    Serial.begin(115200);
    M1.attach(0);

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
    uint16_t position = qtr.readLineBlack(sensorValues);

    Serial.println(position);

    M1.writeMicroseconds(map(position, 0, 1000, 1000, 2000));

    delay(10);
}