#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <Wire.h>
#include <Servo.h>
#include <PIDino.hpp>

Servo M1, M2;
QTRSensors qtr;
PIDino PID1(500, 1, 0, 0, 1000);


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
    const uint16_t input = qtr.readLineBlack(sensorValues);

    const float pidOut = PID1.compute(input);

    // Calculate motor signals
    int m1 = 1200 + (pidOut - 1000);
    int m2 = 1200 + (1000 - pidOut);


    // Clamp values to safe range
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);

    M1.writeMicroseconds(m1);
    M2.writeMicroseconds(m2);

    Serial.print("PID: "); Serial.print(pidOut);
    Serial.print("  M1: "); Serial.print(m1);
    Serial.print("  M2: "); Serial.println(m2);
    delay(10);
}