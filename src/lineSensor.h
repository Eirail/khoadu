#include <Arduino.h>

#define s1 25
#define s2 26
#define s3 27
#define s4 32
#define s5 33

const int sensorLen = 5;
const int sensorPin[5] = {27, 26, 25, 33, 32};
int sensor[5];
const int threshold = 1400;
bool inverted = 1;

void initLineSensor()
{
    pinMode(s1, INPUT);
    pinMode(s2, INPUT);
    pinMode(s3, INPUT);
    pinMode(s4, INPUT);
    pinMode(s5, INPUT);
}

void readSensorRaw()
{
    for (int i = 0; i < sensorLen; i++)
        sensor[i] = analogRead(sensorPin[i]) >= threshold;
}
void readSensorTreated()
{
    for (int i = 0; i < sensorLen; i++)
    {
        sensor[i] = (analogRead(sensorPin[i]) >= threshold) ? !inverted : inverted;
    }
    // for (int i = 0; i < sensorLen; i++){
    // Serial.print(sensor[i] = analogRead(sensorPin[i]));
    // Serial.print(" ");
    // }
}