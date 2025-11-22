#include <Arduino.h>

#define PWMA 4
#define AIN1 17
#define AIN2 16
#define PWMB 23
#define BIN1 18
#define BIN2 19
#define STBY 5

#define left_coeff 0.825
#define right_coeff 1

void motorLeft(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    if (speed < 0)
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }

    speed = constrain(abs(speed), 0, 255) * left_coeff;

    analogWrite(PWMA, speed);
}

void motorRight(int speed)
{
    if (speed >= 0)
    {
        digitalWrite(BIN1, HIGH);
        digitalWrite(BIN2, LOW);
    }
    if (speed < 0)
    {
        digitalWrite(BIN1, LOW);
        digitalWrite(BIN2, HIGH);
    }

    speed = constrain(abs(speed), 0, 255) * right_coeff;

    analogWrite(PWMB, speed);
}

void initMotor()
{
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH); // always on
    motorLeft(0); // motor off when started
    motorRight(0); // motor off when started
}