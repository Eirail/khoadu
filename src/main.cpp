#include <Arduino.h>
#include "lineSensor.h"
#include "motor.h"
#include "distance.h"

#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define trigger1 15
#define echo1 13

float Kp = 75;
float Ki = 0;
float Kd = 0.5;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float pre_error = 0;

int speed1 = 225;
int speed2 = 165;
int speedleft = 200;
int speedright = 200;

bool lineDebug = false;
bool pidDebug = false;

int k = 0;
bool maze = false;

void calculate_error()
{
  readSensorTreated(); // Ä'á»c cáº£m biáº¿n

  if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = 6;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))
    error = 3;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 1.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = 0.5;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -0.5;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -1.5;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = -6;
  // else if (oldsensorRead[0] != sensor[0] || oldsensorRead[1] != sensor[1] || oldsensorRead[2] != sensor[2] || oldsensorRead[3] != sensor[3] || oldsensorRead[4] != sensor[4])
  // {
  //   if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1)
  //   {
  //     countT++;
  //     SerialBT.print(countT);
  //   }
  //   for (int i = 0; i < 5; i++) oldsensorRead[i] = sensor[i];
  // }
  // // pre= !pre;
  // // if (aft == pre) countT++;
  // // SerialBT.print(countT);
  // // aft = pre;

  if (
      !sensor[0] &&
      !sensor[1] &&
      !sensor[2] &&
      !sensor[4] &&
      !sensor[5] && abs(error) <= 1.0)
    error = 0;
}

void calculate_pid()
{
  P = error;
  I = I + error;
  D = error - pre_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  pre_error = error; // feedback
}

void motor_control()
{
  calculate_error(); // tÃ­nh error
  calculate_pid();   // tÃ­nh giÃ¡ trá»‹ PID

  if (error <= 1)
  {
    speedleft = (speed1 + PID_value);  // tÃ­nh tá»'c Ä'á»™ Ä'á»™ng cÆ¡ bÃªn trÃ¡i
    speedright = (speed1 - PID_value); // tÃ­nh tá»'c Ä'á»™ Ä'á»™ng cÆ¡ bÃªn pháº£i
  }
  else
  {
    speedleft = (speed2 + PID_value);  // tÃ­nh tá»'c Ä'á»™ Ä'á»™ng cÆ¡ bÃªn trÃ¡i
    speedright = (speed2 - PID_value); // tÃ­nh tá»'c Ä'á»™ Ä'á»™ng cÆ¡ bÃªn pháº£i
  }

  Serial.printf("e: %0.2f - l: %d - r:%d\n", error, speedleft, speedright);

  motorLeft(speedleft);
  motorRight(speedright);
}

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("khoadu");
  // hcsr04 no1
  pinMode(trigger1, OUTPUT);
  pinMode(echo1, INPUT);

  // init motor
  initMotor();

  // init line sensor
  initLineSensor();

  // init onboard led
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW);

  SerialBT.println("Halted! Send S to start run");
  SerialBT.println("l to enable/disable line debug\np to enable/disable pid debug\ni to show crucial info\n");
  pinMode(0, INPUT);
  while (true)
  {
    if (!digitalRead(0))
      break;
    if (SerialBT.available())
    {
      String inp = SerialBT.readStringUntil('\n');
      inp.trim();
      if (inp == "s")
        break;
      switch (inp[0])
      {
      case 'p':
      {
        Kp = inp.substring(1).toFloat();
        SerialBT.printf("Kp = %.02f\n ", Kp);
        break;
      }
      case 'i':
      {
        Ki = inp.substring(1).toFloat();
        SerialBT.printf("Ki = %.02f\n ", Ki);
        break;
      }
      case 'd':
      {
        Kd = inp.substring(1).toFloat();
        SerialBT.printf("Kd = %.02f\n ", Kd);
        break;
      }
      default:
        break;
      }
    }
  }
  SerialBT.println("Started!");
  while (true)
  {
    float distanceCM = headsensor(trigger1, echo1);
    if (distanceCM == 0)
      break;
    else
    {
      digitalWrite(12, !digitalRead(12));
      delay(250);
    }
  }
}
long long int prev_hcsr_read = 0;

void loop()
{
  if (millis() - prev_hcsr_read >= 100)
  {
    prev_hcsr_read = millis();
    float distanceCM = headsensor(trigger1, echo1);
    // Print the distance on the Serial Monitor
    // SerialBT.print("Distance: ");
    // SerialBT.print(distanceCM);
    // SerialBT.println(" cm");
    if (distanceCM < 4 && abs(error) < 0.5 && distanceCM != 0 && k == 1)
    {
      analogWrite(PWMA, 0);
      analogWrite(PWMB, 0);
      while (true)
      {
        digitalWrite(12, !digitalRead(12));
        delay(500);
      }
    }

    if (distanceCM < 35 && abs(error) < 0.5 && distanceCM != 0 && k == 0) // ne vat can
    {
      motorLeft(0);
      motorRight(0);
      delay(25);
      motorLeft(200); // xoay phai
      motorRight(-200);
      delay(450);
      motorLeft(0); // ngung
      motorRight(0);
      delay(25);
      motorLeft(200); // di thang
      motorRight(200);
      delay(620);
      motorLeft(0); // ngung
      motorRight(0);
      delay(25);
      motorLeft(-200); // xoay trai
      motorRight(200);
      delay(580);
      motorLeft(0); // ngung
      motorRight(0);
      delay(25);
      motorLeft(200); // di thang
      motorRight(200);
      delay(650);
      motorLeft(0); // ngung
      motorRight(0);
      delay(25);
      motorLeft(-200); // xoay trai
      motorRight(200);
      delay(450);
      k++;
    }

  }

  motor_control();

  if (lineDebug)
    SerialBT.printf("%d %d %d %d %d\n", sensor[0], sensor[1], sensor[2], sensor[3], sensor[4]);
  if (pidDebug)
    SerialBT.printf("e: %0.2f - l: %d - r:%d\n", error, speedleft, speedright);

  if (SerialBT.available())
  {
    String inp = SerialBT.readString();
    inp.trim();
    switch (inp[0])
    {
    case 'l':
    {
      lineDebug = !lineDebug;
      break;
    }
    case 'p':
    {
      pidDebug = !pidDebug;
      break;
    }
    case 'f':
    {
      SerialBT.printf("kp: %d, kd: %d, ki: %d\n", Kp, Kd, Ki);
      break;
    }
    case 'i':
    {
      inverted = !inverted;
      break;
    }
    case 'e':
    {
      Serial.println("Robot halted! Reboot");
      motorLeft(0);
      motorRight(0);
      while (true)
        ;
    }
    default:
      break;
    }
  }

  delay(25);
}