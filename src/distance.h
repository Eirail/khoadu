#include <Arduino.h>

float headsensor(int triggerPin, int echoPin)
{
  // Trigger the sensor by setting the TRIG_PIN HIGH for 10 microseconds
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Read the ECHO_PIN, returns the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH, 600);

  // Calculate the distance (speed of sound = 0.0343 cm/µs)
  // Divide by 2 because the signal travels to the object and back
  float distanceCm = (float)duration * 0.034 / 2;

  return distanceCm;
}
