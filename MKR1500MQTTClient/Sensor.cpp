#include "Sensor.h"

Sensor createSensor(uint8_t relayPin, uint8_t sensorPin, char* mqttTopic,
                    int minReading, int maxReading, bool retainReading) {
  Sensor s;
  s.relayPin = relayPin;
  s.sensorPin = sensorPin;
  s.mqttTopic = mqttTopic;
  s.minReading = minReading;
  s.maxReading = maxReading;
  s.lastReadingRaw = 0;
  s.lastReadingScaled = 0;
  s.retainReading = retainReading;

  pinMode(s.relayPin, OUTPUT);
  pinMode(s.sensorPin, INPUT);

  return s;
}