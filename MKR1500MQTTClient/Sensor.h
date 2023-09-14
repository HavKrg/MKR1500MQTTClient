#include <Arduino.h>

struct Sensor {
  uint8_t relayPin;
  uint8_t sensorPin;
  char* mqttTopic;
  int minReading;
  int maxReading;
  int lastReadingRaw;
  int lastReadingScaled;
  bool retainReading;
};

Sensor createSensor(uint8_t relayPin, uint8_t sensorPin, char* mqttTopic,
                    int minReading, int maxReading, bool retainReading);
