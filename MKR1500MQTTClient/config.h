// name for the MQTT-client

// Sensor configuration for a waterlevel-sensor
#define WATERLEVELSENSOR_RELAYPIN 1 // relay pin for the sensor -1 if it doesn't use a relay
#define WATERLEVELSENSOR_SENSORPIN A1 // pin for the sensor is connected to
#define WATERLEVELSENSOR_MQTTTOPIC "hv/inntak/tank/niva" // mqtt topic
#define WATERLEVELSENSOR_MINREADING 0 // minimum reading for the sensor, used for scaling
#define WATERLEVELSENSOR_MAXREADING 500 // maximum reading for the sensor, used for scaling
#define WATERLEVELSENSOR_RETAINREADING true // enable or disable the retained tag when publishing a reading to the mqtt-broker

// Enable or disable 12-bit resolution for analogRead()
#define HIGHRESOLUTIONANALOGREAD true

// Number of seconds the board goes to sleep between readings.
#define TIMEBETWEENREADING 3600

// constants for min/max value of analogRead()
#define ANALOGREADMINVALUE 0
#define ANALOGREADMAXVALUE10BIT 1023
#define ANALOGREADMAXVALUE12BIT 4095

