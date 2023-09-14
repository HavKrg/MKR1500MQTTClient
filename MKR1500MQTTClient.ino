/*

 This sketch connects to mqtt-broker a MKR NB 1500 board, and publishes a sensor reading.

  Parts:
 * MKR NB 1500 board
 * Antenna
 * SIM card with a data plan
 * MKR Relay shield
*/

// libraries
#include <MKRNB.h>
#include <ArduinoLowPower.h>
#include <PubSubClient.h>
#include "arduino_secrets.h"
#include "config.h"
#include "Sensor.h"

// Please enter your sensitive data in the Secret tab or arduino_secrets.h
// PIN Number
const char PINNUMBER[] = SECRET_PINNUMBER;

// initialize the library instance
NBClient client;
NB modem;
GPRS gprs;
PubSubClient mqttClient(client);

void initializeModem();
void connectToGPRS();
void connectToMqttServer();
void readSensorData(Sensor &sensor);
void publishSensorData(Sensor sensor);
void resetBools();
void sleepBoard(int timeToSleepInSeconds);

Sensor waterLevelSensor; // instance of the sensor struct

int analogReadMaxValue;  // max value for the analogRead()-function

void setup() {
  // configure mqtt-client
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setSocketTimeout(30);

  // creates populates the properties of the sensor
  waterLevelSensor = createSensor(WATERLEVELSENSOR_RELAYPIN, WATERLEVELSENSOR_SENSORPIN, WATERLEVELSENSOR_MQTTTOPIC, 
                                  WATERLEVELSENSOR_MINREADING, WATERLEVELSENSOR_MAXREADING, WATERLEVELSENSOR_RETAINREADING);

  // determine if the analogRead() is 10bit or 12 bit and sets 'analogReadMaxValue' to the propper value
  if(HIGHRESOLUTIONANALOGREAD)
  {
    analogReadResolution(12);
    analogReadMaxValue = ANALOGREADMAXVALUE12BIT;
  }
  else
  {
    analogReadResolution(10);
    analogReadMaxValue = ANALOGREADMAXVALUE10BIT;
  }

  // initialize serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Starting MKR1500 sensor reader");
}

char messageBuffer[100];

void loop() {
   if (!Serial) {
    Serial.begin(9600);
    delay(2000); // Wait for 2 seconds to establish the connection
  }
  
  Serial.println("\nnew loop");

  initializeModem();                // initialize the modem
  connectToGPRS();                  // connect to cellular network
  connectToMqttServer();            // connect to MQTT-broker
  readSensorData(waterLevelSensor); // read sensor-data
  publishSensorData(waterLevelSensor); // publish sensor-data       
  //sleepBoard(TIMEBETWEENREADING);   // put arduino in sleep mode
  delay(60000);
}


// initializes modem
void initializeModem() {
  if (modem.status() != NB_READY) 
  {
    while (modem.status() != NB_READY) 
    {
      Serial.print("Initializing modem... ");
      if (modem.begin("", true, true) == NB_READY) 
      {
        Serial.println("OK");
      } else {
        Serial.print("FAILED - ");
        Serial.println(modem.status());
        delay(10000);
      }
    }
  } 
  else 
    Serial.println("Modem already initialized");
}

// connects the board to the cellular network
void connectToGPRS() {
  if (gprs.status() != GPRS_READY) 
  {
    while (gprs.status() != GPRS_READY) 
    {
      Serial.print("Connecting to cellular network... ");
      if (gprs.attachGPRS() == GPRS_READY) 
      {
        Serial.println("OK");
      } else {
        Serial.print("FAILED - ");
        Serial.println(gprs.status());
        delay(10000);
      }
    }
  } 
  else 
    Serial.println("Already connected to cellular network");
}

// connects the board to the mqtt broker
void connectToMqttServer() {
  if (!mqttClient.connected()) 
  {
    while (!mqttClient.connected()) 
    {
      Serial.print("Connecting to MQTT-broker... ");
      if (mqttClient.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASSWORD)) 
      {
        Serial.println("OK");
      } else {
        Serial.print("FAILED - ");
        Serial.println(mqttClient.state());
        delay(10000);
      }
    }
  }
  else
    Serial.println("Already connected to MQTT-broker");
}

// reads a sensor and stores that reading in the sensor-struct property 'lastReadingRaw' then stores a scaled reading in 'lastReadingScaled'
void readSensorData(Sensor &sensor) {
  Serial.print("Reading sensordata ");
  if(sensor.relayPin == -1)
  {
    sensor.lastReadingRaw = analogRead(sensor.sensorPin);
  }
  else
  {
    digitalWrite(sensor.relayPin, HIGH);
    delay(500);
    sensor.lastReadingRaw = analogRead(sensor.sensorPin);
    delay(500);
    digitalWrite(sensor.relayPin, LOW);
  }
  sensor.lastReadingScaled = map(sensor.lastReadingRaw, ANALOGREADMINVALUE, analogReadMaxValue, sensor.minReading, sensor.maxReading);
  Serial.println(sensor.lastReadingScaled);
}

// publishes sensordata to mqtt-broker
void publishSensorData(Sensor sensor) {
  sprintf(messageBuffer, "%d", sensor.lastReadingScaled);
  Serial.print("Publishing sensordata... ");
  if (mqttClient.publish(sensor.mqttTopic, messageBuffer, sensor.retainReading))  // publish to the topic
    Serial.println("OK");
  else
    Serial.println("FAILED");
}

// puts the board to sleep the 'x' seconds
void sleepBoard(int timeToSleepInSeconds)
{
  Serial.print("going to sleep for ");
  Serial.print(timeToSleepInSeconds);
  Serial.println(" seconds...");
  Serial.flush();
  Serial.end();
  LowPower.sleep(timeToSleepInSeconds*1000);      
}