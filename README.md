# MKR1500MQTTClient
Program that sends sensor-data from an Arduino MKR 1500 to an MQTT-broker

It utilizes the following libraries:

MKRNB
ArduinoLowPower
PubSubClient

Note that it goes to sleep pretty fast, so remember that "double-clicking" the reset button puts the board into bootloader mode, which gives you time to upload an updated scetch. 

Also, I ran into an issue where the board was only able to connect to IP addresses and not urls. 
This was solved by using the ChooseRadioAccessTechnology scetch from MKRNB library to set the Radio Access Technology to CAT M1 only (option 0)