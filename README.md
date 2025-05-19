# GNSS5-PROJECT
This repository contains code for an IoT device that reads NMEA data from a GNSS module (Walksnail WS-M181) and environmental data from a DHT11 sensor. The data is parsed and published to a public MQTT broker (BRIN).

## Technologies
- ESP32
- GNSS NMEA Protocol
- DHT11 Sensor
- MQTT (MQTTX, MQTT Explorer)

## Project Steps
1. Data acquisition via serial from GNSS
2. Parsing and formatting into JSON
3. Publish to MQTT broker (topic: GNSS5)
4. Casing design using TinkerCAD
