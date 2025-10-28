# RS485-ESP32-MultiSensor-smart-fish-pond
This project integrates multiple environmental sensors (DO, pH, NH4, Turbidity)x2 with ESP32-S using RS485 / Modbus RTU protocol.
The ESP32-S operates in dual mode:

1- Master: Reads sensor data via RS485.

2- Slave: Publishes the same data as Holding Registers for the Samkoon HMI.

Additionally, the data is uploaded to Blynk IoT for cloud monitoring and relay control.

HMI Register Map (Samkoon-samkoon ea-070b)

The ESP32 acts as Slave ID = 10. All sensor data is written into fixed Holding Registers

Blynk Mapping

Each sensor is mapped to a Virtual Pin in Blynk Dashboard

Hardware Setup :-

1- ESP32 + MAX485 (RS485 TTL transceiver)

2- All sensors connected in parallel on the RS485 bus (A/B).

3- Samkoon HMI connected on the same bus (listening to ESP32 ID=10).

4- Relays connected to GPIO pins: 27, 14, 12, 13.
