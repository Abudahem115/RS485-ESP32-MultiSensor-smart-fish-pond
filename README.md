# RS485-ESP32-MultiSensor-smart-fish-pond
This project integrates multiple environmental sensors (DO, pH, NH4, Turbidity) with ESP32 using RS485 / Modbus RTU protocol.
The ESP32 operates in dual mode:

1- Master: Reads sensor data via RS485.

2- Slave: Publishes the same data as Holding Registers for the Samkoon HMI.

Additionally, the data is uploaded to Blynk IoT for cloud monitoring and relay control.
