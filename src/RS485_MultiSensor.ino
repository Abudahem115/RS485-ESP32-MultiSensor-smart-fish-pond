// ================= MODBUS =================
// ESP32 as Master (read sensors) + Slave (respond to Samkoon HMI)
// Sensors: DO, pH, AMMONIUM, TURBIDITY (Multiple Units)
// Optimized Version - No blocking delays
// ========================================================================

// ---------------- Blynk Configuration ----------------
#define BLYNK_TEMPLATE_ID "TMPL6So33vRJr"
#define BLYNK_TEMPLATE_NAME "test"
#define BLYNK_AUTH_TOKEN "Kob-5oLaUgdMQ8cbqTQxIPJvrd4MXiql"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Arduino.h>
#include <ModbusMaster.h>
#include <ModbusRTU.h>
#include <HardwareSerial.h>

// ---------------- RS485 Pins ----------------
#define DE_RE     4
#define RS485_RX  26
#define RS485_TX  25

HardwareSerial RS485(2);
HardwareSerial RS485_HMI(1);     // UART1 → HMI

// ---------------- Relay 4CH Pins ----------------
#define RELAY1  27
#define RELAY2  14
#define RELAY3  12
#define RELAY4  13

// ---------------- Modbus Slave IDs ----------------
#define DO_SLAVE_ID      1
#define PH_SLAVE_ID      2
#define NH4_SLAVE_ID     3
#define TURB_SLAVE_ID    4
#define DO2_SLAVE_ID     5
#define TURB2_SLAVE_ID   6
#define NH4_2_SLAVE_ID   7
#define PH_TEMP_SLAVE_ID 8 
#define ESP32_SLAVE_ID   10   // HMI sees ESP32 as ID=10

// ---------------- Blynk Timer ----------------
BlynkTimer timer;

// Blynk Virtual Pins - DO Sensor 1
#define VP_DO_SAT       V0
#define VP_DO_CONC      V1
#define VP_DO_TEMP      V2

// Blynk Virtual Pins - pH Sensor
#define VP_PH           V4
#define VP_PH_TEMP      V5

// Blynk Virtual Pins - NH4 Sensor 1
#define VP_NH4          V6

// Blynk Virtual Pins - Turbidity Sensor 1
#define VP_TURB         V7

// Blynk Virtual Pins - DO Sensor 2
#define VP_DO2_MGL      V8
#define VP_DO2_PERCENT  V9
#define VP_DO2_TEMP     V10

// Blynk Virtual Pins - Turbidity Sensor 2
#define VP_TURB2        V11

// Blynk Virtual Pins - NH4 Sensor 2
#define VP_NH4_2        V12

// Blynk Virtual Pins - pH Sensor 2
#define VP_PH_TEMP2     V13
#define VP_PH_LD        V14

// Blynk Virtual Pins - Relay Control
#define VP_RELAY1       V15
#define VP_RELAY2       V16
#define VP_RELAY3       V17
#define VP_RELAY4       V18

// ---------------- Modbus Master Objects ----------------
ModbusRTU mb;
ModbusMaster DOsensor;
ModbusMaster PHSensor;
ModbusMaster NH4Sensor;
ModbusMaster TurbSensor;
ModbusMaster DO2Sensor;
ModbusMaster Turb2Sensor;
ModbusMaster NH4_2_Sensor;
ModbusMaster PHTempSensor;

// ---------------- Sensor Variables ----------------
// DO Sensor 1
float do_percent    = 0.0f;
float do_mgL        = 0.0f;
float do_temp       = 0.0f;

// pH Sensor
float phValue       = 0.0f;
float ph_temp       = 0.0f;

// NH4 Sensor 1
float nh4_value     = 0.0f;

// Turbidity Sensor 1
float turb_value    = 0.0f;

// DO Sensor 2
float do2_mgL       = 0.0f;
float do2_percent   = 0.0f;
float do2_temp      = 0.0f;

// Turbidity Sensor 2
float turb2_value   = 0.0f;

// NH4 Sensor 2
float nh4_2_value   = 0.0f;

// // LD144BD pH + Temp
float ph_temp_value = 0.0f;
float ph_ld_value   = 0.0f;

// ---------------- Error Tracking ----------------
uint16_t sensorErrors[8] = {0};
const char* sensorNames[] = {"DO1", "pH", "NH4_1", "Turb1", "DO2", "Turb2", "NH4_2", "pH_Temp"};

// ---------------- WiFi Credentials ----------------
const char* WIFI_SSID = "him";
const char* WIFI_PASS = "aaaaaaaa";

// ---------------- Timing Variables ----------------
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 5000;           // Read cycle every 5 seconds
unsigned long sensorDelay = 0;
uint8_t currentSensor = 0;                           // Current sensor being read
const unsigned long delayBetweenSensors = 200;       // 200ms between each sensor

// ========================================================================
// Modbus CRC16 Checksum Calculation
// ========================================================================
uint16_t calculateCRC16(uint8_t *buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    
    for (uint8_t i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  
  return crc;
}

// ========================================================================
// Validate Modbus Response Checksum
// ========================================================================
bool validateModbusChecksum(ModbusMaster &sensor, uint8_t responseLength) {
  if (responseLength < 2) return false;
  
  // Get response buffer
  uint8_t buffer[256];
  buffer[0] = sensor.getResponseBuffer(0) >> 8;  // Slave ID
  buffer[1] = sensor.getResponseBuffer(0) & 0xFF; // Function code
  
  for (uint8_t i = 0; i < responseLength; i++) {
    uint16_t word = sensor.getResponseBuffer(i);
    buffer[2 + i*2] = word >> 8;
    buffer[2 + i*2 + 1] = word & 0xFF;
  }
  
  uint16_t calculatedCRC = calculateCRC16(buffer, responseLength * 2 + 2);
  uint16_t receivedCRC = sensor.getResponseBuffer(responseLength);
  
  return (calculatedCRC == receivedCRC);
}

// ---------------- RS485 Direction Control ----------------
void preTransmission() {
  digitalWrite(DE_RE, HIGH);
}

void postTransmission() {
  digitalWrite(DE_RE, LOW);
}

// ========================================================================
// Safe Modbus Read with Checksum Validation
// ========================================================================
bool safeModbusRead(ModbusMaster &sensor, uint16_t address, uint16_t quantity, 
                    uint8_t sensorIndex, const char* sensorName) {
  uint8_t result = sensor.readHoldingRegisters(address, quantity);
  
  if (result == sensor.ku8MBSuccess) {
    // Modbus Master library handles CRC automatically
    // Additional validation can be done here if needed
    return true;
  } else {
    sensorErrors[sensorIndex]++;
    if (sensorErrors[sensorIndex] % 5 == 0) {  // Report every 5 errors
      Serial.printf("⚠️  %s Error (Count: %d) - Code: 0x%02X\n", 
                    sensorName, sensorErrors[sensorIndex], result);
    }
    return false;
  }
}

// ========================================================================
// SETUP
// ========================================================================
void setup() {
  // RS485 Direction Control
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);

  // Serial Communication
  Serial.begin(9600);
  RS485.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // Initialize Modbus Master for DO Sensor 1
  DOsensor.begin(DO_SLAVE_ID, RS485);
  DOsensor.preTransmission(preTransmission);
  DOsensor.postTransmission(postTransmission);

  // Initialize Modbus Master for pH Sensor
  PHSensor.begin(PH_SLAVE_ID, RS485);
  PHSensor.preTransmission(preTransmission);
  PHSensor.postTransmission(postTransmission);

  // Initialize Modbus Master for NH4 Sensor 1
  NH4Sensor.begin(NH4_SLAVE_ID, RS485);
  NH4Sensor.preTransmission(preTransmission);
  NH4Sensor.postTransmission(postTransmission);

  // Initialize Modbus Master for Turbidity Sensor 1
  TurbSensor.begin(TURB_SLAVE_ID, RS485);
  TurbSensor.preTransmission(preTransmission);
  TurbSensor.postTransmission(postTransmission);

  // Initialize Modbus Master for DO Sensor 2
  DO2Sensor.begin(DO2_SLAVE_ID, RS485);
  DO2Sensor.preTransmission(preTransmission);
  DO2Sensor.postTransmission(postTransmission);

  // Initialize Modbus Master for Turbidity Sensor 2
  Turb2Sensor.begin(TURB2_SLAVE_ID, RS485);
  Turb2Sensor.preTransmission(preTransmission);
  Turb2Sensor.postTransmission(postTransmission);

  // Initialize Modbus Master for NH4 Sensor 2
  NH4_2_Sensor.begin(NH4_2_SLAVE_ID, RS485);
  NH4_2_Sensor.preTransmission(preTransmission);
  NH4_2_Sensor.postTransmission(postTransmission);

  // Initialize Modbus Master for pH Sensor 2
  PHTempSensor.begin(PH_TEMP_SLAVE_ID, RS485);
  PHTempSensor.preTransmission(preTransmission);
  PHTempSensor.postTransmission(postTransmission);

  // Relay Setup
  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  pinMode(RELAY4, OUTPUT);

  // Default Relays OFF (HIGH = OFF for active LOW relay)
  digitalWrite(RELAY1, HIGH);
  digitalWrite(RELAY2, HIGH);
  digitalWrite(RELAY3, HIGH);
  digitalWrite(RELAY4, HIGH);

  // Initialize Modbus Slave for HMI
  mb.begin(&RS485, DE_RE);
  mb.slave(ESP32_SLAVE_ID);
  
  // Add Holding Registers (0-24) 
  for (int i = 0; i < 25; i++) mb.addHreg(i, 0);

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║    RS485 Multi-Sensor System Started   ║");
  Serial.println("║        ESP32 Master + Slave Mode       ║");
  Serial.println("╚════════════════════════════════════════╝");

  // WiFi Connection
  Serial.print("🌐 Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Blynk Configuration
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Blynk Timers
  timer.setInterval(5000L, checkConnections);   // Check WiFi + Blynk every 5s
  timer.setInterval(60000L, sendToBlynk);       // Send to Blynk every 60s

  Serial.println("✅ Setup Complete!");
  Serial.println();
}

// ========================================================================
// MAIN LOOP - Non-blocking Design
// ========================================================================
void loop() {
  // Priority 1: Handle Modbus Slave requests from HMI
  mb.task();

  // Priority 2: Handle Blynk connection
  Blynk.run();
  timer.run();

  // Priority 3: Sequential sensor reading without blocking
  handleSensorReading();

  // Priority 4: Check for serial commands
  checkSerialCommands();
}

// ========================================================================
// Non-Blocking Sensor Reading Handler
// ========================================================================
void handleSensorReading() {
  unsigned long now = millis();

  if (currentSensor == 0 && (now - lastSensorRead >= sensorInterval)) {
    lastSensorRead = now;
    sensorDelay = now;
    currentSensor = 1;
    Serial.println("\n🔄 Starting sensor cycle (CRC validated)...");
  }

  if (currentSensor > 0 && (now - sensorDelay >= delayBetweenSensors)) {
    sensorDelay = now;

    switch (currentSensor) {
      case 1:
        readDOSensor();
        Serial.println("  📊 [1/8] DO Sensor");
        currentSensor++;
        break;
      case 2:
        readPHSensor();
        Serial.println("  📊 [2/8] pH Sensor");
        currentSensor++;
        break;
      case 3:
        readNH4Sensor();
        Serial.println("  📊 [3/8] NH4 Sensor");
        currentSensor++;
        break;
      case 4:
        readTurbSensor();
        Serial.println("  📊 [4/8] Turbidity Sensor");
        currentSensor++;
        break;
      case 5:
        readDO2Sensor();
        Serial.println("  📊 [5/8] DO 2 Sensor");
        currentSensor++;
        break;
      case 6:
        readTurb2Sensor();
        Serial.println("  📊 [6/8] Turbidity 2 Sensor");
        currentSensor++;
        break;
      case 7:
        readNH4_2_Sensor();
        Serial.println("  📊 [7/8] NH4_2 Sensor");
        currentSensor++;
        break;
      case 8:
        readPHTempSensor();
        Serial.println("  📊 [8/8] pH 2 Sensor");
        currentSensor++;
        break;
      case 9:
        updateRegisters();
        printToDebugger();
        currentSensor = 0;
        Serial.println("✅ Cycle complete!\n");
        break;
    }
  }
}

// ========================================================================
// Read DO Sensor 1 (VMS-3002-LDO-N01-20)
// ========================================================================
void readDOSensor() {

  // Read DO Saturation (%)
  if (safeModbusRead(DOsensor, 0x0000, 2, 0, "DO1-Sat")) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_percent, &raw, sizeof(do_percent));
  }

  // Read DO Concentration (mg/L)
  if (safeModbusRead(DOsensor, 0x0002, 2, 0, "DO1-Conc")) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_mgL, &raw, sizeof(do_mgL));
  }

  // Read DO Temperature (°C)
  if (safeModbusRead(DOsensor, 0x0004, 2, 0, "DO1-Temp")) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_temp, &raw, sizeof(do_temp));
  }

}

// ========================================================================
// Read pH Sensor (PHT-485)
// ========================================================================
void readPHSensor() {
  if (safeModbusRead(PHSensor, 0x0000, 2, 1, "pH")) {
    uint16_t temp_raw = PHSensor.getResponseBuffer(0);
    uint16_t ph_raw = PHSensor.getResponseBuffer(1);
    ph_temp = temp_raw / 10.0f;
    phValue = ph_raw / 10.0f;
  }
}

// ========================================================================
// Read Ammonium Sensor 1 (RK500-15)
// ========================================================================
void readNH4Sensor() {
  if (safeModbusRead(NH4Sensor, 0x0000, 2, 2, "NH4_1")) {
    uint16_t hi = NH4Sensor.getResponseBuffer(0);
    uint16_t lo = NH4Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&nh4_value, &raw, sizeof(nh4_value));
  }
}

// ========================================================================
// Read Turbidity Sensor 1 (PK500-07)
// ========================================================================
void readTurbSensor() {
  if (safeModbusRead(TurbSensor, 0x0000, 2, 3, "Turb1")) {
    uint16_t hi = TurbSensor.getResponseBuffer(0);
    uint16_t lo = TurbSensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&turb_value, &raw, sizeof(turb_value));
  }
}

// ========================================================================
// Read DO Sensor 2 (RK500-04)
// ========================================================================
void readDO2Sensor() {

  // Read DO2 Concentration (mg/L)
  if (safeModbusRead(DO2Sensor, 0x0000, 2, 4, "DO2-Conc")) {
    uint16_t hi = DO2Sensor.getResponseBuffer(0);
    uint16_t lo = DO2Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do2_mgL, &raw, sizeof(do2_mgL));
  }

  // Read DO2 Saturation (%)
  if (safeModbusRead(DO2Sensor, 0x0002, 2, 4, "DO2-Sat")) {
    uint16_t hi = DO2Sensor.getResponseBuffer(0);
    uint16_t lo = DO2Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do2_percent, &raw, sizeof(do2_percent));
  }

  // Read DO2 Temperature (°C)
  if (safeModbusRead(DO2Sensor, 0x0004, 2, 4, "DO2-Temp")) {
    uint16_t hi = DO2Sensor.getResponseBuffer(0);
    uint16_t lo = DO2Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do2_temp, &raw, sizeof(do2_temp));
  }
}

// ========================================================================
// Read Turbidity Sensor 2 (RK500-07)
// ========================================================================
void readTurb2Sensor() {
  if (safeModbusRead(Turb2Sensor, 0x0000, 2, 5, "Turb2")) {
    uint16_t hi = Turb2Sensor.getResponseBuffer(0);
    uint16_t lo = Turb2Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&turb2_value, &raw, sizeof(turb2_value));
  }
}

// ========================================================================
// Read Ammonium Sensor 2 (RK500-15)
// ========================================================================
void readNH4_2_Sensor() {
  if (safeModbusRead(NH4_2_Sensor, 0x0000, 2, 6, "NH4_2")) {
    uint16_t hi = NH4_2_Sensor.getResponseBuffer(0);
    uint16_t lo = NH4_2_Sensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&nh4_2_value, &raw, sizeof(nh4_2_value));
  }
}

// ========================================================================
// Read LD144BD Sensor 2 (pH + Temperature)
// ========================================================================
void readPHTempSensor() {
  // Read PH
  if (safeModbusRead(PHTempSensor, 0x0000, 1, 7, "pH_Temp-T")) {
    uint16_t raw = PHTempSensor.getResponseBuffer(0);
    ph_temp_value = raw / 10.0f;
  }
  // Read PH-Temp
  if (safeModbusRead(PHTempSensor, 0x0001, 1, 7, "pH_Temp-pH")) {
    uint16_t raw = PHTempSensor.getResponseBuffer(0);
    ph_ld_value = raw / 10.0f;
  }
}


// ========================================================================
// Update Modbus Holding Registers for HMI
// ========================================================================
void updateRegisters() {
  // Registers 0-3: DO Sensor 1
  mb.Hreg(0, (int)(do_percent  * 100));
  mb.Hreg(1, (int)(do_mgL      * 100));
  mb.Hreg(2, (int)(do_temp     * 10));

  // Register 4: pH Sensor
  mb.Hreg(4, (int)(phValue * 100));
  mb.Hreg(5, (int)(ph_temp * 100));

  // Register 6: NH4 Sensor 1
  mb.Hreg(6, (int)(nh4_value * 100));

  // Register 7: Turbidity Sensor 1
  mb.Hreg(7, (int)(turb_value * 100));

  // Registers 8-10: DO Sensor 2
  mb.Hreg(8,  (int)(do2_mgL     * 100));
  mb.Hreg(9,  (int)(do2_percent * 100));
  mb.Hreg(10, (int)(do2_temp    * 10));

  // Register 11: Turbidity Sensor 2
  mb.Hreg(11, (int)(turb2_value * 100));

  // Register 12: NH4 Sensor 2
  mb.Hreg(12, (int)(nh4_2_value * 100));

  // Register 13-14 LD144BD Temperature & pH
  //mb.Hreg(13, (int)(ph_temp_value * 10));
  mb.Hreg(14, (int)(ph_ld_value * 10));


  // Registers 15-18: Relay Status (1=ON, 0=OFF)
  //mb.Hreg(15, digitalRead(RELAY1) == LOW ? 1 : 0);
  //mb.Hreg(16, digitalRead(RELAY2) == LOW ? 1 : 0);
  //mb.Hreg(17, digitalRead(RELAY3) == LOW ? 1 : 0);
  //mb.Hreg(18, digitalRead(RELAY4) == LOW ? 1 : 0);
}

// ========================================================================
// Check WiFi and Blynk Connection
// ========================================================================
void checkConnections() {
  // Check WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi disconnected! Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    // WiFi connected, check Blynk
    if (!Blynk.connected()) {
      Serial.println("⚠️  Blynk disconnected! Reconnecting...");
      Blynk.connect(3000);
    }
  }
}

// ========================================================================
// Send Data to Blynk
// ========================================================================
void sendToBlynk() {
  if (!Blynk.connected()) return;

  // DO Sensor 1
  Blynk.virtualWrite(VP_DO_SAT, do_percent);
  Blynk.virtualWrite(VP_DO_CONC, do_mgL);
  Blynk.virtualWrite(VP_DO_TEMP, do_temp);

  // pH Sensor
  Blynk.virtualWrite(VP_PH, phValue);
  Blynk.virtualWrite(VP_PH_TEMP, ph_temp);

  // NH4 Sensor 1
  Blynk.virtualWrite(VP_NH4, nh4_value);

  // Turbidity Sensor 1
  Blynk.virtualWrite(VP_TURB, turb_value);

  // DO Sensor 2
  Blynk.virtualWrite(VP_DO2_MGL, do2_mgL);
  Blynk.virtualWrite(VP_DO2_PERCENT, do2_percent);
  Blynk.virtualWrite(VP_DO2_TEMP, do2_temp);

  // Turbidity Sensor 2
  Blynk.virtualWrite(VP_TURB2, turb2_value);

  // NH4 Sensor 2
  Blynk.virtualWrite(VP_NH4_2, nh4_2_value);

  // pH Sensor 2
  Blynk.virtualWrite(VP_PH_LD, ph_ld_value);
  Blynk.virtualWrite(VP_PH_TEMP2, ph_temp_value);  

  Serial.println("📤 Data sent to Blynk");
}

// ========================================================================
// Blynk Relay Control Handlers
// ========================================================================
BLYNK_WRITE(VP_RELAY1) {
  int val = param.asInt();
  digitalWrite(RELAY1, val ? LOW : HIGH);
  Serial.printf("🔌 Relay 1: %s\n", val ? "ON" : "OFF");
}

BLYNK_WRITE(VP_RELAY2) {
  int val = param.asInt();
  digitalWrite(RELAY2, val ? LOW : HIGH);
  Serial.printf("🔌 Relay 2: %s\n", val ? "ON" : "OFF");
}

BLYNK_WRITE(VP_RELAY3) {
  int val = param.asInt();
  digitalWrite(RELAY3, val ? LOW : HIGH);
  Serial.printf("🔌 Relay 3: %s\n", val ? "ON" : "OFF");
}

BLYNK_WRITE(VP_RELAY4) {
  int val = param.asInt();
  digitalWrite(RELAY4, val ? LOW : HIGH);
  Serial.printf("🔌 Relay 4: %s\n", val ? "ON" : "OFF");
}

// ========================================================================
// Debug Output
// ========================================================================
void printToDebugger() {
  Serial.println("\n  ╔══════════════════════════════════════════╗");
  Serial.println("  ║       SENSOR READINGS (CRC VALIDATED)    ║");
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║             DO SENSOR 1                  ║");
  Serial.printf("  ║  Saturation    : %6.2f %%                ║\n", do_percent);
  Serial.printf("  ║  Concentration : %6.2f mg/L             ║\n", do_mgL);
  Serial.printf("  ║  Temperature   : %6.2f °C               ║\n", do_temp);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║              pH SENSOR                   ║");
  Serial.printf("  ║  pH            : %6.2f                  ║\n", phValue);
  Serial.printf("  ║  Temperature   : %6.2f °C               ║\n", ph_temp);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║           AMMONIUM SENSOR 1              ║");
  Serial.printf("  ║  NH4+          : %6.2f mg/L             ║\n", nh4_value);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║         TURBIDITY SENSOR 1               ║");
  Serial.printf("  ║  Turbidity     : %6.2f NTU              ║\n", turb_value);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║             DO SENSOR 2                  ║");
  Serial.printf("  ║  Concentration : %6.2f mg/L             ║\n", do2_mgL);
  Serial.printf("  ║  Saturation    : %6.2f %%                ║\n", do2_percent);
  Serial.printf("  ║  Temperature   : %6.2f °C               ║\n", do2_temp);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║         TURBIDITY SENSOR 2               ║");
  Serial.printf("  ║  Turbidity     : %6.2f NTU              ║\n", turb2_value);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║          AMMONIUM SENSOR 2               ║");
  Serial.printf("  ║  NH4+          : %6.2f mg/L             ║\n", nh4_2_value);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║          pH TEMP SENSOR                  ║");
  Serial.printf("  ║  Temperature   : %6.2f °C               ║\n", ph_temp_value);
  Serial.printf("  ║  pH            : %6.2f                  ║\n", ph_ld_value);
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║           RELAY STATUS                   ║");
  Serial.printf("  ║  Relay 1: %-3s  Relay 2: %-3s              ║\n", 
    digitalRead(RELAY1)==LOW?"ON":"OFF", 
    digitalRead(RELAY2)==LOW?"ON":"OFF");
  Serial.printf("  ║  Relay 3: %-3s  Relay 4: %-3s              ║\n", 
    digitalRead(RELAY3)==LOW?"ON":"OFF", 
    digitalRead(RELAY4)==LOW?"ON":"OFF");
  Serial.println("  ╠══════════════════════════════════════════╣");
  Serial.println("  ║         ERROR STATISTICS                 ║");
  for (int i = 0; i < 8; i++) {
    if (sensorErrors[i] > 0) {
      Serial.printf("  ║  %-10s : %4d errors                ║\n", 
        sensorNames[i], sensorErrors[i]);
    }
  }
  Serial.println("  ╚══════════════════════════════════════════╝\n");
}

// ========================================================================
// Serial Commands for Manual Relay Control
// ========================================================================
void checkSerialCommands() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case '1':
        digitalWrite(RELAY1, LOW);
        Serial.println("✅ Relay 1 → ON");
        break;
      case '2':
        digitalWrite(RELAY1, HIGH);
        Serial.println("❌ Relay 1 → OFF");
        break;
      case '3':
        digitalWrite(RELAY2, LOW);
        Serial.println("✅ Relay 2 → ON");
        break;
      case '4':
        digitalWrite(RELAY2, HIGH);
        Serial.println("❌ Relay 2 → OFF");
        break;
      case '5':
        digitalWrite(RELAY3, LOW);
        Serial.println("✅ Relay 3 → ON");
        break;
      case '6':
        digitalWrite(RELAY3, HIGH);
        Serial.println("❌ Relay 3 → OFF");
        break;
      case '7':
        digitalWrite(RELAY4, LOW);
        Serial.println("✅ Relay 4 → ON");
        break;
      case '8':
        digitalWrite(RELAY4, HIGH);
        Serial.println("❌ Relay 4 → OFF");
        break;
      case 's':
      case 'S':
        printToDebugger();
        break;
      default:
        Serial.println("❓ Unknown command. Use 1-8 for relays, S for status");
        break;
    }
  }
}