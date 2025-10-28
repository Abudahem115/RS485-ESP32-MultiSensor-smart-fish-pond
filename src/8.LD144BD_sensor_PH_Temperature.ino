// ================= MODBUS =================
// ESP32 as Master (read pH2 sensor) + Slave (respond to Samkoon HMI)
// Sensor: pH2 (LD144BD) - Reads pH + Temperature
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

// ---------------- Modbus Slave IDs ----------------
#define PH_TEMP_SLAVE_ID  8
#define ESP32_SLAVE_ID    10   // HMI sees ESP32 as ID=10

// ---------------- Blynk Timer ----------------
BlynkTimer timer;

// Blynk Virtual Pins - pH2 Sensor
#define VP_PH_TEMP2   V13
#define VP_PH_LD      V14

// ---------------- Modbus Master Objects ----------------
ModbusRTU mb;
ModbusMaster PHTempSensor;

// ---------------- Sensor Variables ----------------
float ph_temp_value = 0.0f;
float ph_ld_value   = 0.0f;

// ---------------- Error Tracking ----------------
uint16_t sensorErrors = 0;

// ---------------- WiFi Credentials ----------------
const char* WIFI_SSID = "him";
const char* WIFI_PASS = "aaaaaaaa";

// ---------------- Timing Variables ----------------
unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 5000;   // Read every 5 seconds

// ---------------- RS485 Direction Control ----------------
void preTransmission() { digitalWrite(DE_RE, HIGH); }
void postTransmission(){ digitalWrite(DE_RE, LOW); }

// ========================================================================
// Safe Modbus Read
// ========================================================================
bool safeModbusRead(ModbusMaster &sensor, uint16_t address, uint16_t quantity) {
  uint8_t result = sensor.readHoldingRegisters(address, quantity);

  if (result == sensor.ku8MBSuccess) {
    return true;
  } else {
    sensorErrors++;
    if (sensorErrors % 5 == 0) {
      Serial.printf("⚠️  pH2 Sensor Error (Count: %d) - Code: 0x%02X\n",
                    sensorErrors, result);
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

  // Initialize Modbus Master for pH2 Sensor
  PHTempSensor.begin(PH_TEMP_SLAVE_ID, RS485);
  PHTempSensor.preTransmission(preTransmission);
  PHTempSensor.postTransmission(postTransmission);

  // Initialize Modbus Slave for HMI
  mb.begin(&RS485, DE_RE);
  mb.slave(ESP32_SLAVE_ID);
  for (int i = 0; i < 10; i++) mb.addHreg(i, 0);

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║        PH2 SENSOR SYSTEM STARTED       ║");
  Serial.println("║     ESP32 Master + Slave (HMI Mode)    ║");
  Serial.println("╚════════════════════════════════════════╝");

  // WiFi Connection
  Serial.print("🌐 Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  // Blynk Configuration
  Blynk.config(BLYNK_AUTH_TOKEN);

  // Timers
  timer.setInterval(5000L, checkConnections);   // Check WiFi + Blynk
  timer.setInterval(60000L, sendToBlynk);       // Send to Blynk every 60s
}

// ========================================================================
// MAIN LOOP
// ========================================================================
void loop() {
  // Handle Modbus Slave requests
  mb.task();

  // Handle Blynk
  Blynk.run();
  timer.run();

  // Handle pH2 Sensor reading
  handleSensorReading();
}

// ========================================================================
// Non-Blocking Sensor Reading
// ========================================================================
void handleSensorReading() {
  unsigned long now = millis();
  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;
    readPHTempSensor();
    updateRegisters();
    printToDebugger();
  }
}

// ========================================================================
// Read pH2 Sensor (LD144BD)
// ========================================================================
void readPHTempSensor() {
  // Temperature
  if (safeModbusRead(PHTempSensor, 0x0000, 1)) {
    uint16_t raw = PHTempSensor.getResponseBuffer(0);
    ph_temp_value = raw / 10.0f;
  }
  // pH
  if (safeModbusRead(PHTempSensor, 0x0001, 1)) {
    uint16_t raw = PHTempSensor.getResponseBuffer(0);
    ph_ld_value = raw / 10.0f;
  }
}

// ========================================================================
// Update Modbus Holding Registers for HMI
// ========================================================================
void updateRegisters() {
  mb.Hreg(0, (int)(ph_temp_value * 10));
  mb.Hreg(1, (int)(ph_ld_value   * 10));
}

// ========================================================================
// Check WiFi and Blynk Connection
// ========================================================================
void checkConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️  WiFi disconnected! Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
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

  Blynk.virtualWrite(VP_PH_TEMP2, ph_temp_value);
  Blynk.virtualWrite(VP_PH_LD, ph_ld_value);

  Serial.println("📤 pH2 Data sent to Blynk");
}

// ========================================================================
// Debug Output
// ========================================================================
void printToDebugger() {
  Serial.println("\n  ╔══════════════════════════════════════╗");
  Serial.println("  ║            PH2 SENSOR DATA           ║");
  Serial.println("  ╠══════════════════════════════════════╣");
  Serial.printf("  ║  Temperature   : %6.2f °C           ║\n", ph_temp_value);
  Serial.printf("  ║  pH Value      : %6.2f              ║\n", ph_ld_value);
  Serial.println("  ╠══════════════════════════════════════╣");
  Serial.printf("  ║  Errors        : %d                 ║\n", sensorErrors);
  Serial.println("  ╚══════════════════════════════════════╝\n");
}
