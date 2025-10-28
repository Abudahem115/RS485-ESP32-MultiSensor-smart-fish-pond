// ================= MODBUS =================
// ESP32 as Master (read DO sensor) + Slave (respond to Samkoon HMI)
// Sensor: DO (VMS-3002-LDO-N01-20)
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
#define DO_SLAVE_ID      1
#define ESP32_SLAVE_ID   10   // HMI sees ESP32 as ID=10

// ---------------- Blynk Timer ----------------
BlynkTimer timer;

// Blynk Virtual Pins - DO Sensor
#define VP_DO_SAT       V0
#define VP_DO_CONC      V1
#define VP_DO_TEMP      V2

// ---------------- Modbus Master Objects ----------------
ModbusRTU mb;
ModbusMaster DOsensor;

// ---------------- Sensor Variables ----------------
float do_percent    = 0.0f;
float do_mgL        = 0.0f;
float do_temp       = 0.0f;

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
      Serial.printf("⚠️  DO Sensor Error (Count: %d) - Code: 0x%02X\n",
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

  // Initialize Modbus Master for DO Sensor
  DOsensor.begin(DO_SLAVE_ID, RS485);
  DOsensor.preTransmission(preTransmission);
  DOsensor.postTransmission(postTransmission);

  // Initialize Modbus Slave for HMI
  mb.begin(&RS485, DE_RE);
  mb.slave(ESP32_SLAVE_ID);
  for (int i = 0; i < 10; i++) mb.addHreg(i, 0);

  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║        DO SENSOR SYSTEM STARTED        ║");
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

  // Handle DO Sensor reading
  handleSensorReading();
}

// ========================================================================
// Non-Blocking Sensor Reading
// ========================================================================
void handleSensorReading() {
  unsigned long now = millis();
  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;
    readDOSensor();
    updateRegisters();
    printToDebugger();
  }
}

// ========================================================================
// Read DO Sensor (VMS-3002)
// ========================================================================
void readDOSensor() {
  // DO Saturation (%)
  if (safeModbusRead(DOsensor, 0x0000, 2)) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_percent, &raw, sizeof(do_percent));
  }

  // DO Concentration (mg/L)
  if (safeModbusRead(DOsensor, 0x0002, 2)) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_mgL, &raw, sizeof(do_mgL));
  }

  // DO Temperature (°C)
  if (safeModbusRead(DOsensor, 0x0004, 2)) {
    uint16_t hi = DOsensor.getResponseBuffer(0);
    uint16_t lo = DOsensor.getResponseBuffer(1);
    uint32_t raw = ((uint32_t)hi << 16) | lo;
    memcpy(&do_temp, &raw, sizeof(do_temp));
  }
}

// ========================================================================
// Update Modbus Holding Registers for HMI
// ========================================================================
void updateRegisters() {
  mb.Hreg(0, (int)(do_percent * 100));
  mb.Hreg(1, (int)(do_mgL     * 100));
  mb.Hreg(2, (int)(do_temp    * 10));
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

  Blynk.virtualWrite(VP_DO_SAT, do_percent);
  Blynk.virtualWrite(VP_DO_CONC, do_mgL);
  Blynk.virtualWrite(VP_DO_TEMP, do_temp);

  Serial.println("📤 DO Data sent to Blynk");
}

// ========================================================================
// Debug Output
// ========================================================================
void printToDebugger() {
  Serial.println("\n  ╔══════════════════════════════════════╗");
  Serial.println("  ║            DO SENSOR DATA            ║");
  Serial.println("  ╠══════════════════════════════════════╣");
  Serial.printf("  ║  Saturation    : %6.2f %%           ║\n", do_percent);
  Serial.printf("  ║  Concentration : %6.2f mg/L        ║\n", do_mgL);
  Serial.printf("  ║  Temperature   : %6.2f °C          ║\n", do_temp);
  Serial.println("  ╠══════════════════════════════════════╣");
  Serial.printf("  ║  Errors        : %d                 ║\n", sensorErrors);
  Serial.println("  ╚══════════════════════════════════════╝\n");
}
