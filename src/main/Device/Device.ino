#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <HardwareSerial.h>
#include <HX711_ADC.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

#include <WiFi.h>
#include <AsyncTCP.h>
// #include <ESPAsyncWebServer.h>
#include <WebServer.h>
#include <ElegantOTA.h>

#include <DNSServer.h>
#include <SPIFFS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// WiFi credentials /////////////////////////////////////////////////////////////////
// #define WIFI_SSID "SAMURAI@CREEDS_2.4G"
// #define WIFI_PASSWORD "samurai1@creeds"

// WiFi Manager Configuration
const char *AP_SSID = "ESP32_Setup";
const char *AP_PASSWORD = "12345678";
const char *WIFI_FILE = "/wifi_credentials.json";
const int LED_PIN = 2; // Onboard LED
const int MAX_NETWORKS = 20;

// Function declarations
// Function declarations
bool loadWiFiCredentials(String &ssid, String &password);
void saveWiFiCredentials(const String &ssid, const String &password);
void setupWiFiAP();
bool connectToWiFi(const String &ssid, const String &password);
String getWifiNetworkList();
void handleRoot();
void handleConnect();
void handleNotFound();
void blinkError(int count);
void initializeSensors();
void initializeFirebase();
void startTasks();
//////////////////////////////////////////////////////////////////////////////////////////

// Captive Portal
const byte DNS_PORT = 53;
IPAddress apIP(192, 168, 4, 1);

// Web Server
// AsyncWebServer server(80);
WebServer server(80);
DNSServer dnsServer;
bool wifiConnected = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long connectionTimeout = 30000;

// BME680 (SPI)
#define BME_CS 5    // GPIO5 for - (CS)
#define BME_MOSI 23 // VSPI MOSI - (SDA)
#define BME_MISO 19 // VSPI MISO - (SD0)
#define BME_SCK 18  // VSPI SCK  - (SCL)

// PMS7003 (UART Serial2)
#define PMS_RX_PIN 16 // RX2 (connects to PMS7003 TX)
#define PMS_TX_PIN 17 // TX2 (connects to PMS7003 RX)

// Gas Sensors
#define MQ6_ANALOG_PIN 34 // LPG sensor
#define MQ7_ANALOG_PIN 35 // CO sensor

#define MICS4514_CO_PIN 36  // CO sensor
#define MICS4514_NO2_PIN 39 // NO2 sensor (VN)

// HX711 Load Cell
#define HX711_DOUT_PIN 25 // Digital input
#define HX711_SCK_PIN 26  // Digital output

// Calibration values
#define MQ6_RO_CLEAN_AIR_FACTOR 10.0
#define MQ7_RO_CLEAN_AIR_FACTOR 27.0
#define HX711_CALIBRATION_FACTOR -2689.7 // -21667.333984  // -20933.333984  -37170.00

#define EEPROM_SIZE 64
#define CALIBRATION_FACTOR_ADDR 0
#define TARE_VALUE_ADDR 8
#define CALIBRATION_VALID_ADDR 16

// Firebase credentials //////////////////////////////////////////////////////////////////////////
bool isDB = false;
#define Web_API_KEY "AIzaSyAaiE2hCxCOIzy39Gq_BW8KlD_bUSR6Tyw"
#define DATABASE_URL "https://esp-gas-ai-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "satoru.thilina@gmail.com"
#define USER_PASS "123456"

// Firebase components
UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
// using AsyncClient = AsyncClientClass;
// AsyncClient aClient(ssl_client);
AsyncClientClass aClient(ssl_client);
RealtimeDatabase Database;
//////////////////////////////////////////////////////////////////////////////////////////////

SemaphoreHandle_t sensorDataMutex;
QueueHandle_t sensorDataQueue;
TaskHandle_t sensorReadTaskHandle;
TaskHandle_t firebaseTaskHandle;
TaskHandle_t serialPrintTaskHandle;

// NTP Client for time synchronization
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000); // Update every 60 seconds

struct CalibrationData
{
  float calibrationFactor;
  float tareValue;
  bool isValid;
};

struct BME680_Data
{
  float temperature = NAN;
  float humidity = NAN;
  float pressure = NAN;
  float gas_resistance = NAN;
  float altitude = NAN;
  bool isValid = false;
};

struct PMS7003_Data
{
  uint16_t pm1_0 = 0;
  uint16_t pm2_5 = 0;
  uint16_t pm10 = 0;
  uint16_t particles_0_3um = 0;
  uint16_t particles_0_5um = 0;
  uint16_t particles_1_0um = 0;
  uint16_t particles_2_5um = 0;
  uint16_t particles_5_0um = 0;
  uint16_t particles_10_0um = 0;
  bool isValid = false;
  uint8_t errorCode = 0;
};

struct MQ6_Data
{
  float lpg = 0.0;
  float methane = 0.0;
  float butane = 0.0;
  float rawValue = 0.0;
  float voltage = 0.0;
  bool isValid = false;
};

struct MQ7_Data
{
  float co = 0.0;
  float rawValue = 0.0;
  float voltage = 0.0;
  bool isValid = false;
};

struct MICS4514_Data
{
  float co = 0.0;
  float no2 = 0.0;
  float co_voltage = 0.0;
  float no2_voltage = 0.0;
  bool isValid = false;
};

struct HX711_Data
{
  float weight = 0.0;
  bool isValid = false;
};

struct CombinedSensorData
{
  BME680_Data bme680;
  PMS7003_Data pms7003;
  MQ6_Data mq6;
  MQ7_Data mq7;
  MICS4514_Data mics4514;
  HX711_Data hx711;
};

class PMS7003
{
private:
  HardwareSerial &_serial;
  byte _buffer[32];

public:
  PMS7003(HardwareSerial &serial, int rxPin, int txPin) : _serial(serial) {}

  bool read(PMS7003_Data &data)
  {
    if (_serial.available() < 32)
    {
      return false;
    }

    while (_serial.available() >= 32)
    {
      if (_serial.read() == 0x42 && _serial.peek() == 0x4D)
      {
        _serial.read();

        if (_serial.readBytes(_buffer, 30) != 30)
        {
          data.errorCode = 1;
          return false;
        }

        uint16_t checksum = 0x42 + 0x4D;
        for (int i = 0; i < 28; i++)
        {
          checksum += _buffer[i];
        }

        uint16_t receivedChecksum = (_buffer[28] << 8) | _buffer[29];
        if (checksum != receivedChecksum)
        {
          data.errorCode = 2;
          return false;
        }

        data.pm1_0 = (_buffer[4] << 8) | _buffer[5];
        data.pm2_5 = (_buffer[6] << 8) | _buffer[7];
        data.pm10 = (_buffer[8] << 8) | _buffer[9];
        data.particles_0_3um = (_buffer[16] << 8) | _buffer[17];
        data.particles_0_5um = (_buffer[18] << 8) | _buffer[19];
        data.particles_1_0um = (_buffer[20] << 8) | _buffer[21];
        data.particles_2_5um = (_buffer[22] << 8) | _buffer[23];
        data.particles_5_0um = (_buffer[24] << 8) | _buffer[25];
        data.particles_10_0um = (_buffer[26] << 8) | _buffer[27];

        data.isValid = true;
        data.errorCode = 0;
        return true;
      }
    }

    data.errorCode = 3;
    return false;
  }
};

class SensorManager
{
private:
  Adafruit_BME680 _bme;
  HardwareSerial _pmsSerial;
  PMS7003 _pmsSensor;
  HX711_ADC _loadCell;

  CalibrationData calibrationData;

  float _mq6Ro = 10.0;
  float _mq7Ro = 10.0;

  float protectedCurrentWeight = 0.0;
  bool protectedWeightDataReady = false;

  void loadCalibrationFromEEPROM()
  {
    EEPROM.get(CALIBRATION_FACTOR_ADDR, calibrationData.calibrationFactor);
    EEPROM.get(TARE_VALUE_ADDR, calibrationData.tareValue);
    EEPROM.get(CALIBRATION_VALID_ADDR, calibrationData.isValid);

    // Check if calibration data is valid (not NaN or infinity)
    if (isnan(calibrationData.calibrationFactor) || isinf(calibrationData.calibrationFactor) ||
        isnan(calibrationData.tareValue) || isinf(calibrationData.tareValue))
    {
      calibrationData.isValid = false;
    }

    if (calibrationData.isValid)
    {
      Serial.println("Loaded calibration from EEPROM:");
      Serial.println("  Calibration Factor: " + String(calibrationData.calibrationFactor, 6));
      Serial.println("  Tare Value: " + String(calibrationData.tareValue, 6));
    }
    else
    {
      Serial.println("No valid calibration found in EEPROM, using defaults");
      calibrationData.calibrationFactor = HX711_CALIBRATION_FACTOR;
      calibrationData.tareValue = 0.0;
    }
  }

  // Save calibration to EEPROM
  void saveCalibrationToEEPROM()
  {
    EEPROM.put(CALIBRATION_FACTOR_ADDR, calibrationData.calibrationFactor);
    EEPROM.put(TARE_VALUE_ADDR, calibrationData.tareValue);
    EEPROM.put(CALIBRATION_VALID_ADDR, calibrationData.isValid);
    EEPROM.commit();

    Serial.println("Calibration saved to EEPROM:");
    Serial.println("  Calibration Factor: " + String(calibrationData.calibrationFactor, 6));
    Serial.println("  Tare Value: " + String(calibrationData.tareValue, 6));
  }

  // Manual calibration process
  void performHX711Calibration()
  {
    Serial.println("\n=== HX711 MANUAL CALIBRATION ===");
    Serial.println("1. Remove all weight from the load cell");
    Serial.println("2. Press any key when ready to tare...");

    // Wait for user input
    while (!Serial.available())
    {
      delay(100);
    }
    while (Serial.available())
      Serial.read(); // Clear buffer

    // Perform tare with stability check
    Serial.println("Taring... Please wait");
    _loadCell.start(5000, true); // Longer timeout for tare

    if (_loadCell.getTareTimeoutFlag())
    {
      Serial.println("Tare failed! Check connections.");
      return;
    }

    // Wait for readings to stabilize after tare
    Serial.println("Stabilizing after tare...");
    delay(3000);

    // Get stable baseline readings
    float stableBaseline = getStableReading(10, 500);
    Serial.println("Stable baseline: " + String(stableBaseline, 2));

    calibrationData.tareValue = _loadCell.getTareOffset();
    Serial.println("Tare completed. Offset: " + String(calibrationData.tareValue));

    // Calibration with known weight
    Serial.println("\n3. Place a known weight on the load cell");
    Serial.println("4. Enter the weight in grams (e.g., 1000 for 1kg):");

    // Wait for weight input
    while (!Serial.available())
    {
      delay(100);
    }

    String weightStr = Serial.readStringUntil('\n');
    weightStr.trim();
    float knownWeight = weightStr.toFloat();

    if (knownWeight <= 0)
    {
      Serial.println("Invalid weight entered. Calibration cancelled.");
      return;
    }

    Serial.println("Known weight: " + String(knownWeight) + "g");
    Serial.println("Stabilizing readings... Please wait 10 seconds");

    // Longer stabilization time
    delay(10000);

    // Get stable readings with improved method
    _loadCell.setCalFactor(1.0); // Set to 1.0 to get raw counts

    float stableWeightReading = getStableReading(20, 300);
    Serial.println("Stable weight reading: " + String(stableWeightReading, 2));

    // Calculate new calibration factor
    float weightInKg = knownWeight / 1000.0;
    float newCalFactor = stableWeightReading / weightInKg;

    calibrationData.calibrationFactor = newCalFactor;
    calibrationData.isValid = true;

    // Apply new calibration
    _loadCell.setCalFactor(calibrationData.calibrationFactor);

    Serial.println("\nCalibration Results:");
    Serial.println("  Stable reading: " + String(stableWeightReading, 2));
    Serial.println("  Known weight: " + String(knownWeight) + "g (" + String(weightInKg, 3) + "kg)");
    Serial.println("  New calibration factor: " + String(calibrationData.calibrationFactor, 6));

    // Verify calibration with improved stability
    Serial.println("\nVerifying calibration...");
    delay(3000);

    float verificationWeight = getStableReading(10, 500) * 1000.0; // Convert to grams
    float error = abs(verificationWeight - knownWeight);
    float errorPercent = (error / knownWeight) * 100.0;

    Serial.println("  Verification weight: " + String(verificationWeight, 1) + "g");
    Serial.println("  Expected: " + String(knownWeight, 1) + "g");
    Serial.println("  Error: " + String(error, 1) + "g");
    Serial.println("  Error percentage: " + String(errorPercent, 1) + "%");

    if (errorPercent > 5.0)
    {
      Serial.println("  WARNING: High calibration error! Consider recalibrating.");
    }
    else
    {
      Serial.println("  Calibration accuracy is good!");
    }

    // Save to EEPROM
    saveCalibrationToEEPROM();

    Serial.println("\nCalibration completed and saved!");
    Serial.println("Remove the calibration weight and press any key to continue...");

    while (!Serial.available())
    {
      delay(100);
    }
    while (Serial.available())
      Serial.read(); // Clear buffer
  }

  float getStableReading(int numReadings, int delayBetweenReadings)
  {
    float readings[numReadings];
    int validReadings = 0;

    // Collect readings
    for (int i = 0; i < numReadings; i++)
    {
      int attempts = 0;
      while (attempts < 10)
      { // Max 10 attempts per reading
        if (_loadCell.update())
        {
          readings[validReadings] = _loadCell.getData();
          validReadings++;
          break;
        }
        delay(50);
        attempts++;
      }
      if (validReadings < i + 1)
      {
        Serial.println("Warning: Failed to get reading " + String(i + 1));
      }
      delay(delayBetweenReadings);
    }

    if (validReadings < 3)
    {
      Serial.println("Error: Not enough valid readings for stability calculation");
      return 0.0;
    }

    // Sort readings for median calculation
    for (int i = 0; i < validReadings - 1; i++)
    {
      for (int j = 0; j < validReadings - i - 1; j++)
      {
        if (readings[j] > readings[j + 1])
        {
          float temp = readings[j];
          readings[j] = readings[j + 1];
          readings[j + 1] = temp;
        }
      }
    }

    // Calculate statistics
    float median = readings[validReadings / 2];
    float sum = 0;
    int filteredCount = 0;

    // Use readings within 2 standard deviations of median
    for (int i = 0; i < validReadings; i++)
    {
      float deviation = abs(readings[i] - median);
      if (deviation < abs(median) * 0.1)
      { // Within 10% of median
        sum += readings[i];
        filteredCount++;
      }
    }

    if (filteredCount == 0)
    {
      Serial.println("Warning: All readings filtered out, using median");
      return median;
    }

    float average = sum / filteredCount;

    Serial.println("Stability stats: Valid=" + String(validReadings) +
                   ", Filtered=" + String(filteredCount) +
                   ", Range=" + String(readings[validReadings - 1] - readings[0], 2) +
                   ", Average=" + String(average, 2));

    return average;
  }

  void updateProtectedWeight(float weight, bool isReady)
  {
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE)
    {
      protectedCurrentWeight = weight;
      protectedWeightDataReady = isReady;
      xSemaphoreGive(sensorDataMutex);
    }
  }
  HX711_Data getProtectedWeight()
  {
    HX711_Data data;
    if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE)
    {
      data.weight = protectedCurrentWeight;
      data.isValid = protectedWeightDataReady;
      xSemaphoreGive(sensorDataMutex);
    }
    return data;
  }

  void calibrateMQ6()
  {
    float val = 0;
    for (int i = 0; i < 10; i++)
    {
      val += analogRead(MQ6_ANALOG_PIN);
      delay(50);
    }
    val = val / 10.0;
    float voltage = val * (3.3 / 4095.0);
    _mq6Ro = voltage / MQ6_RO_CLEAN_AIR_FACTOR;

    Serial.print("MQ6 calibrated. Ro = ");
    Serial.println(_mq6Ro);
  }

  void calibrateMQ7()
  {
    float val = 0;
    for (int i = 0; i < 10; i++)
    {
      val += analogRead(MQ7_ANALOG_PIN);
      delay(50);
    }
    val = val / 10.0;
    float voltage = val * (3.3 / 4095.0);
    _mq7Ro = voltage / MQ7_RO_CLEAN_AIR_FACTOR;

    Serial.print("MQ7 calibrated. Ro = ");
    Serial.println(_mq7Ro);
  }

  BME680_Data readBME680()
  {
    BME680_Data data;

    if (!_bme.performReading())
    {
      Serial.println("Failed to read BME680!");
      return data;
    }

    data.temperature = _bme.temperature;
    data.humidity = _bme.humidity;
    data.pressure = _bme.pressure / 100.0;
    data.gas_resistance = _bme.gas_resistance / 1000.0;
    data.altitude = _bme.readAltitude(1013.25);
    data.isValid = true;

    return data;
  }

  PMS7003_Data readPMS7003()
  {
    PMS7003_Data data;
    if (!_pmsSensor.read(data))
    {
      Serial.println("PMS7003 read failed. Error code: " + String(data.errorCode));
    }
    return data;
  }

  MQ6_Data readMQ6()
  {
    MQ6_Data data;

    int rawValue = analogRead(MQ6_ANALOG_PIN);
    float voltage = rawValue * (3.3 / 4095.0);
    float rs = (3.3 - voltage) / voltage;
    float ratio = rs / _mq6Ro;

    data.rawValue = rawValue;
    data.voltage = voltage;
    data.lpg = 1000.0 * pow(ratio, -2.5);
    data.methane = 3000.0 * pow(ratio, -2.2);
    data.butane = 1500.0 * pow(ratio, -2.3);
    data.isValid = true; // Always valid

    return data;
  }

  MQ7_Data readMQ7()
  {
    MQ7_Data data;

    int rawValue = analogRead(MQ7_ANALOG_PIN);
    float voltage = rawValue * (3.3 / 4095.0);
    float rs = (3.3 - voltage) / voltage;
    float ratio = rs / _mq7Ro;

    data.rawValue = rawValue;
    data.voltage = voltage;
    data.co = 100.0 * pow(ratio, -1.5);
    data.isValid = true; // Always valid

    return data;
  }

  MICS4514_Data readMICS4514()
  {
    MICS4514_Data data;

    int coRaw = analogRead(MICS4514_CO_PIN);
    int no2Raw = analogRead(MICS4514_NO2_PIN);

    data.co_voltage = coRaw * (3.3 / 4095.0);
    data.no2_voltage = no2Raw * (3.3 / 4095.0);

    data.co = 1.0 / data.co_voltage * 50.0;
    data.no2 = data.no2_voltage * 10.0;
    data.isValid = true;

    // Debug output
    Serial.println("MICS4514 readings:");
    Serial.println("  CO raw: " + String(coRaw) + ", voltage: " + String(data.co_voltage) + ", ppm: " + String(data.co));
    Serial.println("  NO2 raw: " + String(no2Raw) + ", voltage: " + String(data.no2_voltage) + ", ppm: " + String(data.no2));

    return data;
  }

  HX711_Data readHX711()
  {
    HX711_Data data;

    // Get the continuously updated weight with error checking
    if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      data.weight = protectedCurrentWeight;
      data.isValid = protectedWeightDataReady;
      xSemaphoreGive(sensorDataMutex);
    }
    else
    {
      // Mutex timeout, return invalid data
      data.weight = 0.0;
      data.isValid = false;
      Serial.println("Warning: Weight data mutex timeout");
    }

    // Only print occasionally to reduce serial spam
    static unsigned long lastPrint = 0;
    if (data.isValid && (millis() - lastPrint > 1000))
    { // Print every 1 second
      Serial.println("Weight: " + String(data.weight, 1) + " g");
      lastPrint = millis();
    }

    return data;
  }

public:
  SensorManager() : _pmsSerial(2),
                    _pmsSensor(_pmsSerial, PMS_RX_PIN, PMS_TX_PIN),
                    _loadCell(HX711_DOUT_PIN, HX711_SCK_PIN),
                    _bme(BME_CS) {}

  bool begin()
  {
    bool success = true;

    // Initialize EEPROM first
    if (!EEPROM.begin(EEPROM_SIZE))
    {
      Serial.println("Failed to initialize EEPROM");
      success = false;
    }

    // Load calibration data
    loadCalibrationFromEEPROM();

    // Initialize SPI for BME680
    SPI.begin(BME_SCK, BME_MISO, BME_MOSI, BME_CS);
    if (!_bme.begin())
    {
      Serial.println("Could not find BME680 sensor! Check wiring and SPI configuration.");
      success = false;
    }
    else
    {
      _bme.setTemperatureOversampling(BME680_OS_8X);
      _bme.setHumidityOversampling(BME680_OS_2X);
      _bme.setPressureOversampling(BME680_OS_4X);
      _bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
      _bme.setGasHeater(320, 150);
      Serial.println("BME680 initialized successfully via SPI");
    }

    // Initialize PMS7003
    _pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
    Serial.println("PMS7003 serial initialized");

    // Initialize MQ sensors (without calibration)
    pinMode(MQ6_ANALOG_PIN, INPUT);
    pinMode(MQ7_ANALOG_PIN, INPUT);
    Serial.println("MQ sensors initialized (calibration skipped)");

    // Initialize MICS-4514
    pinMode(MICS4514_CO_PIN, INPUT);
    pinMode(MICS4514_NO2_PIN, INPUT);
    Serial.println("MICS-4514 initialized");

    // Initialize HX711
    _loadCell.begin();
    _loadCell.start(2000, false); // false - Do NOT perform tare on startup
    _loadCell.setCalFactor(calibrationData.calibrationFactor);

    // If we have a valid tare value, apply it
    if (calibrationData.isValid && calibrationData.tareValue != 0)
    {
      _loadCell.setTareOffset(calibrationData.tareValue);
    }

    if (_loadCell.getTareTimeoutFlag())
    {
      Serial.println("HX711 initialization timeout! Check wiring and connections!");
      success = false;
    }
    else if (_loadCell.getSignalTimeoutFlag())
    {
      Serial.println("HX711 Signal Timeout! Check wiring and connections!");
      success = false;
    }
    else
    {
      Serial.println("HX711 load cell initialized successfully");
      Serial.println("Current calibration factor: " + String(calibrationData.calibrationFactor, 6));
      if (calibrationData.isValid)
      {
        Serial.println("Using saved calibration from EEPROM");
      }
      else
      {
        Serial.println("Using default calibration - consider manual calibration");
      }
    }

    return success;
  }

  // Check for manual calibration request
  bool checkForCalibrationRequest()
  {
    Serial.println("\n=== CALIBRATION CHECK ===");
    Serial.println("Press 'c' within 5 seconds for manual HX711 calibration...");

    unsigned long startTime = millis();
    while (millis() - startTime < 5000)
    {
      if (Serial.available())
      {
        char input = Serial.read();
        if (input == 'c' || input == 'C')
        {
          while (Serial.available())
            Serial.read(); // Clear buffer
          performHX711Calibration();
          return true;
        }
      }
      delay(100);
    }

    Serial.println("Calibration check timeout - continuing with current settings");
    return false;
  }

  void printCalibrationInfo()
  {
    Serial.println("\n=== CURRENT CALIBRATION INFO ===");
    Serial.println("Calibration Factor: " + String(calibrationData.calibrationFactor, 6));
    Serial.println("Tare Offset: " + String(calibrationData.tareValue, 6));
    Serial.println("Calibration Valid: " + String(calibrationData.isValid ? "Yes" : "No"));
    Serial.println("Current HX711 Factor: " + String(_loadCell.getCalFactor(), 6));
    Serial.println("Current HX711 Tare: " + String(_loadCell.getTareOffset(), 6));
  }

  void continuousWeightUpdate()
  {
    static float lastValidWeight = 0.0;
    static unsigned long lastValidTime = 0;
    static int consecutiveFailures = 0;
    static float weightBuffer[5] = {0}; // Moving average buffer
    static int bufferIndex = 0;
    static bool bufferFilled = false;

    if (_loadCell.update())
    {
      float weightKg = _loadCell.getData();
      float weightGrams = weightKg * 1000.0;

      // Apply reasonable bounds
      if (abs(weightGrams) > 50000)
      { // 50kg limit
        consecutiveFailures++;
        if (consecutiveFailures > 10)
        {
          Serial.println("Too many consecutive extreme readings, possible hardware issue");
        }
        return;
      }

      // Check for reasonable rate of change
      unsigned long currentTime = millis();
      if (lastValidTime > 0 && (currentTime - lastValidTime) < 100)
      { // Less than 100ms
        float rateOfChange = abs(weightGrams - lastValidWeight) / ((currentTime - lastValidTime) / 1000.0);
        if (rateOfChange > 10000)
        { // More than 10kg/second is suspicious
          Serial.println("Extreme rate of change detected: " + String(rateOfChange) + " g/s");
          consecutiveFailures++;
          return;
        }
      }

      // Add to moving average buffer
      weightBuffer[bufferIndex] = weightGrams;
      bufferIndex = (bufferIndex + 1) % 5;
      if (bufferIndex == 0)
        bufferFilled = true;

      // Calculate moving average
      float sum = 0;
      int count = bufferFilled ? 5 : bufferIndex;
      for (int i = 0; i < count; i++)
      {
        sum += weightBuffer[i];
      }
      float smoothedWeight = sum / count;

      // Reset failure counter on successful reading
      consecutiveFailures = 0;
      lastValidWeight = smoothedWeight;
      lastValidTime = currentTime;

      updateProtectedWeight(smoothedWeight, true);
    }
    else
    {
      consecutiveFailures++;
      if (consecutiveFailures > 100)
      {
        Serial.println("HX711 communication lost - too many failed updates");
        updateProtectedWeight(0.0, false);
      }
    }
  }

  void readAllSensors(CombinedSensorData &data)
  {
    data.bme680 = readBME680();
    data.pms7003 = readPMS7003();
    data.mq6 = readMQ6();
    data.mq7 = readMQ7();
    data.mics4514 = readMICS4514();
    data.hx711 = getProtectedWeight(); // Get thread-safe weight data
  }

  HX711_Data getCurrentWeight()
  {
    return getProtectedWeight();
  }
};

SensorManager sensorManager;

void processData(AsyncResult &aResult)
{
  if (aResult.isError())
  {
    Serial.print("Error in async task: ");
    Serial.println(aResult.error().message());
    return;
  }

  Serial.print("Async task completed: ");
  Serial.println(aResult.uid());
}

String getFormattedTime()
{
  timeClient.update();
  time_t epochTime = timeClient.getEpochTime();
  struct tm *ptm = gmtime(&epochTime);

  char buffer[20];
  // Format as ISO 8601: YYYY-MM-DDThh:mm:ssZ
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", ptm);
  return String(buffer);
}

// FreeRTOS Task Functions
void sensorReadTask(void *parameter)
{
  SensorManager *sensorMgr = (SensorManager *)parameter;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10); // Read every 10ms for responsive weight readings

  for (;;)
  {
    // Continuously update weight reading
    sensorMgr->continuousWeightUpdate();

    // Send current sensor data to queue every 100ms for console printing
    static uint8_t printCounter = 0;
    if (++printCounter >= 10)
    { // Every 100ms (10 * 10ms)
      printCounter = 0;

      CombinedSensorData currentData;
      sensorMgr->readAllSensors(currentData);

      // Send to queue for console printing (non-blocking)
      xQueueSend(sensorDataQueue, &currentData, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void pushToDatabase(const String &path, JsonDocument &jsonData, const String &taskName = "DB_Push")
{
  if (!app.ready())
  {
    Serial.println("Firebase not ready. Cannot push data to " + path);
    return;
  }

  // Sanitize JSON document
  JsonObject root = jsonData.as<JsonObject>();
  for (JsonPair kv : root)
  {
    if (kv.value().is<float>())
    {
      float val = kv.value().as<float>();
      if (isnan(val) || isinf(val) || val > 1e10)
      {
        Serial.println("Sanitizing invalid float for key " + String(kv.key().c_str()) + ": " + String(val));
        root[kv.key()] = 999999.0;
      }
    }
    else if (kv.value().is<JsonObject>())
    {
      JsonObject nested = kv.value().as<JsonObject>();
      for (JsonPair nestedKv : nested)
      {
        if (nestedKv.value().is<float>())
        {
          float val = nestedKv.value().as<float>();
          if (isnan(val) || isinf(val) || val > 1e10)
          {
            Serial.println("Sanitizing invalid nested float for key " + String(nestedKv.key().c_str()) + ": " + String(val));
            nested[nestedKv.key()] = 999999.0;
          }
        }
      }
    }
  }

  String timestamp = getFormattedTime();
  String dataPath = path + "/" + timestamp;

  String jsonString;
  serializeJson(jsonData, jsonString);

  Serial.println("Pushing to path: " + dataPath);
  Serial.println("JSON data: " + jsonString);

  // Push the entire object at once instead of field by field
  Database.set<object_t>(aClient, dataPath, object_t(jsonString), processData, taskName);

  Serial.println("Pushed complete object for " + taskName);
}

void firebaseTask(void *parameter)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(30000); // 30 seconds

  for (;;)
  {
    if (app.ready())
    {
      CombinedSensorData data;
      sensorManager.readAllSensors(data);

      // BME680
      {
        StaticJsonDocument<128> bmeData;
        bmeData["temperature"] = data.bme680.temperature;
        bmeData["humidity"] = data.bme680.humidity;
        bmeData["pressure"] = data.bme680.pressure;
        bmeData["gas_resistance"] = data.bme680.gas_resistance;
        bmeData["altitude"] = data.bme680.altitude;
        if (isDB)
        {
          pushToDatabase("/sensor_data/bme680", bmeData, "BME680_PUSH");
        }
      }

      // PMS7003
      {
        StaticJsonDocument<256> pmsData;
        pmsData["pm1_0"] = data.pms7003.pm1_0;
        pmsData["pm2_5"] = data.pms7003.pm2_5;
        pmsData["pm10"] = data.pms7003.pm10;
        pmsData["particles_0_3um"] = data.pms7003.particles_0_3um;
        pmsData["particles_0_5um"] = data.pms7003.particles_0_5um;
        pmsData["particles_1_0um"] = data.pms7003.particles_1_0um;
        pmsData["particles_2_5um"] = data.pms7003.particles_2_5um;
        pmsData["particles_5_0um"] = data.pms7003.particles_5_0um;
        pmsData["particles_10_0um"] = data.pms7003.particles_10_0um;
        if (isDB)
        {
          pushToDatabase("/sensor_data/pms7003", pmsData, "PMS7003_PUSH");
        }
      }

      // MQ6
      {
        StaticJsonDocument<128> mq6Data;
        mq6Data["lpg"] = data.mq6.lpg;
        mq6Data["methane"] = data.mq6.methane;
        mq6Data["butane"] = data.mq6.butane;
        mq6Data["voltage"] = data.mq6.voltage;
        if (isDB)
        {
          pushToDatabase("/sensor_data/mq6", mq6Data, "MQ6_PUSH");
        }
      }

      // MQ7
      {
        StaticJsonDocument<128> mq7Data;
        float co_value = data.mq7.co;
        if (co_value > 1000000)
          co_value = 1000000;
        mq7Data["co"] = co_value;
        mq7Data["voltage"] = data.mq7.voltage;
        if (isDB)
        {
          pushToDatabase("/sensor_data/mq7", mq7Data, "MQ7_PUSH");
        }
      }

      // MICS4514
      {
        StaticJsonDocument<128> micsData;
        micsData["co"] = data.mics4514.co;
        micsData["no2"] = data.mics4514.no2;
        micsData["co_voltage"] = data.mics4514.co_voltage;
        micsData["no2_voltage"] = data.mics4514.no2_voltage;
        if (isDB)
        {
          pushToDatabase("/sensor_data/mics4514", micsData, "MICS4514_PUSH");
        }
      }

      // HX711
      {
        StaticJsonDocument<64> hx711Data;
        hx711Data["weight"] = data.hx711.weight;
        if (isDB)
        {
          pushToDatabase("/sensor_data/hx711", hx711Data, "HX711_PUSH");
        }
      }

      // Summary
      StaticJsonDocument<256> summaryData;
      JsonObject bmeObj = summaryData.createNestedObject("bme");
      bmeObj["temp"] = data.bme680.temperature;
      bmeObj["hum"] = data.bme680.humidity;
      JsonObject pmsObj = summaryData.createNestedObject("pms");
      pmsObj["pm25"] = data.pms7003.pm2_5;
      pmsObj["pm10"] = data.pms7003.pm10;
      summaryData["lpg"] = data.mq6.lpg;
      summaryData["weight"] = data.hx711.weight;
      if (isDB)
      {
        pushToDatabase("/sensor_summary", summaryData, "SUMMARY_PUSH");
      }

      Serial.println("=== Firebase Push Complete ===");
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void serialPrintTask(void *parameter)
{
  CombinedSensorData receivedData;

  for (;;)
  {
    // Wait for sensor data from queue
    if (xQueueReceive(sensorDataQueue, &receivedData, portMAX_DELAY) == pdTRUE)
    {
      // Print sensor readings to console
      Serial.println("========== SENSOR READINGS ==========");

      if (receivedData.bme680.isValid)
      {
        Serial.printf("BME680 - Temp: %.2f°C, Humidity: %.2f%%, Pressure: %.2f hPa\n",
                      receivedData.bme680.temperature, receivedData.bme680.humidity, receivedData.bme680.pressure);
        Serial.printf("BME680 - Gas: %.2f kOhm, Altitude: %.2f m\n",
                      receivedData.bme680.gas_resistance, receivedData.bme680.altitude);
      }

      if (receivedData.pms7003.isValid)
      {
        Serial.printf("PMS7003 - PM1.0: %d, PM2.5: %d, PM10: %d μg/m³\n",
                      receivedData.pms7003.pm1_0, receivedData.pms7003.pm2_5, receivedData.pms7003.pm10);
      }

      if (receivedData.mq6.isValid)
      {
        Serial.printf("MQ6 - LPG: %.2f ppm, Methane: %.2f ppm, Butane: %.2f ppm\n",
                      receivedData.mq6.lpg, receivedData.mq6.methane, receivedData.mq6.butane);
      }

      if (receivedData.mq7.isValid)
      {
        Serial.printf("MQ7 - CO: %.2f ppm\n", receivedData.mq7.co);
      }

      if (receivedData.mics4514.isValid)
      {
        Serial.printf("MICS4514 - CO: %.2f ppm, NO2: %.2f ppm\n",
                      receivedData.mics4514.co, receivedData.mics4514.no2);
      }

      if (receivedData.hx711.isValid)
      {
        Serial.printf("HX711 - Weight: %.0f g\n", receivedData.hx711.weight);
      }

      Serial.println("=====================================");
    }
  }
}

String getWifiNetworkList()
{
  Serial.println("Scanning for WiFi networks...");

  // Scan in AP+STA mode (no need to switch modes)
  WiFi.mode(WIFI_AP_STA);

  // Scan for networks
  int n = WiFi.scanNetworks();
  Serial.println("Scan completed");

  // Return to AP mode after scanning
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  String networkList = "";

  if (n == 0)
  {
    networkList = "<option value=''>No networks found</option>";
  }
  else
  {
    // Create array to store networks (to avoid duplicates)
    String networks[MAX_NETWORKS];
    int networksCount = 0;

    for (int i = 0; i < n && networksCount < MAX_NETWORKS; i++)
    {
      // Check if network is already in our list (avoid duplicates)
      bool isDuplicate = false;
      for (int j = 0; j < networksCount; j++)
      {
        if (networks[j] == WiFi.SSID(i))
        {
          isDuplicate = true;
          break;
        }
      }

      if (!isDuplicate && WiFi.SSID(i).length() > 0)
      {
        // Add to our array
        networks[networksCount] = WiFi.SSID(i);
        networksCount++;

        // Add to option list with signal strength
        networkList += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i);
        networkList += " (";

        // Signal strength icon based on RSSI
        int rssi = WiFi.RSSI(i);
        if (rssi > -50)
        {
          networkList += "Strong";
        }
        else if (rssi > -70)
        {
          networkList += "Good";
        }
        else
        {
          networkList += "Weak";
        }

        // Add encryption type
        networkList += ", ";
        switch (WiFi.encryptionType(i))
        {
        case WIFI_AUTH_OPEN:
          networkList += "Open";
          break;
        case WIFI_AUTH_WEP:
          networkList += "WEP";
          break;
        case WIFI_AUTH_WPA_PSK:
          networkList += "WPA";
          break;
        case WIFI_AUTH_WPA2_PSK:
          networkList += "WPA2";
          break;
        case WIFI_AUTH_WPA_WPA2_PSK:
          networkList += "WPA/WPA2";
          break;
        default:
          networkList += "Unknown";
        }

        networkList += ")</option>";
      }
    }
  }

  return networkList;
}

bool loadWiFiCredentials(String &ssid, String &password)
{
  if (SPIFFS.exists(WIFI_FILE))
  {
    File file = SPIFFS.open(WIFI_FILE, "r");
    if (file)
    {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, file);
      file.close();

      if (!error)
      {
        ssid = doc["ssid"].as<String>();
        password = doc["password"].as<String>();
        return true;
      }
    }
  }
  return false;
}

void saveWiFiCredentials(const String &ssid, const String &password)
{
  File file = SPIFFS.open(WIFI_FILE, "w");
  if (file)
  {
    StaticJsonDocument<256> doc;
    doc["ssid"] = ssid;
    doc["password"] = password;
    serializeJson(doc, file);
    file.close();
    Serial.println("WiFi credentials saved");
  }
  else
  {
    Serial.println("Failed to open file for writing");
  }
}

void setupServer()
{
  server.onNotFound([]()
                    {
    server.sendHeader("Location", "http://192.168.4.1/setup", true);
    server.send(302, "text/plain", ""); });

  server.on("/setup", HTTP_GET, []()
            {
    String networkList = getWifiNetworkList();

    String html = "<!DOCTYPE html>"
                  "<html><head><title>ESP32 WiFi Setup</title>"
                  "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                  "<style>"
                  "body{font-family:Arial,sans-serif;margin:0;padding:20px;text-align:center;background-color:#f5f5f5;}"
                  "h1{color:#0066cc;margin-bottom:20px;}"
                  ".container{max-width:400px;margin:0 auto;background-color:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}"
                  "form{text-align:left;}"
                  "label{display:block;margin:10px 0 5px;font-weight:bold;}"
                  "select,input{width:100%;padding:10px;box-sizing:border-box;margin-bottom:15px;border:1px solid #ddd;border-radius:4px;}"
                  "button{background-color:#0066cc;color:white;border:none;padding:12px 20px;cursor:pointer;border-radius:4px;width:100%;font-size:16px;}"
                  "button:hover{background-color:#0055bb;}"
                  ".status{margin-top:20px;padding:10px;border-radius:4px;}"
                  ".loader{border:5px solid #f3f3f3;border-top:5px solid #3498db;border-radius:50%;width:30px;height:30px;animation:spin 2s linear infinite;margin:10px auto;display:none;}"
                  "@keyframes spin{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}}"
                  ".refresh-btn{background-color:#4CAF50;margin-bottom:15px;}"
                  "</style></head>"
                  "<body>"
                  "<div class='container'>"
                  "<h1>ESP32 WiFi Setup</h1>"
                  "<button class='refresh-btn' onclick='location.reload()'>Refresh WiFi List</button>"
                  "<form method='POST' action='/connect'>"
                  "<label for='ssid'>Select WiFi Network:</label>"
                  "<select id='ssid' name='ssid' required>" +
                  networkList +
                  "</select>"
                  "<label for='password'>WiFi Password:</label>"
                  "<input type='password' id='password' name='password'>"
                  "<button type='submit'>Connect</button>"
                  "</form>"
                  "<div class='loader' id='loader'></div>"
                  "<div class='status' id='status'></div>"
                  "</div>"
                  "</body></html>";

    server.send(200, "text/html", html); });

  server.begin();
}

void setup()
{
  // Basic initialization
  Serial.begin(115200);
  delay(2000);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize SPIFFS
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS Mount Failed");
    blinkError(5);
    ESP.restart();
  }

  // Try to load saved WiFi credentials first
  String ssid, password;
  if (loadWiFiCredentials(ssid, password))
  {
    Serial.println("Found saved WiFi credentials, attempting to connect...");
    if (connectToWiFi(ssid, password))
    {
      wifiConnected = true;
      Serial.println("Successfully connected to WiFi");
      digitalWrite(LED_PIN, HIGH); // Solid LED indicates connected
    }
    else
    {
      Serial.println("Failed to connect with saved credentials");
    }
  }

  // If not connected, start configuration AP
  if (!wifiConnected)
  {
    Serial.println("Starting configuration AP");
    setupWiFiAP();
    setupWebServer();
    lastConnectionAttempt = millis();

    // Wait for WiFi configuration
    while (!wifiConnected)
    {
      server.handleClient();
      dnsServer.processNextRequest();

      // Blink LED to indicate waiting for configuration
      digitalWrite(LED_PIN, (millis() / 500) % 2);

      // Timeout after 5 minutes and reboot
      if (millis() - lastConnectionAttempt > 300000)
      {
        Serial.println("WiFi configuration timeout, rebooting...");
        ESP.restart();
      }

      delay(10);
    }
  }

  // Only proceed with sensor initialization if WiFi is connected
  if (wifiConnected)
  {
    initializeSensors();
    initializeFirebase();
    startTasks();
    Serial.println("System fully initialized");
  }
  else
  {
    Serial.println("System in limited operation mode (WiFi not connected)");
  }
}

void loop()
{
  if (wifiConnected)
  {
    // Handle OTA updates
    ElegantOTA.loop();

    // Handle Firebase operations if needed
    app.loop();
  }

  delay(1000);
}

void setupWiFiAP()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Start DNS server for captive portal
  dnsServer.start(DNS_PORT, "*", apIP);
}

void initializeSensors()
{
  Serial.println("Initializing sensors...");
  if (!sensorManager.begin())
  {
    Serial.println("Sensor initialization failed!");
    blinkError(3);
  }
  sensorManager.printCalibrationInfo();
}

void initializeFirebase()
{
  if (!wifiConnected)
    return;

  Serial.println("Initializing Firebase...");
  ssl_client.setInsecure();
  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");

  unsigned long authStart = millis();
  while (!app.ready() && millis() - authStart < 10000)
  {
    delay(100);
  }

  if (app.ready())
  {
    app.getApp<RealtimeDatabase>(Database);
    Database.url(DATABASE_URL);
    isDB = true;
    Serial.println("Firebase ready");
  }
  else
  {
    Serial.println("Firebase init failed");
    isDB = false;
  }
}

void startTasks()
{
  Serial.println("Starting FreeRTOS tasks...");
  xTaskCreatePinnedToCore(sensorReadTask, "SensorRead", 4096, &sensorManager, 2, &sensorReadTaskHandle, 0);
  xTaskCreatePinnedToCore(firebaseTask, "FirebaseTask", 8192, NULL, 2, &firebaseTaskHandle, 1);
  xTaskCreatePinnedToCore(serialPrintTask, "SerialPrint", 3072, NULL, 1, &serialPrintTaskHandle, 0);
}

void setupWebServer()
{
  server.on("/", HTTP_GET, handleRoot);
  server.on("/connect", HTTP_POST, handleConnect);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void handleRoot()
{
  String networkList = getWifiNetworkList();

  String html = "<!DOCTYPE html>"
                "<html><head><title>ESP32 WiFi Setup</title>"
                "<meta name='viewport' content='width=device-width, initial-scale=1'>"
                "<style>"
                "body{font-family:Arial,sans-serif;margin:0;padding:20px;text-align:center;background-color:#f5f5f5;}"
                "h1{color:#0066cc;margin-bottom:20px;}"
                ".container{max-width:400px;margin:0 auto;background-color:white;padding:20px;border-radius:10px;box-shadow:0 2px 10px rgba(0,0,0,0.1);}"
                "form{text-align:left;}"
                "label{display:block;margin:10px 0 5px;font-weight:bold;}"
                "select,input{width:100%;padding:10px;box-sizing:border-box;margin-bottom:15px;border:1px solid #ddd;border-radius:4px;}"
                "button{background-color:#0066cc;color:white;border:none;padding:12px 20px;cursor:pointer;border-radius:4px;width:100%;font-size:16px;}"
                "button:hover{background-color:#0055bb;}"
                ".status{margin-top:20px;padding:10px;border-radius:4px;}"
                ".loader{border:5px solid #f3f3f3;border-top:5px solid #3498db;border-radius:50%;width:30px;height:30px;animation:spin 2s linear infinite;margin:10px auto;display:none;}"
                "@keyframes spin{0%{transform:rotate(0deg);}100%{transform:rotate(360deg);}"
                ".refresh-btn{background-color:#4CAF50;margin-bottom:15px;}"
                "</style></head>"
                "<body>"
                "<div class='container'>"
                "<h1>ESP32 WiFi Setup</h1>"
                "<button class='refresh-btn' onclick='location.reload()'>Refresh WiFi List</button>"
                "<form method='POST' action='/connect'>"
                "<label for='ssid'>Select WiFi Network:</label>"
                "<select id='ssid' name='ssid' required>" +
                networkList +
                "</select>"
                "<label for='password'>WiFi Password:</label>"
                "<input type='password' id='password' name='password'>"
                "<button type='submit'>Connect</button>"
                "</form>"
                "<div class='loader' id='loader'></div>"
                "<div class='status' id='status'></div>"
                "</div>"
                "<script>"
                "document.querySelector('form').addEventListener('submit', function(e) {"
                "  document.getElementById('loader').style.display = 'block';"
                "  document.getElementById('status').innerHTML = 'Connecting...';"
                "});"
                "</script>"
                "</body></html>";

  server.send(200, "text/html", html);
}

void handleConnect()
{
  if (server.hasArg("ssid"))
  {
    String ssid = server.arg("ssid");
    String password = server.hasArg("password") ? server.arg("password") : "";

    server.send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='10;url=/'></head><body>"
                                  "<h1>Connecting to WiFi...</h1><p>Attempting to connect to " +
                                      ssid + "</p></body></html>");

    Serial.println("Attempting to connect to: " + ssid);
    if (connectToWiFi(ssid, password))
    {
      saveWiFiCredentials(ssid, password);
      wifiConnected = true;
      Serial.println("WiFi connected successfully");
      digitalWrite(LED_PIN, HIGH);
      // Give some time for the response to be sent
      delay(1000);
      // Restart to initialize all services with WiFi
      ESP.restart();
    }
    else
    {
      server.send(200, "text/html", "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='5;url=/'></head><body>"
                                    "<h1>Connection Failed</h1><p>Could not connect to " +
                                        ssid + "</p></body></html>");
      Serial.println("Failed to connect to WiFi");
    }
  }
  else
  {
    server.send(400, "text/plain", "Missing SSID");
  }
}

void handleNotFound()
{
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

bool connectToWiFi(const String &ssid, const String &password)
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000)
  {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink while connecting
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nConnected to WiFi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    return true;
  }

  Serial.println("\nFailed to connect to WiFi");
  return false;
}

void blinkError(int count)
{
  for (int i = 0; i < count; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}