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

#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// WiFi credentials
// #define WIFI_SSID "SAMURAI@CREEDS_2.4G"
// #define WIFI_PASSWORD "samurai1@creeds"

// Firebase credentials
#define Web_API_KEY "AIzaSyAaiE2hCxCOIzy39Gq_BW8KlD_bUSR6Tyw"
#define DATABASE_URL "https://esp-gas-ai-default-rtdb.asia-southeast1.firebasedatabase.app"
#define USER_EMAIL "satoru.thilina@gmail.com"
#define USER_PASS "123456"

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

bool isDB = false;

// Firebase components
UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
// using AsyncClient = AsyncClientClass;
// AsyncClient aClient(ssl_client);
AsyncClientClass aClient;
RealtimeDatabase Database;

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

// Sensor data structures
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

// Combined sensor data structure
struct CombinedSensorData
{
  BME680_Data bme680;
  PMS7003_Data pms7003;
  MQ6_Data mq6;
  MQ7_Data mq7;
  MICS4514_Data mics4514;
  HX711_Data hx711;
};

////////////////////////////////////////////////////////////////////////////////////////////
class WiFiSetupManager
{
private:
  AsyncWebServer *server;
  DNSServer dnsServer;
  const char *apSSID = "ESP32_Setup";
  const char *apPassword = "12345678";
  const char *wifiFile = "/wifi.json";
  IPAddress apIP = IPAddress(192, 168, 4, 1);
  bool setupMode = false;

  bool loadCredentials(String &ssid, String &password);
  void saveCredentials(const String &ssid, const String &password);
  void startAP();
  void setupWebServer();
  String scanNetworks();

public:
  WiFiSetupManager();
  ~WiFiSetupManager();
  bool begin();        // Returns true if connected to WiFi, false if in setup mode
  void handleClient(); // Call in loop when in setup mode
  bool isInSetupMode() { return setupMode; }
};

// WiFiSetupManager.cpp
WiFiSetupManager::WiFiSetupManager() : server(nullptr) {}

WiFiSetupManager::~WiFiSetupManager()
{
  if (server)
    delete server;
}

bool WiFiSetupManager::begin()
{
  if (!SPIFFS.begin(true))
  {
    Serial.println("SPIFFS failed");
    return false;
  }

  String ssid, password;
  if (loadCredentials(ssid, password))
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), password.c_str());

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000)
    {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());
      return true;
    }
  }

  // Start setup mode
  setupMode = true;
  startAP();
  setupWebServer();
  Serial.println("WiFi Setup Mode - Connect to: " + String(apSSID));
  return false;
}

void WiFiSetupManager::handleClient()
{
  if (setupMode)
  {
    dnsServer.processNextRequest();
  }
}

bool WiFiSetupManager::loadCredentials(String &ssid, String &password)
{
  if (!SPIFFS.exists(wifiFile))
    return false;

  File file = SPIFFS.open(wifiFile, "r");
  if (!file)
    return false;

  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, file) == DeserializationError::Ok)
  {
    ssid = doc["ssid"].as<String>();
    password = doc["password"].as<String>();
    file.close();
    return ssid.length() > 0;
  }
  file.close();
  return false;
}

void WiFiSetupManager::saveCredentials(const String &ssid, const String &password)
{
  File file = SPIFFS.open(wifiFile, "w");
  if (file)
  {
    StaticJsonDocument<256> doc;
    doc["ssid"] = ssid;
    doc["password"] = password;
    serializeJson(doc, file);
    file.close();
  }
}

void WiFiSetupManager::startAP()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID, apPassword);
  dnsServer.start(53, "*", apIP);
}

String WiFiSetupManager::scanNetworks()
{
  WiFi.mode(WIFI_AP_STA);
  int n = WiFi.scanNetworks();
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSSID, apPassword);

  String options = "";
  for (int i = 0; i < n; i++)
  {
    if (WiFi.SSID(i).length() > 0)
    {
      options += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + "</option>";
    }
  }
  return options.length() > 0 ? options : "<option>No networks found</option>";
}

void WiFiSetupManager::setupWebServer()
{
  server = new AsyncWebServer(80);

  server->onNotFound([](AsyncWebServerRequest *request)
                     { request->redirect("http://192.168.4.1/setup"); });

  server->on("/setup", HTTP_GET, [this](AsyncWebServerRequest *request)
             {
          String html = "<!DOCTYPE html><html><head><title>WiFi Setup</title>"
                       "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                       "<style>body{font-family:Arial;text-align:center;padding:20px;background:#f5f5f5;}"
                       ".container{max-width:400px;margin:0 auto;background:white;padding:20px;border-radius:10px;}"
                       "input,select,button{width:100%;padding:10px;margin:10px 0;border:1px solid #ddd;border-radius:5px;}"
                       "button{background:#007bff;color:white;border:none;cursor:pointer;}"
                       "button:hover{background:#0056b3;}</style></head>"
                       "<body><div class='container'><h2>WiFi Setup</h2>"
                       "<form id='f'><select id='ssid' required>" + scanNetworks() + "</select>"
                       "<input type='password' id='pwd' placeholder='Password'>"
                       "<button type='submit'>Connect</button></form>"
                       "<div id='msg'></div></div>"
                       "<script>document.getElementById('f').onsubmit=function(e){"
                       "e.preventDefault();var s=document.getElementById('ssid').value;"
                       "var p=document.getElementById('pwd').value;"
                       "fetch('/connect',{method:'POST',headers:{'Content-Type':'application/json'},"
                       "body:JSON.stringify({ssid:s,password:p})}).then(r=>r.json()).then(d=>{"
                       "document.getElementById('msg').innerHTML=d.message;"
                       "if(d.success)setTimeout(()=>location.reload(),2000);});};</script>"
                       "</body></html>";
          request->send(200, "text/html", html); });

  server->on("/connect", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
             {
          StaticJsonDocument<256> doc;
          deserializeJson(doc, data, len);
          
          String ssid = doc["ssid"];
          String password = doc["password"];
          
          AsyncResponseStream *response = request->beginResponseStream("application/json");
          StaticJsonDocument<128> resp;
          
          if (ssid.length() > 0) {
              saveCredentials(ssid, password);
              resp["success"] = true;
              resp["message"] = "Saved! Restarting...";
          } else {
              resp["success"] = false;
              resp["message"] = "SSID required";
          }
          
          serializeJson(resp, *response);
          request->send(response);
          
          if (resp["success"]) {
              delay(1000);
              ESP.restart();
          } });

  server->begin();
}

////////////////////////////////////////////////////////////////////////////////////////////
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

WiFiSetupManager wifiManager;
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

void pushToDatabase(const String &dataPath, ArduinoJson::V741PB22::JsonDocument &jsonDoc, const String &taskName)
{
  if (!app.ready())
  {
    Serial.println("Firebase not ready. Cannot push data to " + dataPath);
    return;
  }

  // Sanitize JSON document
  JsonObject root = jsonDoc.as<JsonObject>();
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
  String fullDataPath = dataPath + "/" + timestamp;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println("Pushing to path: " + fullDataPath);
  Serial.println("JSON data: " + jsonString);

  // Push the entire object at once instead of field by field
  Database.set<object_t>(aClient, fullDataPath, object_t(jsonString), processData, "");

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

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting ESP32 Gas Sensor System with FreeRTOS");

  // WiFi Setup - handles connection or captive portal setup
  if (!wifiManager.begin())
  {
    Serial.println("WiFi setup mode active - connect to ESP32_Setup network");
    while (wifiManager.isInSetupMode())
    {
      wifiManager.handleClient();
      delay(100);
    }
  }

  Serial.println("WiFi Connected - Starting sensor initialization...");
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());

  // Create FreeRTOS objects
  sensorDataMutex = xSemaphoreCreateMutex();
  sensorDataQueue = xQueueCreate(5, sizeof(CombinedSensorData));

  if (sensorDataMutex == NULL || sensorDataQueue == NULL)
  {
    Serial.println("Failed to create FreeRTOS objects!");
    while (1)
      ;
  }

  if (!sensorManager.begin())
  {
    Serial.println("Failed to initialize some sensors!");
  }

  // Check for manual calibration request
  sensorManager.checkForCalibrationRequest();

  // Print current calibration info
  sensorManager.printCalibrationInfo();

  // Initialize NTP client
  timeClient.begin();
  Serial.println("Connecting to NTP server...");
  while (!timeClient.update())
  {
    Serial.print(".");
    timeClient.forceUpdate();
    delay(500);
  }
  Serial.println("\nNTP time synchronized");

  // Configure SSL client for AsyncClientClass
  // aClient.setInsecure();
  // aClient.setConnectionTimeout(5000);
  // aClient.setHandshakeTimeout(3000);

  // Initialize Firebase app with corrected function call
  initializeApp(aClient, app, getAuth(user_auth), processData, "");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);

  Serial.println("Firebase initialization complete");

  Serial.println("Warming up sensors...");
  delay(20000);
  Serial.println("Sensors ready");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(
      sensorReadTask,        // Task function
      "SensorRead",          // Task name
      8192,                  // Stack size
      &sensorManager,        // Parameter
      2,                     // Priority (higher for sensor reading)
      &sensorReadTaskHandle, // Task handle
      0                      // Core 0
  );

  xTaskCreatePinnedToCore(
      firebaseTask,        // Task function
      "FirebaseTask",      // Task name
      16384,               // Stack size (larger for network operations)
      NULL,                // Parameter
      1,                   // Priority (lower than sensor reading)
      &firebaseTaskHandle, // Task handle
      1                    // Core 1
  );

  xTaskCreatePinnedToCore(
      serialPrintTask,        // Task function
      "SerialPrint",          // Task name
      4096,                   // Stack size
      NULL,                   // Parameter
      1,                      // Priority
      &serialPrintTaskHandle, // Task handle
      0                       // Core 0
  );

  if (sensorDataMutex == NULL || sensorDataQueue == NULL ||
      sensorReadTaskHandle == NULL || firebaseTaskHandle == NULL ||
      serialPrintTaskHandle == NULL)
  {
    Serial.println("Failed to create FreeRTOS objects!");
    while (1)
      ;
  }

  Serial.println("FreeRTOS tasks created successfully");
  Serial.println("System ready! Send 't' for manual tare, 'i' for calibration info");
  Serial.printf("Free heap: %d, Min free heap: %d\n",
                ESP.getFreeHeap(), ESP.getMinFreeHeap());
  delay(1000);
}

void loop()
{
  // Keep Firebase app loop running on main thread
  app.loop();

  // Small delay to prevent watchdog issues
  delay(1000);
}