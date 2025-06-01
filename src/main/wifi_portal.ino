#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <DHT.h>

// WiFi Manager Configuration
const char* AP_SSID = "ESP32_Setup";
const char* AP_PASSWORD = "12345678";
const char* WIFI_FILE = "/wifi_credentials.json";
const int LED_PIN = 2; // Onboard LED
const int MAX_NETWORKS = 20; // Maximum number of networks to scan

// Captive Portal
const byte DNS_PORT = 53;
DNSServer dnsServer;
IPAddress apIP(192, 168, 4, 1);

// DHT Sensor Configuration (example sensor)
#define DHTPIN 4       // Pin connected to DHT sensor
#define DHTTYPE DHT22  // DHT22 or DHT11
DHT dht(DHTPIN, DHTTYPE);

// Web Server
AsyncWebServer server(80);

// WiFi connection status
bool wifiConnected = false;

// Function declarations
bool loadWiFiCredentials(String &ssid, String &password);
void saveWiFiCredentials(const String &ssid, const String &password);
void setupWiFiAP();
void setupServer();
bool connectToWiFi(const String &ssid, const String &password);
void readSensorData();
String getWifiNetworkList();

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }
  
  // Initialize DHT sensor
  dht.begin();
  
  // Try to load WiFi credentials
  String ssid, password;
  if (loadWiFiCredentials(ssid, password)) { 
    // Try to connect to WiFi with saved credentials
    if (connectToWiFi(ssid, password)) {
      wifiConnected = true;
      Serial.println("Connected to WiFi");
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("Failed to connect with saved credentials");
      setupWiFiAP();
    }
  } else {
    Serial.println("No saved WiFi credentials found");
    setupWiFiAP();
  }
  
  // Setup web server
  setupServer();
}


void loop() {
  // Handle DNS requests when in AP mode
  if (!wifiConnected) {
    dnsServer.processNextRequest();
  }
  
  // Blink LED to indicate status
  if (wifiConnected) {
    digitalWrite(LED_PIN, HIGH);
    delay(5000);  // Read sensor every 5 seconds when connected
    readSensorData();
  } else {
    // Blinking pattern when in AP mode
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

String getWifiNetworkList() {
  Serial.println("Scanning for WiFi networks...");
  
  // Scan in AP+STA mode (no need to switch modes)
  WiFi.mode(WIFI_AP_STA);
  
  // Scan for networks
  int n = WiFi.scanNetworks();
  Serial.println("Scan completed");
  
  // Return to AP mode after scanning (but keep STA enabled if we're connected)
  if (wifiConnected) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
    WiFi.softAP(AP_SSID, AP_PASSWORD);
  }
  
  String networkList = "";
  
  if (n == 0) {
    networkList = "<option value=''>No networks found</option>";
  } else {
    // Create array to store networks (to avoid duplicates)
    String networks[MAX_NETWORKS];
    int networksCount = 0;
    
    for (int i = 0; i < n && networksCount < MAX_NETWORKS; i++) {
      // Check if network is already in our list (avoid duplicates)
      bool isDuplicate = false;
      for (int j = 0; j < networksCount; j++) {
        if (networks[j] == WiFi.SSID(i)) {
          isDuplicate = true;
          break;
        }
      }
      
      if (!isDuplicate && WiFi.SSID(i).length() > 0) {
        // Add to our array
        networks[networksCount] = WiFi.SSID(i);
        networksCount++;
        
        // Add to option list with signal strength
        networkList += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i);
        networkList += " (";
        
        // Signal strength icon based on RSSI
        int rssi = WiFi.RSSI(i);
        if (rssi > -50) {
          networkList += "Strong";
        } else if (rssi > -70) {
          networkList += "Good";
        } else {
          networkList += "Weak";
        }
        
        // Add encryption type
        networkList += ", ";
        switch (WiFi.encryptionType(i)) {
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

bool loadWiFiCredentials(String &ssid, String &password) {
  if (SPIFFS.exists(WIFI_FILE)) {
    File file = SPIFFS.open(WIFI_FILE, "r");
    if (file) {
      StaticJsonDocument<256> doc;
      DeserializationError error = deserializeJson(doc, file);
      file.close();
      
      if (!error) {
        ssid = doc["ssid"].as<String>();
        password = doc["password"].as<String>();
        return true;
      }
    }
  }
  return false;
}

void saveWiFiCredentials(const String &ssid, const String &password) {
  File file = SPIFFS.open(WIFI_FILE, "w");
  if (file) {
    StaticJsonDocument<256> doc;
    doc["ssid"] = ssid;
    doc["password"] = password;
    serializeJson(doc, file);
    file.close();
    Serial.println("WiFi credentials saved");
  } else {
    Serial.println("Failed to open file for writing");
  }
}

void setupWiFiAP() {
  if (wifiConnected) {
    WiFi.mode(WIFI_AP_STA);
  } else {
    WiFi.mode(WIFI_AP);
  }
  
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  
  // Initialize DNS server for captive portal
  dnsServer.start(DNS_PORT, "*", apIP);
  
  Serial.println("WiFi AP Mode Started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
}

void setupServer() {
  // Handle all web server requests - redirect to setup page
  server.onNotFound([](AsyncWebServerRequest *request) {
    // If the client requests any URI, send the setup page
    request->redirect("http://192.168.4.1/setup");
  });
  
  // Setup page
  server.on("/setup", HTTP_GET, [](AsyncWebServerRequest *request) {
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
                  "<form id='wifiForm'>"
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
                  "document.getElementById('wifiForm').addEventListener('submit', function(e){"
                  "e.preventDefault();"
                  "var ssid = document.getElementById('ssid').value;"
                  "var password = document.getElementById('password').value;"
                  "document.getElementById('status').innerHTML = 'Connecting to ' + ssid + '...';"
                  "document.getElementById('loader').style.display = 'block';"
                  "fetch('/connect', {"
                  "method: 'POST',"
                  "headers: {'Content-Type': 'application/json'},"
                  "body: JSON.stringify({ssid: ssid, password: password})"
                  "})"
                  ".then(response => response.json())"
                  ".then(data => {"
                  "document.getElementById('loader').style.display = 'none';"
                  "document.getElementById('status').innerHTML = data.message;"
                  "document.getElementById('status').style.backgroundColor = data.success ? '#d4edda' : '#f8d7da';"
                  "document.getElementById('status').style.color = data.success ? '#155724' : '#721c24';"
                  "if(data.success) {"
                  "setTimeout(function(){document.getElementById('status').innerHTML += '<br>Device will restart now. Connect to your regular WiFi network.'}, 2000);"
                  "}"
                  "});"
                  "});"
                  "</script>"
                  "</body></html>";
    
    request->send(200, "text/html", html);
  });
  
  // API endpoint to connect to a new WiFi network
  server.on("/connect", HTTP_POST, [](AsyncWebServerRequest *request) {
    // Not used, we'll use JSON response from onBody handler
  }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, data, len);
    
    String ssid = doc["ssid"];
    String password = doc["password"];
    
    AsyncResponseStream *response = request->beginResponseStream("application/json");
    StaticJsonDocument<256> responseDoc;
    
    if (ssid.length() > 0) {
      saveWiFiCredentials(ssid, password);
      responseDoc["success"] = true;
      responseDoc["message"] = "Credentials saved. ESP32 will restart and connect to your WiFi.";
    } else {
      responseDoc["success"] = false;
      responseDoc["message"] = "SSID cannot be empty";
    }
    
    serializeJson(responseDoc, *response);
    request->send(response);
    
    if (responseDoc["success"]) {
      // Allow time for the response to be sent before restarting
      delay(1000);
      ESP.restart();
    }
  });
  
  // Start the server
  server.begin();
}

bool connectToWiFi(const String &ssid, const String &password) {
  Serial.println("Attempting to connect to WiFi");
  Serial.print("SSID: ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  
  // Wait for connection, timeout after 20 seconds
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  
  return WiFi.status() == WL_CONNECTED;
}

void readSensorData() {
  // Read temperature and humidity
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  }
  
  // You can add more sensors here as needed
}