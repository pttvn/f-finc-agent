#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <vector>

// THƯ VIỆN BỔ SUNG
#include <HTTPClient.h> 
#include <PubSubClient.h> 
#include <WiFiClient.h> 
#include <HTTPUpdate.h>
#include <Update.h>

// THÊM THƯ VIỆN WATCHDOG
#include "esp_task_wdt.h"

// FIX CONFLICT
#define HTTP_METHOD_DEF 
#include <ESPAsyncWebServer.h>

// --- CẤU HÌNH FIRMWARE OTA TỰ ĐỘNG ---
#define FIRMWARE_VERSION 1.2f
const char* firmware_manifest_url = "https://pttvn.github.io/f-finc-agent/firmware/manifest.json"; 

// --- CẤU HÌNH GPIO ---
const int INPUT_PINS[] = {32, 33, 34, 35, 36, 39, 14, 13};
const int NUM_INPUTS = 8;
const int OUTPUT_PINS[] = {26, 25, 27, 18, 19, 23, 4, 2};
const int NUM_OUTPUTS = 8;
bool outputStates[NUM_OUTPUTS] = {false, false, false, false, false, false, false, false};

// --- CÁC ĐỐI TƯỢNG TOÀN CỤC CHO MẠNG ---
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char* mqtt_client_id = "ESP32_F_FINC_Agent_01";

// --- CÁC CẤU TRÚC VÀ BIẾN TOÀN CỤC ---
struct Device {
  String id;
  bool connected;
  std::vector<String> registers;
  std::vector<int> values;
};
std::vector<Device> rtuDevices;
std::vector<Device> tcpDevices;
std::vector<Device> snmpDevices;

struct AlarmConfig {
  String condition;
  int defaultInterval;
  int alarmInterval;
  bool isAlarmActive;
} alarmConfig = {"input1 > 10", 15, 5, false};

struct MqttConfig {
  String url;
  int port;
  String topic;
  String username;
  String password;
  bool enabled; 
} mqttConfig = {"test.mosquitto.org", 1883, "f-finc/data/01", "user", "pass", false};

struct ApiConfig {
  String url;
  String username;
  String password;
  bool enabled;
} apiConfig = {"http://your-api.com/data", "apiuser", "apipass", false};

int firmwareInterval = 12;
unsigned long lastFirmwareCheck = 0;

unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 5000;

extern const char index_html[] PROGMEM; 
AsyncWebServer server(80);

// --- HÀM CẬP NHẬT FIRMWARE TỰ ĐỘNG ---
void checkFirmwareUpdate() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("OTA: WiFi not connected, skipping update check.");
        return;
    }

    Serial.println("OTA: Checking for firmware updates...");
    Serial.print("OTA: Manifest URL: ");
    Serial.println(firmware_manifest_url);

    HTTPClient http;
    
    // 1. Tải Manifest (JSON)
    http.begin(firmware_manifest_url); 
    int httpCode = http.GET();
    
    if (httpCode == HTTP_CODE_OK) {
        String payload = http.getString();
        DynamicJsonDocument doc(256);
        DeserializationError error = deserializeJson(doc, payload);

        if (error) {
            Serial.print("OTA: Failed to parse manifest JSON: ");
            Serial.println(error.c_str());
        } else {
            float newVersion = doc["version"];
            const char* binaryUrl = doc["url"]; 

            Serial.print("OTA: Current version: ");
            Serial.println(FIRMWARE_VERSION, 2);
            Serial.print("OTA: Available version from server: ");
            Serial.println(newVersion, 2);
            Serial.print("OTA: Binary URL: ");
            Serial.println(binaryUrl);

            if (newVersion > FIRMWARE_VERSION) {
                Serial.println("OTA: New firmware available. Starting update process...");

                http.begin(binaryUrl);
                int httpCode = http.GET();
                if (httpCode == HTTP_CODE_OK) {
                  int contentLength = http.getSize();
                  if (contentLength > 0) {
                    bool canUpdate = Update.begin(contentLength);
                    if (canUpdate) {
                      WiFiClient &stream = http.getStream();
                      Update.writeStream(stream);
                      if (Update.end()) {
                        Serial.println("OTA update successful!");
                        ESP.restart();
                      } else {
                        Serial.printf("OTA update failed: %s\n", Update.errorString());
                      }
                    } else {
                      Serial.println("Not enough space to begin OTA update.");
                    }
                  } else {
                    Serial.println("No content length in HTTP response.");
                  }
                } else {
                  Serial.printf("HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
                }
                
                /* WiFiClient client;
                t_httpUpdate_return ret = httpUpdate.update(client, binaryUrl);

                switch (ret) {
                    case HTTP_UPDATE_FAILED:
                        Serial.printf("OTA Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
                        break;
                    case HTTP_UPDATE_NO_UPDATES:
                        Serial.println("OTA: No updates needed.");
                        break;
                    case HTTP_UPDATE_OK:
                        Serial.println("OTA: Update successful!");
                        break;
                } */
            } else {
                Serial.println("OTA: Firmware is already up to date.");
            }
        }
    } else {
        Serial.printf("OTA: Failed to download manifest, HTTP error (%d): %s\n", httpCode, http.errorToString(httpCode).c_str());
    }
    
    http.end(); // Kết thúc kết nối HTTP
}

/** Global functions */
String readAndSerializeData() {
    DynamicJsonDocument doc(512);
    JsonObject inputs = doc.createNestedObject("inputs");
    JsonObject outputs = doc.createNestedObject("outputs");

    for(int i = 0; i < NUM_INPUTS; i++) {
        inputs[String("i") + (i + 1)] = (digitalRead(INPUT_PINS[i]) == LOW);
    }
    for(int i = 0; i < NUM_OUTPUTS; i++) {
        outputs[String("o") + (i + 1)] = outputStates[i];
    }
    
    String output;
    serializeJson(doc, output);
    return output;
}
void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total, void (*handler)(AsyncWebServerRequest*, DynamicJsonDocument&)) {
    if (index + len == total) {
        String body = "";
        for (size_t i = 0; i < len; i++) {
            body += (char)data[i];
        }

        DynamicJsonDocument doc(1024);
        DeserializationError error = deserializeJson(doc, body);
        if (error) {
            request->send(400, "application/json", "{\"error\":\"Invalid JSON body\"}");
            return;
        }
        handler(request, doc);
    }
}

// Login
void handleLogin(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    String username = doc["username"];
    String password = doc["password"];
    bool success = (username == "admin" && password == "admin");
    request->send(200, "application/json", "{\"success\":" + String(success ? "true" : "false") + "}");
}

// IO
void handleOutputControl(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    int id = doc["id"]; 
    bool state = doc["state"];
    if (id >= 1 && id <= NUM_OUTPUTS) {
        int index = id - 1;
        outputStates[index] = state;
        digitalWrite(OUTPUT_PINS[index], state ? HIGH : LOW);
        request->send(200, "application/json", "{\"success\":true, \"id\":" + String(id) + ", \"state\":" + String(state ? "true" : "false") + "}");
    } else {
        request->send(400, "application/json", "{\"error\":\"Invalid output ID\"}");
    }
}

// Network config
void handleNetworkConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    bool staticIp = doc["staticIp"];
    request->send(200, "application/json", "{\"success\":true}");
}

// Alarm config
void handleAlarmConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    alarmConfig.condition = doc["condition"].as<String>();
    alarmConfig.defaultInterval = doc["defaultInterval"].as<int>();
    alarmConfig.alarmInterval = doc["alarmInterval"].as<int>();
    request->send(200, "application/json", "{\"success\":true}");
}

// MQTT
void reconnectMqtt() {
  if (!mqttConfig.enabled) return;
  Serial.println("MQTT: Reconnect routine started.");
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(mqtt_client_id, mqttConfig.username.c_str(), mqttConfig.password.c_str())) {
      Serial.println("connected!");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" trying again in 5 seconds");
      delay(5000);
    }
  }
}
void handleMqttConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    mqttConfig.url = doc["url"].as<String>();
    mqttConfig.port = doc["port"].as<int>();
    mqttConfig.topic = doc["topic"].as<String>();
    mqttConfig.username = doc["username"].as<String>();
    mqttConfig.password = doc["password"].as<String>();
    mqttConfig.enabled = doc["enabled"].as<bool>();
    
    if (mqttConfig.enabled) {
        mqttClient.setServer(mqttConfig.url.c_str(), mqttConfig.port);
        reconnectMqtt();
    } else if (mqttClient.connected()) {
        mqttClient.disconnect();
    }
    
    request->send(200, "application/json", "{\"success\":true}");
}
void sendMqttData(const String& payload) {
    if (!mqttConfig.enabled) return;
    if (!mqttClient.connected()) {
        reconnectMqtt();
    }
    
    if (mqttClient.connected()) {
        if (mqttClient.publish(mqttConfig.topic.c_str(), payload.c_str())) {
            Serial.println("MQTT: Successfully published data.");
        } else {
            Serial.println("MQTT: Publish failed.");
        }
    }
}

// REST API
void handleApiConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    apiConfig.url = doc["url"].as<String>();
    apiConfig.username = doc["username"].as<String>();
    apiConfig.password = doc["password"].as<String>();
    apiConfig.enabled = doc["enabled"].as<bool>(); 
    request->send(200, "application/json", "{\"success\":true}");
}
void handlePostApiData(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    String data = doc["data"].as<String>();
    request->send(200, "application/json", "{\"success\":true}");
}
void sendApiData(const String& payload) {
    if (!apiConfig.enabled) {
        return;
    }
    if (WiFi.status() == WL_CONNECTED && !apiConfig.url.isEmpty()) {
        HTTPClient http;
        Serial.print("API POST: Connecting to ");
        Serial.println(apiConfig.url);

        http.begin(apiConfig.url.c_str()); 
        http.addHeader("Content-Type", "application/json");

        if (!apiConfig.username.isEmpty() && !apiConfig.password.isEmpty()) {
            http.setAuthorization(apiConfig.username.c_str(), apiConfig.password.c_str());
        }

        int httpResponseCode = http.POST(payload);
        if (httpResponseCode > 0) {
            Serial.print("API POST: Success, HTTP code: ");
            Serial.println(httpResponseCode);
        } else {
            Serial.print("API POST: Failed, HTTP error code: ");
            Serial.println(httpResponseCode);
        }
        http.end();
    }
}

// --- SETUP ---
void setup() {
  Serial.begin(115200);

  /** Inputs & Outputs */
  for (int i = 0; i < NUM_INPUTS; i++) {
    int pin = INPUT_PINS[i];
    
    // Kiểm tra chân GPIO: Các chân >= 34 (34, 35, 36, 39) là Input-Only và KHÔNG hỗ trợ PULLUP.
    // Việc cố gắng cấu hình PULLUP cho các chân này thường gây ra lỗi GPIO 78/79.
    if (pin >= 34) {
        pinMode(pin, INPUT); // Chỉ đặt là INPUT để đọc giá trị
    } else {
        pinMode(pin, INPUT_PULLUP); // Đặt INPUT_PULLUP cho các chân hỗ trợ (32, 33, 14, 13)
    }
  }
  for (int i = 0; i < NUM_OUTPUTS; i++) {
      pinMode(OUTPUT_PINS[i], OUTPUT);
      digitalWrite(OUTPUT_PINS[i], LOW);
  }

  /** Wifi manager */
  WiFiManager wm;
  if (!wm.autoConnect("F-FINC_ESP32_SETUP", "12345678")) {
    delay(3000);
    ESP.restart();
  }

  /** MQTT Services */
  if (mqttConfig.enabled) {
      mqttClient.setServer(mqttConfig.url.c_str(), mqttConfig.port);
      reconnectMqtt();
  }

  /** Web services */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html);
  });
  
  server.on("/auth/login", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleLogin);
  });
  
  server.on("/control/output", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleOutputControl);
  });
  
  server.on("/io-status", HTTP_GET, [](AsyncWebServerRequest *request) {
    String dataPayload = readAndSerializeData(); 
    DynamicJsonDocument doc(1024);
    DeserializationError error = deserializeJson(doc, dataPayload);
    DynamicJsonDocument newDoc(1024);
    JsonArray inputs = newDoc.createNestedArray("inputs");
    JsonArray outputs = newDoc.createNestedArray("outputs");

    if (!error) {
        JsonObject inputsObj = doc.as<JsonObject>()["inputs"];
        JsonObject outputsObj = doc.as<JsonObject>()["outputs"];
        
        for (JsonPair p : inputsObj) {
            JsonObject input = inputs.add<JsonObject>();
            input["id"] = String(p.key().c_str()).substring(1).toInt(); 
            input["state"] = p.value().as<bool>();
        }
        for (JsonPair p : outputsObj) {
            JsonObject output = outputs.add<JsonObject>();
            output["id"] = String(p.key().c_str()).substring(1).toInt(); 
            output["state"] = p.value().as<bool>();
        }
    }
    
    String finalOutput;
    serializeJson(newDoc, finalOutput);
    request->send(200, "application/json", finalOutput);
  });
  
  server.on("/system-info", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["firmware_version"] = FIRMWARE_VERSION; // <-- THÊM DÒNG NÀY
    doc["ram_total"] = ESP.getHeapSize();
    doc["ram_free"] = ESP.getFreeHeap();
    doc["flash_total"] = ESP.getFlashChipSize();
    doc["flash_used"] = ESP.getSketchSize();
    doc["flash_free_update"] = ESP.getFreeSketchSpace(); 
    doc["cpu_freq"] = ESP.getCpuFreqMHz();
    doc["cpu_cores"] = ESP.getChipCores();
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });
  
  server.on("/config/mqtt", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleMqttConfig);
  });
  
  server.on("/config/api", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleApiConfig);
  });
  
  server.on("/config/network", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleNetworkConfig);
  });
  
  server.on("/config/alarm", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleAlarmConfig);
  });
  
  server.on("/api/post", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handlePostApiData);
  });
  
  // THAY ĐỔI: Endpoint này chỉ để lưu interval
  server.on("/firmware/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, 
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
        if (index + len == total) {
            String body;
            for(size_t i=0; i<len; i++) body += (char)data[i];
            DynamicJsonDocument doc(128);
            if (!deserializeJson(doc, body)) {
                firmwareInterval = doc["interval"];
                request->send(200, "application/json", "{\"success\":true}");
            } else {
                request->send(400, "application/json", "{\"success\":false}");
            }
        }
    });

  // THÊM MỚI: Endpoint để kích hoạt kiểm tra thủ công
  server.on("/firmware/check-now", HTTP_GET, [](AsyncWebServerRequest *request) {
      xTaskCreate(
          [](void* pvParameters) {
              checkFirmwareUpdate();
              vTaskDelete(NULL);
          },
          "otaCheckTask", 4096, NULL, 1, NULL
      );
      request->send(200, "application/json", "{\"success\":true, \"message\":\"Đã bắt đầu kiểm tra cập nhật.\"}");
  });

  server.begin();
}

// --- LOOP ---
void loop() {
  if (mqttConfig.enabled) {
    if (!mqttClient.connected()) {
      reconnectMqtt();
    }
    mqttClient.loop();
  }
  
  if (firmwareInterval > 0 && (millis() - lastFirmwareCheck >= (unsigned long)firmwareInterval * 3600000)) {
    checkFirmwareUpdate();
    lastFirmwareCheck = millis();
  }
  
  if (millis() - lastDataSend >= DATA_SEND_INTERVAL) {
      String payload = readAndSerializeData();
      sendMqttData(payload);
      sendApiData(payload);
      lastDataSend = millis();
  }
}