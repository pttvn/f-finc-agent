#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <vector>
#include <HTTPClient.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#include <Update.h>
#include <ModbusIP_ESP8266.h>

// FIX CONFLICT
#define HTTP_METHOD_DEF
#include <ESPAsyncWebServer.h>

/** GPIO */
const int INPUT_PINS[] = {32, 33, 34, 35, 36, 39, 14, 13};
const int NUM_INPUTS = 8;
const int OUTPUT_PINS[] = {26, 25, 27, 18, 19, 23, 4, 2};
const int NUM_OUTPUTS = 8;
bool outputStates[NUM_OUTPUTS] = {false, false, false, false, false, false, false, false};

unsigned long lastDataSend = 0;
const unsigned long DATA_SEND_INTERVAL = 5000;

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

/** FIRMWARE */
#define FIRMWARE_VERSION 1.4f

const char* firmware_manifest_url = "https://pttvn.github.io/f-finc-agent/firmware/manifest.json";
int firmwareInterval = 12;
unsigned long lastFirmwareCheck = 0;

void handleUpdateFirmware() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("*ota:WiFi not connected! Skipping update check");
    return;
  }

  // Tải Manifest
  Serial.println("*ota:Checking for firmware updates...");

  HTTPClient http;
  http.begin(firmware_manifest_url);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("*ota:Failed to download manifest, HTTP error (%d): %s\n", httpCode, http.errorToString(httpCode).c_str());
    http.end();
    return;
  }

  // Check version
  String payload = http.getString();
  DynamicJsonDocument doc(256);

  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    Serial.print("*ota:Failed to parse manifest JSON: ");
    Serial.println(error.c_str());
    http.end();
    return;
  }

  float newVersion = doc["version"];
  const char* binaryUrl = doc["url"];

  if (newVersion <= FIRMWARE_VERSION) {
    Serial.print("*ota:Current version: ");
    Serial.println(FIRMWARE_VERSION, 2);
    Serial.print("*ota:Available version: ");
    Serial.println(newVersion, 2);
    Serial.println("*ota:Firmware is already up to date.");

    http.end();
    return;
  }

  // Starting update
  Serial.println("*ota:New firmware available. Starting update process...");
  Serial.print("*ota:Binary url ");
  Serial.println(binaryUrl);

  http.begin(binaryUrl);
  httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength > 0) {
      bool canUpdate = Update.begin(contentLength);
      if (canUpdate) {
        WiFiClient &stream = http.getStream();
        Update.writeStream(stream);
        if (Update.end()) {
          Serial.println("*ota:Update successful! Restarting...");
          ESP.restart();
        } else {
          Serial.printf("*ota:Update failed: %s\n", Update.errorString());
        }
      } else {
        Serial.println("*ota:Not enough space to begin OTA update.");
      }
    } else {
      Serial.println("*ota:No content length in HTTP response.");
    }
  } else {
    Serial.printf("*ota:HTTP GET failed, error: %s\n", http.errorToString(httpCode).c_str());
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

  http.end();
}

// Alarm config
struct AlarmConfig {
  String condition;
  int defaultInterval;
  int alarmInterval;
  bool isAlarmActive;
} alarmConfig = {"input1 > 10", 15, 5, false};

void handleAlarmConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    alarmConfig.condition = doc["condition"].as<String>();
    alarmConfig.defaultInterval = doc["defaultInterval"].as<int>();
    alarmConfig.alarmInterval = doc["alarmInterval"].as<int>();
    request->send(200, "application/json", "{\"success\":true}");
}

/** Modbus TCP */
struct ModbusTcpRegister {
  String name;
  uint16_t address;
  String type; // "coil", "discrete", "holding", "input"
  String byteOrder; // "ABCD", "CDAB", "BADC", "DCBA"
  bool enabled;
  uint16_t value;
  bool lastReadSuccess;
};
struct ModbusTcpDevice {
  String id;
  String name;
  String ip;
  uint16_t port;
  uint16_t pollInterval;
  bool enabled;
  std::vector<ModbusTcpRegister> registers;
  unsigned long lastPollTime;
  bool connected;
  std::vector<uint16_t> values;
};
struct ModbusTCPConfig {
  bool enabled;
  std::vector<ModbusTcpDevice> devices;
  unsigned long lastPoll;
} modbusTCPConfig;

std::vector<ModbusTcpDevice> tcpDevices;

ModbusIP mb;

void initModbusTCP() {
  if (modbusTCPConfig.enabled && tcpDevices.size() > 0) {
    Serial.println("Modbus TCP: Initializing...");
    mb.client();
  }
}
void pollModbusDevice(ModbusTcpDevice& device);
void pollModbusDevice(ModbusTcpDevice& device) {
  if (!device.enabled || device.ip.isEmpty() || device.port <= 0) {
    return;
  }
  
  Serial.printf("Modbus TCP: Polling device %s (%s:%d)\n", device.name.c_str(), device.ip.c_str(), device.port);
  
  // Khởi tạo địa chỉ IP cho thiết bị
  IPAddress serverIP;
  if (!serverIP.fromString(device.ip)) {
    Serial.println("Modbus TCP: Invalid IP address");
    device.connected = false;
    return;
  }
  
  // Kiểm tra kết nối và kết nối nếu cần
  if (!mb.isConnected(serverIP)) {
    mb.connect(serverIP, device.port);
    device.connected = mb.isConnected(serverIP);
    if (!device.connected) {
      Serial.println("Modbus TCP: Failed to connect");
      return;
    }
  }
  
  // Poll each register
  for (size_t i = 0; i < device.registers.size(); i++) {
    ModbusTcpRegister& reg = device.registers[i];
    if (!reg.enabled) continue;
    
    // Đảm bảo vector có đủ không gian
    if (i >= device.values.size()) {
      device.values.resize(i + 1, 0);
    }
    
    uint8_t slaveId = 1;  // ID thiết bị Modbus
    bool success = false;
    uint16_t value = 0;
     
    // Đọc giá trị từ thanh ghi dựa trên loại
    if (reg.type == "coil") {
      // Đọc coil (0xxxx)
      bool coilValue = false;
      success = mb.readCoil(serverIP, reg.address, &coilValue);
      value = coilValue ? 1 : 0;
    } else if (reg.type == "discrete") {
      // Đọc discrete input (1xxxx)
      bool discreteValue = false;
      success = mb.readIsts(serverIP, reg.address, &discreteValue);
      value = discreteValue ? 1 : 0;
    } else if (reg.type == "holding") {
      // Đọc holding register (4xxxx)
      success = mb.readHreg(serverIP, reg.address, &value);
    } else if (reg.type == "input") {
      // Đọc input register (3xxxx)
      success = mb.readIreg(serverIP, reg.address, &value);
    }
    
    if (success) {
      device.values[i] = value;
      reg.lastReadSuccess = true;
      Serial.printf("Modbus TCP: Read %s register %d = %d\n", reg.type.c_str(), reg.address, value);
    } else {
      reg.lastReadSuccess = false;
      Serial.printf("Modbus TCP: Failed to read %s register %d\n", reg.type.c_str(), reg.address);
    }
  }
  
  // Cập nhật thời gian poll cuối cùng
  device.lastPollTime = millis();
  
  // Xử lý các tác vụ Modbus
  mb.task();
}
void handleModbusTCPConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  modbusTCPConfig.enabled = doc["enabled"].as<bool>();
  request->send(200, "application/json", "{\"success\":true}");
}
void handleModbusDeviceCreate(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  ModbusTcpDevice device;
  device.id = doc["id"].as<String>();
  device.name = doc["name"].as<String>();
  device.ip = doc["ip"].as<String>();
  device.port = doc["port"].as<int>();
  device.pollInterval = doc["pollInterval"].as<int>();
  device.enabled = doc["enabled"].as<bool>();
  device.connected = false;
  
  tcpDevices.push_back(device);
  request->send(200, "application/json", "{\"success\":true}");
}
void handleModbusDeviceUpdate(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String id = doc["id"].as<String>();
  
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    if (tcpDevices[i].id == id) {
      tcpDevices[i].name = doc["name"].as<String>();
      tcpDevices[i].ip = doc["ip"].as<String>();
      tcpDevices[i].port = doc["port"].as<int>();
      tcpDevices[i].pollInterval = doc["pollInterval"].as<int>();
      tcpDevices[i].enabled = doc["enabled"].as<bool>();
      request->send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  request->send(404, "application/json", "{\"success\":false, \"error\":\"ModbusTcpDevice not found\"}");
}
void handleModbusDeviceDelete(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String id = doc["id"].as<String>();
  
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    if (tcpDevices[i].id == id) {
      tcpDevices.erase(tcpDevices.begin() + i);
      request->send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  request->send(404, "application/json", "{\"success\":false, \"error\":\"ModbusTcpDevice not found\"}");
}
void handleModbusRegisterCreate(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String deviceId = doc["deviceId"].as<String>();
  
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    if (tcpDevices[i].id == deviceId) {
      ModbusTcpRegister reg;
      reg.name = doc["name"].as<String>();
      reg.address = doc["address"].as<int>();
      reg.type = doc["type"].as<String>();
      reg.byteOrder = doc["byteOrder"].as<int>();
      reg.enabled = doc["enabled"].as<bool>();
      
      tcpDevices[i].registers.push_back(reg);
      request->send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  request->send(404, "application/json", "{\"success\":false, \"error\":\"ModbusTcpDevice not found\"}");
}
void handleModbusRegisterUpdate(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String deviceId = doc["deviceId"].as<String>();
  int regIndex = doc["regIndex"].as<int>();
  
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    if (tcpDevices[i].id == deviceId && regIndex >= 0 && regIndex < tcpDevices[i].registers.size()) {
      tcpDevices[i].registers[regIndex].name = doc["name"].as<String>();
      tcpDevices[i].registers[regIndex].address = doc["address"].as<int>();
      tcpDevices[i].registers[regIndex].type = doc["type"].as<String>();
      tcpDevices[i].registers[regIndex].byteOrder = doc["byteOrder"].as<int>();
      tcpDevices[i].registers[regIndex].enabled = doc["enabled"].as<bool>();
      
      request->send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  request->send(404, "application/json", "{\"success\":false, \"error\":\"ModbusTcpDevice or register not found\"}");
}
void handleModbusRegisterDelete(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String deviceId = doc["deviceId"].as<String>();
  int regIndex = doc["regIndex"].as<int>();
  
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    if (tcpDevices[i].id == deviceId && regIndex >= 0 && regIndex < tcpDevices[i].registers.size()) {
      tcpDevices[i].registers.erase(tcpDevices[i].registers.begin() + regIndex);
      request->send(200, "application/json", "{\"success\":true}");
      return;
    }
  }
  
  request->send(404, "application/json", "{\"success\":false, \"error\":\"ModbusTcpDevice or register not found\"}");
}

/** MQTT */
WiFiClient espClient;
PubSubClient mqttClient(espClient);
const char* mqtt_client_id = "ESP32_F_FINC_Agent_01";

struct MqttConfig {
  String url;
  int port;
  String topic;
  String username;
  String password;
  bool enabled;
} mqttConfig = {"test.mosquitto.org", 1883, "f-finc/data/01", "user", "pass", false};

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

/** API */
struct ApiConfig {
  String url;
  String username;
  String password;
  bool enabled;
} apiConfig = {"http://your-api.com/data", "apiuser", "apipass", false};

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

/** Web Server */
extern const char index_html[] PROGMEM;
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
void handleLogin(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    String username = doc["username"];
    String password = doc["password"];
    bool success = (username == "admin" && password == "admin");
    request->send(200, "application/json", "{\"success\":" + String(success ? "true" : "false") + "}");
}
void handleNetworkConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    bool staticIp = doc["staticIp"];
    request->send(200, "application/json", "{\"success\":true}");
}

AsyncWebServer server(80);

/** SETUP */
void setup() {
  Serial.begin(115200);

  // Inputs & Outputs
  for (int i = 0; i < NUM_INPUTS; i++) {
    int pin = INPUT_PINS[i];

    // Kiểm tra chân GPIO: Các chân >= 34 (34, 35, 36, 39) là Input-Only và KHÔNG hỗ trợ PULLUP.
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

  // Wifi manager
  WiFiManager wm;
  if (!wm.autoConnect("F-FINC_ESP32_SETUP", "12345678")) {
    delay(3000);
    ESP.restart();
  }

  // MQTT Services
  if (mqttConfig.enabled) {
    mqttClient.setServer(mqttConfig.url.c_str(), mqttConfig.port);
    reconnectMqtt();
  }

  // Web services
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

  // Modbus TCP API endpoints
  server.on("/config/modbus-tcp", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusTCPConfig);
  });
  server.on("/modbus/device/create", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceCreate);
  });
  server.on("/modbus/device/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceUpdate);
  });
  server.on("/modbus/device/delete", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceDelete);
  });
  server.on("/modbus/register/create", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterCreate);
  });
  server.on("/modbus/register/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterUpdate);
  });
  server.on("/modbus/register/delete", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterDelete);
  });
  server.on("/modbus/devices", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(4096);
    JsonArray devices = doc.createNestedArray("devices");
    
    for (size_t i = 0; i < tcpDevices.size(); i++) {
      JsonObject device = devices.createNestedObject();
      device["id"] = tcpDevices[i].id;
      device["name"] = tcpDevices[i].name;
      device["ip"] = tcpDevices[i].ip;
      device["port"] = tcpDevices[i].port;
      device["pollInterval"] = tcpDevices[i].pollInterval;
      device["enabled"] = tcpDevices[i].enabled;
      device["connected"] = tcpDevices[i].connected;
      
      JsonArray registers = device.createNestedArray("registers");
      for (size_t j = 0; j < tcpDevices[i].registers.size(); j++) {
        JsonObject reg = registers.createNestedObject();
        reg["name"] = tcpDevices[i].registers[j].name;
        reg["address"] = tcpDevices[i].registers[j].address;
        reg["type"] = tcpDevices[i].registers[j].type;
        reg["byteOrder"] = tcpDevices[i].registers[j].byteOrder;
        reg["enabled"] = tcpDevices[i].registers[j].enabled;
        
        // Add value if available
        if (j < tcpDevices[i].values.size()) {
          reg["value"] = tcpDevices[i].values[j];
        }
      }
    }
    
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });

  // FIRMWARE
  server.on("/firmware/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
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
  server.on("/firmware/check-now", HTTP_GET, [](AsyncWebServerRequest *request) {
    xTaskCreate([](void* pvParameters) {
        handleUpdateFirmware();
        vTaskDelete(NULL);
      },
      "otaCheckTask", 4096, NULL, 1, NULL
    );
    request->send(200, "application/json", "{\"success\":true, \"message\":\"Đã bắt đầu kiểm tra cập nhật.\"}");
  });

  // START
  server.begin();
  
  // Initialize Modbus TCP if enabled
  if (modbusTCPConfig.enabled) {
    initModbusTCP();
  }
}

/** LOOP */
void loop() {
  if (mqttConfig.enabled) {
    if (!mqttClient.connected()) {
      reconnectMqtt();
    }
    mqttClient.loop();
  }

  // Handle Modbus TCP polling
  if (modbusTCPConfig.enabled) {
    unsigned long currentMillis = millis();
    for (auto& device : tcpDevices) {
      if (device.enabled && (currentMillis - modbusTCPConfig.lastPoll >= device.pollInterval)) {
        modbusTCPConfig.lastPoll = currentMillis;
        pollModbusDevice(device);
      }
    }
  }

  // Logging data
  if (millis() - lastDataSend >= DATA_SEND_INTERVAL) {
    String payload = readAndSerializeData();
    sendMqttData(payload);
    sendApiData(payload);
    lastDataSend = millis();
  }

  // Checking firmware
  if (firmwareInterval > 0 && (millis() - lastFirmwareCheck >= (unsigned long)firmwareInterval * 3600000)) {
    handleUpdateFirmware();
    lastFirmwareCheck = millis();
  }
}