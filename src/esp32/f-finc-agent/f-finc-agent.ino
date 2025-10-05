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
#include <FS.h>
#include <SPIFFS.h>

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
    Serial.println(F("*ota:WiFi not connected! Skipping update check"));
    return;
  }

  // Tải Manifest
  Serial.println(F("*ota:Checking for firmware updates..."));

  HTTPClient http;
  http.begin(firmware_manifest_url);
  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf(F("*ota:Failed to download manifest, HTTP error (%d): %s\n"), httpCode, http.errorToString(httpCode).c_str());
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
          Serial.printf(F("*ota:Update failed: %s\n"), Update.errorString());
        }
      } else {
        Serial.println("*ota:Not enough space to begin OTA update.");
      }
    } else {
      Serial.println("*ota:No content length in HTTP response.");
    }
  } else {
    Serial.printf(F("*ota:HTTP GET failed, error: %s\n"), http.errorToString(httpCode).c_str());
  }

  /* WiFiClient client;
  t_httpUpdate_return ret = httpUpdate.update(client, binaryUrl);
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf(F("OTA Error (%d): %s\n"), httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
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

/** Alarm config */
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

const char* MODBUS_CONFIG_PATH = "/modbus_tcp.json";

bool loadModbusTCPConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*modbusTCP:SPIFFS mount failed while loading");
    return false;
  }
  if (!SPIFFS.exists(MODBUS_CONFIG_PATH)) {
    Serial.println("*modbusTCP:No existing config, skipping load");
    return false;
  }
  File file = SPIFFS.open(MODBUS_CONFIG_PATH, FILE_READ);
  if (!file) {
    Serial.println("*modbusTCP:Failed to open config file for reading");
    return false;
  }

  DynamicJsonDocument doc(16384);
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    Serial.print("*modbusTCP:Failed to parse config JSON: ");
    Serial.println(err.c_str());
    return false;
  }

  modbusTCPConfig.enabled = doc["enabled"].as<bool>();
  tcpDevices.clear();
  JsonArray devices = doc["devices"].as<JsonArray>();
  for (JsonObject dev : devices) {
    ModbusTcpDevice d;
    d.id = dev["id"].as<String>();
    d.name = dev["name"].as<String>();
    d.ip = dev["ip"].as<String>();
    d.port = dev["port"].as<int>();
    d.pollInterval = dev["pollInterval"].as<int>();
    d.enabled = dev["enabled"].as<bool>();
    d.connected = false;
    d.lastPollTime = 0;
    d.values.clear();

    JsonArray regs = dev["registers"].as<JsonArray>();
    for (JsonObject r : regs) {
      ModbusTcpRegister reg;
      reg.name = r["name"].as<String>();
      reg.address = r["address"].as<int>();
      reg.type = r["type"].as<String>();
      reg.byteOrder = r["byteOrder"].as<String>();
      reg.enabled = r["enabled"].as<bool>();
      reg.value = 0;
      reg.lastReadSuccess = false;
      d.registers.push_back(reg);
    }
    tcpDevices.push_back(d);
  }

  Serial.println("*modbusTCP:Config loaded");
  return true;
}
bool saveModbusTCPConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*modbusTCP:SPIFFS mount failed while saving");
    return false;
  }
  File file = SPIFFS.open(MODBUS_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("*modbusTCP:Failed to open config file for writing");
    return false;
  }

  DynamicJsonDocument doc(16384);
  doc["enabled"] = modbusTCPConfig.enabled;
  JsonArray devices = doc.createNestedArray("devices");
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    JsonObject dev = devices.createNestedObject();
    dev["id"] = tcpDevices[i].id;
    dev["name"] = tcpDevices[i].name;
    dev["ip"] = tcpDevices[i].ip;
    dev["port"] = tcpDevices[i].port;
    dev["pollInterval"] = tcpDevices[i].pollInterval;
    dev["enabled"] = tcpDevices[i].enabled;

    JsonArray regs = dev.createNestedArray("registers");
    for (size_t j = 0; j < tcpDevices[i].registers.size(); j++) {
      JsonObject r = regs.createNestedObject();
      r["name"] = tcpDevices[i].registers[j].name;
      r["address"] = tcpDevices[i].registers[j].address;
      r["type"] = tcpDevices[i].registers[j].type;
      r["byteOrder"] = tcpDevices[i].registers[j].byteOrder;
      r["enabled"] = tcpDevices[i].registers[j].enabled;
    }
  }

  bool ok = (serializeJson(doc, file) > 0);
  file.close();
  if (!ok) Serial.println(F("*modbusTCP:Config save failed"));
  return ok;
}

void initModbusTCP() {
  if (modbusTCPConfig.enabled && tcpDevices.size() > 0) {
    Serial.println("*modbusTCP:Initializing...");
    mb.client();
  }
}
void pollModbusDevice(ModbusTcpDevice& device);
void pollModbusDevice(ModbusTcpDevice& device) {
  if (!device.enabled || device.ip.isEmpty() || device.port <= 0) {
    return;
  }
  
  Serial.printf(F("*modbusTCP:Polling device %s (%s:%d)\n"), device.name.c_str(), device.ip.c_str(), device.port);
  
  // Khởi tạo địa chỉ IP cho thiết bị
  IPAddress serverIP;
  if (!serverIP.fromString(device.ip)) {
    Serial.println("*modbusTCP:Invalid IP address");
    device.connected = false;
    return;
  }
  
  // Kiểm tra kết nối và kết nối nếu cần
  if (!mb.isConnected(serverIP)) {
    mb.connect(serverIP, device.port);
    device.connected = mb.isConnected(serverIP);
    if (!device.connected) {
      Serial.println("*modbusTCP:Failed to connect");
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
    
    uint8_t slaveId = 1;
    bool requestOk = false;
    uint16_t value = 0;

    // Đọc giá trị từ thanh ghi (gửi yêu cầu)
    bool coilValue = false;
    bool discreteValue = false;
    if (reg.type == "coil") {
      requestOk = mb.readCoil(serverIP, reg.address, &coilValue, 1, nullptr, slaveId);
    } else if (reg.type == "discrete") {
      requestOk = mb.readIsts(serverIP, reg.address, &discreteValue, 1, nullptr, slaveId);
    } else if (reg.type == "holding") {
      requestOk = mb.readHreg(serverIP, reg.address, &value, 1, nullptr, slaveId);
    } else if (reg.type == "input") {
      requestOk = mb.readIreg(serverIP, reg.address, &value, 1, nullptr, slaveId);
    }

    // Chờ phản hồi ngắn để đảm bảo dữ liệu được cập nhật vào buffer
    // Thư viện ModbusIP hoạt động bất đồng bộ, cần gọi task() để xử lý gói tin.
    if (requestOk) {
      unsigned long start = millis();
      // Gọi task() vài lần trong ~100-200ms để xử lý phản hồi
      while (millis() - start < 150) {
        mb.task();
        delay(10);
      }
      // Sau khi task() xử lý xong, các biến buffer sẽ được ghi giá trị
      if (reg.type == "coil") {
        value = coilValue ? 1 : 0;
      } else if (reg.type == "discrete") {
        value = discreteValue ? 1 : 0;
      }

      device.values[i] = value;
      reg.lastReadSuccess = true;
      Serial.printf(F("*modbusTCP:Read %s register %d = %d\n"), reg.type.c_str(), reg.address, value);
    } else {
      reg.lastReadSuccess = false;
      Serial.printf(F("*modbusTCP:Failed to read %s register %d\n"), reg.type.c_str(), reg.address);
    }
  }
  
  // Cập nhật thời gian
  device.lastPollTime = millis();
  mb.task();
}

void handleModbusTCPConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  modbusTCPConfig.enabled = doc["enabled"].as<bool>();
  saveModbusTCPConfig();
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
  saveModbusTCPConfig();
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
      saveModbusTCPConfig();
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
      saveModbusTCPConfig();
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
      reg.byteOrder = doc["byteOrder"].as<String>();
      reg.enabled = doc["enabled"].as<bool>();
      
      tcpDevices[i].registers.push_back(reg);
      saveModbusTCPConfig();
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
      tcpDevices[i].registers[regIndex].byteOrder = doc["byteOrder"].as<String>();
      tcpDevices[i].registers[regIndex].enabled = doc["enabled"].as<bool>();
      
      saveModbusTCPConfig();
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
      saveModbusTCPConfig();
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

const char* MQTT_CONFIG_PATH = "/mqtt.json";

bool loadMqttConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*mqtt:SPIFFS mount failed while loading");
    return false;
  }
  if (!SPIFFS.exists(MQTT_CONFIG_PATH)) {
    Serial.println("*mqtt:No existing config, skipping load");
    return false;
  }
  File file = SPIFFS.open(MQTT_CONFIG_PATH, FILE_READ);
  if (!file) {
    Serial.println("*mqtt:Failed to open config file for reading");
    return false;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    Serial.print("*mqtt:Failed to parse config JSON: ");
    Serial.println(err.c_str());
    return false;
  }

  mqttConfig.enabled = doc["enabled"].as<bool>();
  mqttConfig.url = doc["url"].as<String>();
  mqttConfig.port = doc["port"].as<int>();
  mqttConfig.topic = doc["topic"].as<String>();
  mqttConfig.username = doc["username"].as<String>();
  mqttConfig.password = doc["password"].as<String>();

  Serial.println("*mqtt:Config loaded");
  return true;
}
bool saveMqttConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*mqtt:SPIFFS mount failed while saving");
    return false;
  }
  File file = SPIFFS.open(MQTT_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("*mqtt:Failed to open config file for writing");
    return false;
  }

  DynamicJsonDocument doc(1024);
  doc["enabled"] = mqttConfig.enabled;
  doc["url"] = mqttConfig.url;
  doc["port"] = mqttConfig.port;
  doc["topic"] = mqttConfig.topic;
  doc["username"] = mqttConfig.username;
  doc["password"] = mqttConfig.password;

  bool ok = (serializeJson(doc, file) > 0);
  file.close();
  if (!ok) Serial.println(F("*mqtt:Config save failed"));
  return ok;
}

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
    saveMqttConfig();
    request->send(200, "application/json", "{\"success\":true}");
}

/** API */
struct ApiConfig {
  String url;
  String username;
  String password;
  bool enabled;
} apiConfig = {"http://your-api.com/data", "apiuser", "apipass", false};

const char* API_CONFIG_PATH = "/api.json";

bool loadApiConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*api:SPIFFS mount failed while loading");
    return false;
  }
  if (!SPIFFS.exists(API_CONFIG_PATH)) {
    Serial.println("*api:No existing config, skipping load");
    return false;
  }
  File file = SPIFFS.open(API_CONFIG_PATH, FILE_READ);
  if (!file) {
    Serial.println("*api:Failed to open config file for reading");
    return false;
  }

  DynamicJsonDocument doc(1024);
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    Serial.print("*api:Failed to parse config JSON: ");
    Serial.println(err.c_str());
    return false;
  }

  apiConfig.enabled = doc["enabled"].as<bool>();
  apiConfig.url = doc["url"].as<String>();
  apiConfig.username = doc["username"].as<String>();
  apiConfig.password = doc["password"].as<String>();

  Serial.println("*api:Config loaded");
  return true;
}
bool saveApiConfig() {
  if (!SPIFFS.begin(true)) {
    Serial.println("*api:SPIFFS mount failed while saving");
    return false;
  }
  File file = SPIFFS.open(API_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("*api:Failed to open config file for writing");
    return false;
  }

  DynamicJsonDocument doc(1024);
  doc["enabled"] = apiConfig.enabled;
  doc["url"] = apiConfig.url;
  doc["username"] = apiConfig.username;
  doc["password"] = apiConfig.password;

  bool ok = (serializeJson(doc, file) > 0);
  file.close();
  if (!ok) Serial.println(F("*api:Config save failed"));
  return ok;
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
void handleApiConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  apiConfig.url = doc["url"].as<String>();
  apiConfig.username = doc["username"].as<String>();
  apiConfig.password = doc["password"].as<String>();
  apiConfig.enabled = doc["enabled"].as<bool>();
  saveApiConfig();
  request->send(200, "application/json", "{\"success\":true}");
}
void handlePostApiData(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
  String data = doc["data"].as<String>();
  request->send(200, "application/json", "{\"success\":true}");
}

/** Network Config */
struct NetworkConfig {
  bool useStaticIP;
  String ipAddress;
  String subnetMask;
  String gateway;
} networkConfig = {false, "", "", ""};

// Build aggregated status JSON payload (IO + System + Network + Modbus)
String buildAggregatedStatusPayload() {
  DynamicJsonDocument doc(32768);
  JsonObject system = doc.createNestedObject("system");
  system["firmware_version"] = FIRMWARE_VERSION;
  system["ram_total"] = ESP.getHeapSize();
  system["ram_free"] = ESP.getFreeHeap();
  system["flash_total"] = ESP.getFlashChipSize();
  system["flash_used"] = ESP.getSketchSize();
  system["flash_free_update"] = ESP.getFreeSketchSpace();
  system["cpu_freq"] = ESP.getCpuFreqMHz();
  system["cpu_cores"] = ESP.getChipCores();

  JsonObject network = doc.createNestedObject("network");
  network["mode"] = networkConfig.useStaticIP ? "static" : "dhcp";
  network["ip"] = WiFi.localIP().toString();

  JsonArray inputs = doc.createNestedArray("inputs");
  JsonArray outputs = doc.createNestedArray("outputs");
  for (int i = 0; i < NUM_INPUTS; i++) {
    JsonObject input = inputs.createNestedObject();
    input["id"] = i + 1;
    input["state"] = (digitalRead(INPUT_PINS[i]) == LOW);
  }
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    JsonObject output = outputs.createNestedObject();
    output["id"] = i + 1;
    output["state"] = outputStates[i];
  }

  JsonObject modbus = doc.createNestedObject("modbus");
  modbus["enabled"] = modbusTCPConfig.enabled;
  JsonArray devices = modbus.createNestedArray("devices");
  for (size_t i = 0; i < tcpDevices.size(); i++) {
    JsonObject dev = devices.createNestedObject();
    dev["id"] = tcpDevices[i].id;
    dev["name"] = tcpDevices[i].name;
    dev["ip"] = tcpDevices[i].ip;
    dev["port"] = tcpDevices[i].port;
    dev["enabled"] = tcpDevices[i].enabled;
    JsonArray regs = dev.createNestedArray("registers");
    for (size_t j = 0; j < tcpDevices[i].registers.size(); j++) {
      JsonObject reg = regs.createNestedObject();
      reg["name"] = tcpDevices[i].registers[j].name;
      reg["address"] = tcpDevices[i].registers[j].address;
      reg["type"] = tcpDevices[i].registers[j].type;
      reg["byteOrder"] = tcpDevices[i].registers[j].byteOrder;
      reg["enabled"] = tcpDevices[i].registers[j].enabled;
      if (j < tcpDevices[i].values.size()) {
        reg["value"] = tcpDevices[i].values[j];
      }
    }
  }

  String out;
  serializeJson(doc, out);
  return out;
}

const char* NETWORK_CONFIG_PATH = "/network.json";

bool loadNetworkConfig() {
  if (!SPIFFS.begin(true)) {
    return false;
  }
  if (!SPIFFS.exists(NETWORK_CONFIG_PATH)) {
    return false;
  }
  File file = SPIFFS.open(NETWORK_CONFIG_PATH, FILE_READ);
  if (!file) {
    return false;
  }
  DynamicJsonDocument doc(512);
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    return false;
  }
  networkConfig.useStaticIP = doc["useStaticIP"].as<bool>();
  networkConfig.ipAddress = doc["ipAddress"].as<String>();
  networkConfig.subnetMask = doc["subnetMask"].as<String>();
  networkConfig.gateway = doc["gateway"].as<String>();
  return true;
}
bool saveNetworkConfig() {
  if (!SPIFFS.begin(true)) {
    return false;
  }
  File file = SPIFFS.open(NETWORK_CONFIG_PATH, FILE_WRITE);
  if (!file) {
    return false;
  }
  DynamicJsonDocument doc(512);
  doc["useStaticIP"] = networkConfig.useStaticIP;
  doc["ipAddress"] = networkConfig.ipAddress;
  doc["subnetMask"] = networkConfig.subnetMask;
  doc["gateway"] = networkConfig.gateway;
  bool ok = (serializeJson(doc, file) > 0);
  file.close();
  return ok;
}
void handleNetworkConfig(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    bool staticIp = doc["staticIp"].as<bool>();
    String ip = doc["ipAddress"].as<String>();
    String subnet = doc["subnetMask"].as<String>();
    String gw = doc["gateway"].as<String>();

    networkConfig.useStaticIP = staticIp;
    networkConfig.ipAddress = ip;
    networkConfig.subnetMask = subnet;
    networkConfig.gateway = gw;

    bool ok = saveNetworkConfig();
    request->send(200, "application/json", String("{\"success\":") + (ok ? "true" : "false") + "}");

    if (ok) {
      // Restart shortly to apply new IP settings cleanly
      xTaskCreate([](void* pvParameters) {
          delay(200);
          ESP.restart();
          vTaskDelete(NULL);
        },
        "netcfgRestart", 2048, NULL, 1, NULL
      );
    }
}

/** Web Server */
extern const char index_html[] PROGMEM;
void handleJsonBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total, void (*handler)(AsyncWebServerRequest*, DynamicJsonDocument&)) {
  // Handle both chunked and non-chunked bodies. If total==0, treat current chunk as full body.
  bool isLastChunk = (index + len == total) || (total == 0);
  if (!isLastChunk) return;

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
void handleLogin(AsyncWebServerRequest *request, DynamicJsonDocument& doc) {
    String username = doc["username"];
    String password = doc["password"];
    bool success = (username == "admin" && password == "admin");
    request->send(200, "application/json", "{\"success\":" + String(success ? "true" : "false") + "}");
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
  loadNetworkConfig();
  if (networkConfig.useStaticIP) {
    IPAddress ip, gw, subnet;
    if (ip.fromString(networkConfig.ipAddress) &&
        gw.fromString(networkConfig.gateway) &&
        subnet.fromString(networkConfig.subnetMask)) {
      wm.setSTAStaticIPConfig(ip, gw, subnet);
    }
  }
  if (!wm.autoConnect("f-FINC Agent Setup", "12345678")) {
    delay(3000);
    ESP.restart();
  }

  // MQTT Services
  loadMqttConfig();
  if (mqttConfig.enabled) {
    mqttClient.setServer(mqttConfig.url.c_str(), mqttConfig.port);
    reconnectMqtt();
  }

  // Load API config
  loadApiConfig();

  // Initialize Modbus TCP if enabled
  loadModbusTCPConfig();
  if (modbusTCPConfig.enabled) {
    initModbusTCP();
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
  server.on("/config/api", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["enabled"] = apiConfig.enabled;
    doc["url"] = apiConfig.url;
    doc["username"] = apiConfig.username;
    doc["password"] = apiConfig.password;
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });
  server.on("/config/mqtt", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["enabled"] = mqttConfig.enabled;
    doc["url"] = mqttConfig.url;
    doc["port"] = mqttConfig.port;
    doc["topic"] = mqttConfig.topic;
    doc["username"] = mqttConfig.username;
    doc["password"] = mqttConfig.password;
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });
  server.on("/config/mqtt", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleMqttConfig);
  });
  server.on("/config/api", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleApiConfig);
  });
  server.on("/config/network", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(512);
    doc["useStaticIP"] = networkConfig.useStaticIP;
    doc["ipAddress"] = networkConfig.ipAddress;
    doc["subnetMask"] = networkConfig.subnetMask;
    doc["gateway"] = networkConfig.gateway;
    doc["currentIp"] = WiFi.localIP().toString();
    doc["mode"] = networkConfig.useStaticIP ? "static" : "dhcp";
    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
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

  server.on("/modbus-tcp/config", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusTCPConfig);
  });
  server.on("/modbus-tcp/config", HTTP_GET, [](AsyncWebServerRequest *request) {
    DynamicJsonDocument doc(256);
    doc["enabled"] = modbusTCPConfig.enabled;
    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });
  server.on("/modbus-tcp/devices", HTTP_GET, [](AsyncWebServerRequest *request) {
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
  // Aggregated status endpoint
  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    // Build a single JSON containing IO, system, network and modbus snapshot
    DynamicJsonDocument doc(32768);

    // System info
    JsonObject system = doc.createNestedObject("system");
    system["firmware_version"] = FIRMWARE_VERSION;
    system["ram_total"] = ESP.getHeapSize();
    system["ram_free"] = ESP.getFreeHeap();
    system["flash_total"] = ESP.getFlashChipSize();
    system["flash_used"] = ESP.getSketchSize();
    system["flash_free_update"] = ESP.getFreeSketchSpace();
    system["cpu_freq"] = ESP.getCpuFreqMHz();
    system["cpu_cores"] = ESP.getChipCores();

    // Network quick info
    JsonObject network = doc.createNestedObject("network");
    network["mode"] = networkConfig.useStaticIP ? "static" : "dhcp";
    network["ip"] = WiFi.localIP().toString();

    // IO status
    JsonArray inputs = doc.createNestedArray("inputs");
    JsonArray outputs = doc.createNestedArray("outputs");
    for (int i = 0; i < NUM_INPUTS; i++) {
      JsonObject input = inputs.createNestedObject();
      input["id"] = i + 1;
      input["state"] = (digitalRead(INPUT_PINS[i]) == LOW);
    }
    for (int i = 0; i < NUM_OUTPUTS; i++) {
      JsonObject output = outputs.createNestedObject();
      output["id"] = i + 1;
      output["state"] = outputStates[i];
    }

    // Modbus snapshot (devices and registers with last read values)
    JsonObject modbus = doc.createNestedObject("modbus");
    modbus["enabled"] = modbusTCPConfig.enabled;
    JsonArray devices = modbus.createNestedArray("devices");
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
        reg["lastReadSuccess"] = tcpDevices[i].registers[j].lastReadSuccess;
        if (j < tcpDevices[i].values.size()) {
          reg["value"] = tcpDevices[i].values[j];
        }
      }
    }

    String output;
    serializeJson(doc, output);
    request->send(200, "application/json", output);
  });

  // Helper removed from setup scope; replaced by global function definition
  server.on("/modbus-tcp/device/create", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceCreate);
  });
  server.on("/modbus-tcp/device/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceUpdate);
  });
  server.on("/modbus-tcp/device/delete", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusDeviceDelete);
  });
  server.on("/modbus-tcp/register/create", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterCreate);
  });
  server.on("/modbus-tcp/register/update", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterUpdate);
  });
  server.on("/modbus-tcp/register/delete", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      handleJsonBody(request, data, len, index, total, handleModbusRegisterDelete);
  });

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
      if (device.enabled && (currentMillis - device.lastPollTime >= device.pollInterval)) {
        pollModbusDevice(device);
      }
    }
  }

  // Logging data
  if (millis() - lastDataSend >= DATA_SEND_INTERVAL) {
    String mqttPayload = readAndSerializeData();
    sendMqttData(mqttPayload);
    String apiPayload = buildAggregatedStatusPayload();
    sendApiData(apiPayload);
    lastDataSend = millis();
  }

  // Checking firmware
  if (firmwareInterval > 0 && (millis() - lastFirmwareCheck >= (unsigned long)firmwareInterval * 3600000)) {
    handleUpdateFirmware();
    lastFirmwareCheck = millis();
  }
}