#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <NTPClient.h>
#include <time.h>
// WiFi & MQTT config
const char *ssid = "PIFLab_M5";
const char *password = "khonghoisaocopass";

const char *mqtt_server = "b69c9261fa01418fbf8478adb5b93b27.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;
const char *mqtt_user = "admin";
const char *mqtt_pass = "Admin@12345";

const char *topic_publish = "esp32/data";
const char *topic_subscribe = "pif/contactor/register/response";
const char *device_id = "";
String deviceId = "";

unsigned long buttonPressTime = 0;
bool buttonWasPressed = false;

#define DEVICE_ID_ADDR 0
#define RELAY_PIN 5
#define SW_USER_PIN 15

#define TOPIC_SUB_ADDR 32
#define MAX_TOPIC_LENGTH 64
#define EEPROM_SIZE 512

#define MODE_ADDR 96
#define ON_DURATION_ADDR 100
#define OFF_DURATION_ADDR 104
#define SCHEDULE_COUNT_ADDR 108
#define SCHEDULES_START_ADDR 112

WiFiClientSecure espClient;
PubSubClient client(espClient);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000);

enum ControlMode {
  MODE_NONE,
  MODE_TIME,
  MODE_CYCLE
};

ControlMode currentMode;
ControlMode previousMode = MODE_NONE;  // Lưu lại mode trước đó khi vào MODE_NONE

// Chế độ time
const int MAX_SCHEDULES = 10;
int scheduleCount = 0;
int onHours[MAX_SCHEDULES];
int onMinutes[MAX_SCHEDULES];
int offHours[MAX_SCHEDULES];
int offMinutes[MAX_SCHEDULES];

// Chế độ cycle
unsigned long onDuration = 3600;   // giây
unsigned long offDuration = 3600;  // giây
unsigned long lastToggleTime = 0;
bool relayState = false;
static int wifiRetryCount = 0;
static int retryConnection = 50;

// Machine state
enum State {
  STATE_WIFI_CONNECTING,
  STATE_MQTT_CONNECTING,
  STATE_RUNNING
};

State currentState = STATE_WIFI_CONNECTING;

unsigned long lastMsg = 0;

void saveAllSettingsToEEPROM() {
  Serial.println("----- Saving All Settings to EEPROM -----");

  // Save current mode
  EEPROM.put(MODE_ADDR, currentMode);
  Serial.print("Saved current mode: ");
  Serial.println(currentMode);

  // Always save CYCLE mode settings
  EEPROM.put(ON_DURATION_ADDR, onDuration);
  Serial.println("=================================");
  Serial.print("Saved onDuration: ");
  Serial.print(onDuration);
  Serial.println("s");

  EEPROM.put(OFF_DURATION_ADDR, offDuration);
  Serial.print("Saved offDuration: ");
  Serial.print(offDuration);
  Serial.println("s");
  Serial.println("=================================");
  // Always save TIME mode settings
  EEPROM.put(SCHEDULE_COUNT_ADDR, scheduleCount);
  Serial.print("Saved scheduleCount: ");
  Serial.println(scheduleCount);

  int addr = SCHEDULES_START_ADDR;
  for (int i = 0; i < scheduleCount; i++) {
    EEPROM.put(addr, onHours[i]);
    Serial.print("Saved schedule[");
    Serial.print(i);
    Serial.println("]: ");

    Serial.print("onHours: ");
    Serial.print(onHours[i]);
    addr += sizeof(int);

    EEPROM.put(addr, onMinutes[i]);
    Serial.print(", onMinutes: ");
    Serial.println(onMinutes[i]);
    addr += sizeof(int);

    EEPROM.put(addr, offHours[i]);
    Serial.print(" offHours: ");
    Serial.print(offHours[i]);
    addr += sizeof(int);

    EEPROM.put(addr, offMinutes[i]);
    Serial.print(", offMinutes: ");
    Serial.println(offMinutes[i]);
    addr += sizeof(int);
  }
  Serial.println("=================================");

  EEPROM.commit();
  Serial.println("All settings saved to EEPROM");
  Serial.println("---------------------------");
}

void loadAllSettingsFromEEPROM() {
  Serial.println("----- Loading All Settings from EEPROM -----");

  // Load current mode
  EEPROM.get(MODE_ADDR, currentMode);
  Serial.print("Loaded current mode: ");
  Serial.println(currentMode);

  // Always load CYCLE mode settings
  Serial.println("=================================");
  EEPROM.get(ON_DURATION_ADDR, onDuration);
  Serial.print("Loaded onDuration: ");
  Serial.print(onDuration);
  Serial.println("s");

  EEPROM.get(OFF_DURATION_ADDR, offDuration);
  Serial.print("Loaded offDuration: ");
  Serial.print(offDuration);
  Serial.println("s");
  Serial.println("=================================");

  lastToggleTime = millis();

  // Always load TIME mode settings
  EEPROM.get(SCHEDULE_COUNT_ADDR, scheduleCount);
  Serial.print("Loaded scheduleCount: ");
  Serial.println(scheduleCount);

  // Validate scheduleCount
  if (scheduleCount > MAX_SCHEDULES) {
    Serial.println("Invalid scheduleCount, resetting to 0");
    scheduleCount = 0;
  }
  int addr = SCHEDULES_START_ADDR;
  for (int i = 0; i < scheduleCount; i++) {

    Serial.print("Schedule[");
    Serial.print(i);
    Serial.println("]: ");

    EEPROM.get(addr, onHours[i]);
    Serial.print("onHours: ");

    Serial.print(onHours[i]);
    addr += sizeof(int);

    EEPROM.get(addr, onMinutes[i]);
    Serial.print(", onMinutes: ");
    Serial.println(onMinutes[i]);
    addr += sizeof(int);

    EEPROM.get(addr, offHours[i]);
    Serial.print("offHours: ");
    Serial.print(offHours[i]);
    addr += sizeof(int);

    EEPROM.get(addr, offMinutes[i]);
    Serial.print(", offMinutes: ");
    Serial.println(offMinutes[i]);
    addr += sizeof(int);
  }
  Serial.println("=================================");
}

String readDeviceIdFromEEPROM() {
  String id = "";
  for (int i = DEVICE_ID_ADDR; i < EEPROM_SIZE; i++) {
    char c = EEPROM.read(i);
    if (c == '\0' || c == 255)
      break;
    id += c;
  }
  return id;
}

void writeDeviceIdToEEPROM(String id) {
  for (int i = 0; i < id.length(); i++) {
    EEPROM.write(DEVICE_ID_ADDR + i, id[i]);
  }
  EEPROM.write(DEVICE_ID_ADDR + id.length(), '\0');
  EEPROM.commit();
}

void subscribeToDeviceConfigTopic(const String &id) {
  String topic = "pif/contactor/cfg/" + id;
  client.subscribe(topic.c_str());

  saveTopicSubToEEPROM(topic);

  Serial.print("Subscribed to config topic: ");
  Serial.println(topic);
}

void saveTopicSubToEEPROM(String topic) {
  if (topic.length() >= MAX_TOPIC_LENGTH) {
    Serial.println("Topic quá dài, không lưu EEPROM");
    return;
  }

  for (int i = 0; i < topic.length(); i++) {
    EEPROM.write(TOPIC_SUB_ADDR + i, topic[i]);
  }
  EEPROM.write(TOPIC_SUB_ADDR + topic.length(), '\0');
  EEPROM.commit();
}

String readTopicSubFromEEPROM() {
  char buffer[64];
  int i = 0;
  char ch;
  while ((ch = EEPROM.read(TOPIC_SUB_ADDR + i)) != '\0' && i < sizeof(buffer) - 1) {
    buffer[i++] = ch;
  }
  buffer[i] = '\0';
  return String(buffer);
}

bool isPrintableString(String str) {
  for (int i = 0; i < str.length(); i++) {
    if (!isPrintable(str.charAt(i))) {
      return false;
    }
  }
  return true;
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("wifi connection lost, return wifi connection status");
    currentState = STATE_WIFI_CONNECTING;
    return;
  }

  if (!client.connected()) {
    Serial.println("connecting MQTT...");
    espClient.setInsecure();
    if (client.connect("ESP32_Client", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT is connected!");

      sendMacAddressToMQTT();

      String topic_sub = readTopicSubFromEEPROM();
      if (topic_sub.length() > 0 && isPrintableString(topic_sub)) {
        client.subscribe(topic_sub.c_str());
        Serial.print("Subscribed from EEPROM: ");
        Serial.println(topic_sub);
        sendHandshakePacket();
      } else {

        Serial.println("Chưa có topic_sub, đang chờ device_id...");
        client.subscribe(topic_subscribe);
      }
      currentState = STATE_RUNNING;
    } else {
      Serial.print("Error MQTT: ");
      Serial.println(client.state());
    }
  }
}

// Kết nối WiFi
void connectWiFi() {

  if (WiFi.status() != WL_CONNECTED) {
    if (wifiRetryCount == 0) {
      Serial.println("Connecting wifi...");
      WiFi.begin(ssid, password);
    }

    delay(500);
    wifiRetryCount++;
    Serial.printf("[WiFi] try again %d...\n", wifiRetryCount);

    if (wifiRetryCount >= retryConnection) {
      Serial.println("[WiFi] connection failed, reset ESP...");
      delay(2000);
      ESP.restart();  // Reset ESP32
    }
  } else {
    Serial.println("[WiFi] wifi is connected!");
    Serial.print("[WiFi] IP address: ");
    Serial.println(WiFi.localIP());

    loadAllSettingsFromEEPROM();

    wifiRetryCount = 0;  // Reset bộ đếm nếu kết nối thành công
    currentState = STATE_MQTT_CONNECTING;
  }
}

String formatTimestamp(time_t rawTime) {
  struct tm *timeinfo = localtime(&rawTime);
  char buffer[25];
  sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
  return String(buffer);
}

unsigned long getTime() {
  timeClient.update();
  unsigned long now = timeClient.getEpochTime();
  return now;
}

void sendHandshakePacket() {
  String macAddress = WiFi.macAddress();        // Lấy MAC
  String deviceId = readDeviceIdFromEEPROM();   // Lấy device_id từ EEPROM
  String timeStr = formatTimestamp(getTime());  // Lấy thời gian thực

  StaticJsonDocument<256> doc;
  doc["id"] = deviceId;
  doc["time"] = timeStr;

  JsonObject data = doc.createNestedObject("data");
  JsonObject handshake = data.createNestedObject("handshake");
  handshake["imei"] = macAddress;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);

  if (client.connected()) {
    client.publish(topic_publish, buffer, len);  // topic_publish là topic MQTT bạn định nghĩa sẵn
    Serial.println("Handshake packet sent:");
    Serial.println(buffer);
  } else {
    Serial.println("MQTT not connected. Cannot send handshake packet.");
  }
}

void sendMacAddressToMQTT() {
  String macAddress = WiFi.macAddress();  // Lấy địa chỉ MAC của ESP32
  Serial.print("MAC Address: ");
  Serial.println(macAddress);

  StaticJsonDocument<256> doc;
  JsonObject data = doc.createNestedObject("data");
  data["imei"] = macAddress;

  char buffer[256];
  size_t len = serializeJson(doc, buffer);

  // Gửi lên MQTT
  if (client.connected()) {
    client.publish(topic_publish, buffer, len);
    Serial.println("MAC address sent to MQTT.");
  } else {
    Serial.println("MQTT not connected. Cannot send MAC address.");
  }
}

bool lastRelayState = false;  // Khởi tạo biến để nhớ trạng thái trước đó
void setRelay(bool state) {
  if (state != lastRelayState) {
    digitalWrite(RELAY_PIN, state ? HIGH : LOW);
    relayState = state;
    lastRelayState = state;

    Serial.println(state ? "RELAY ON" : "RELAY OFF");

    // Cập nhật thời gian
    timeClient.update();
    String formattedTime = timeClient.getFormattedTime();

    // Tạo JSON
    StaticJsonDocument<256> doc;
    doc["relay"] = state ? "ON" : "OFF";
    doc["timestamp"] = formattedTime;

    char buffer[256];
    size_t len = serializeJson(doc, buffer);

    // Gửi MQTT
    if (client.connected()) {
      client.publish(topic_publish, buffer, len);
      Serial.println("Relay status published to server.");
    } else {
      Serial.println("MQTT not connected. Cannot publish.");
    }
  }
}

// Callback xử lý khi nhận message
void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Nhận từ topic: ");
  Serial.println(topic);
  payload[length] = '\0';

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.print("Lỗi parse JSON: ");
    Serial.println(err.c_str());
    return;
  }

  String topicStr = String(topic);

  // Nếu là phản hồi register
  if (topicStr == "pif/contactor/register/response") {
    if (doc.containsKey("device_id")) {
      deviceId = doc["device_id"].as<String>();
      Serial.print("Đã nhận device_id: ");
      Serial.println(deviceId);

      writeDeviceIdToEEPROM(deviceId);
      subscribeToDeviceConfigTopic(deviceId);
    }
    return;  // Chỉ return ở đây
  }

  // Các topic khác sẽ xử lý mode
  String mode = doc["mode"] | "";

  if (mode == "time") {
    currentMode = MODE_TIME;

    Serial.print("check current mode: ");
    Serial.println(currentMode);
    scheduleCount = 0;
    JsonArray schedules = doc["schedules"].as<JsonArray>();
    for (JsonObject sched : schedules) {
      if (scheduleCount >= MAX_SCHEDULES)
        break;
      onHours[scheduleCount] = sched["onHour"] | 0;
      onMinutes[scheduleCount] = sched["onMinute"] | 0;
      offHours[scheduleCount] = sched["offHour"] | 0;
      offMinutes[scheduleCount] = sched["offMinute"] | 0;
      scheduleCount++;
    }
    saveAllSettingsToEEPROM();
    Serial.printf("Set mode: TIME with %d schedule(s)\n", scheduleCount);
  } else if (mode == "cycle") {
    currentMode = MODE_CYCLE;
    Serial.print("check current mode: ");
    Serial.println(currentMode);

    onDuration = doc["onDuration"] | 3600;
    offDuration = doc["offDuration"] | 3600;
    Serial.print("check onDuration: ");
    Serial.println(onDuration);

    Serial.print("check offDuration: ");
    Serial.println(offDuration);

    saveAllSettingsToEEPROM();
    lastToggleTime = millis();
    Serial.println("Set mode: CYCLE");
  } else {
    currentMode = MODE_NONE;
    Serial.println("Set mode: NONE");
  }
}

void handleRelayControl() {
  if (currentMode == MODE_TIME) {

    static unsigned long lastNTPUpdate = 0;

    // Cập nhật NTP mỗi 10 giây để tránh quá tải
    if (millis() - lastNTPUpdate > 10000) {
      timeClient.update();
      lastNTPUpdate = millis();
    }
    time_t epoch = timeClient.getEpochTime();
    struct tm *timeinfo = gmtime((time_t *)&epoch);

    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 1000) {
      lastPrintTime = millis();
      Serial.printf(">>> Current time: %02d:%02d:%02d\n", timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
    }
    int nowMins = timeinfo->tm_hour * 60 + timeinfo->tm_min;
    bool shouldBeOn = false;

    for (int i = 0; i < scheduleCount; ++i) {
      int onMins = onHours[i] * 60 + onMinutes[i];
      int offMins = offHours[i] * 60 + offMinutes[i];

      bool isInTimeRange = (onMins < offMins)
                             ? (nowMins >= onMins && nowMins < offMins)
                             : (nowMins >= onMins || nowMins < offMins);

      if (isInTimeRange) {
        shouldBeOn = true;
        break;
      }
    }

    setRelay(shouldBeOn);
  } else if (currentMode == MODE_CYCLE) {
    unsigned long currentMillis = millis();
    unsigned long cycleTime = (relayState ? onDuration : offDuration) * 1000;

    if (currentMillis - lastToggleTime >= cycleTime) {
      relayState = !relayState;
      setRelay(relayState);
      lastToggleTime = currentMillis;
    }
  }
}

void handleUserButton() {
  static int lastStableState = HIGH;
  static int lastReading = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  int currentReading = digitalRead(SW_USER_PIN);

  if (currentReading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (currentReading != lastStableState) {
      lastStableState = currentReading;

      if (lastStableState == LOW) {
        buttonPressTime = millis();
        buttonWasPressed = true;
        Serial.println("Button pressed");
      }

      if (lastStableState == HIGH && buttonWasPressed) {
        unsigned long pressDuration = millis() - buttonPressTime;
        if (pressDuration >= 6000) {
          Serial.println("----- USER CONFIG INFO -----");
          if (currentMode == MODE_TIME) {
            Serial.println("Current Mode: TIME");
            Serial.printf("Schedule Count: %d\n", scheduleCount);
            for (int i = 0; i < scheduleCount; ++i) {
              Serial.printf("Schedule %d: ON %02d:%02d - OFF %02d:%02d\n", i + 1, onHours[i], onMinutes[i], offHours[i], offMinutes[i]);
            }
          } else if (currentMode == MODE_CYCLE) {
            Serial.println("Current Mode: CYCLE");
            Serial.printf("On Duration: %lus\n", onDuration);
            Serial.printf("Off Duration: %lus\n", offDuration);
          } else {
            Serial.println("Current Mode: NONE");
          }
          Serial.println("----------------------------");
        } else if (pressDuration >= 3000) {
          if (currentMode != MODE_NONE) {
            previousMode = currentMode;
            currentMode = MODE_NONE;
            Serial.println("[BTN] Long press >3s -> MODE_NONE");
          } else {
            currentMode = previousMode;
            Serial.println("[BTN] Long press >3s -> Restore previous mode");
          }
          saveAllSettingsToEEPROM();
        } else {
          if (currentMode == MODE_TIME) {
            currentMode = MODE_CYCLE;
            lastToggleTime = millis();
            Serial.println("[BTN] Short press -> Switch to CYCLE");
          } else if (currentMode == MODE_CYCLE) {
            currentMode = MODE_TIME;
            Serial.println("[BTN] Short press -> Switch to TIME");
          }
          saveAllSettingsToEEPROM();
        }
        buttonWasPressed = false;
      }
    }
  }

  lastReading = currentReading;
}

// {
//   "mode": "time",
//   "schedules": [

//     { "onHour": 11, "onMinute": 1, "offHour": 11, "offMinute": 2 },
//     { "onHour": 11, "onMinute": 3, "offHour": 11, "offMinute": 4 },
//     { "onHour": 11, "onMinute": 5, "offHour": 11, "offMinute": 6 },
//     { "onHour": 11, "onMinute": 7, "offHour": 11, "offMinute": 8 },
//     { "onHour": 11, "onMinute": 9, "offHour": 11, "offMinute": 10 },
//     { "onHour": 11, "onMinute": 11, "offHour": 11, "offMinute": 12 },
//     { "onHour": 11, "onMinute": 13, "offHour": 11, "offMinute": 14 },
//     { "onHour": 11, "onMinute": 15, "offHour": 11, "offMinute": 16 },
//     { "onHour": 11, "onMinute": 17, "offHour": 11, "offMinute": 18 },
//     { "onHour": 11, "onMinute": 19, "offHour": 11, "offMinute": 20 },
//     { "onHour": 11, "onMinute": 21, "offHour": 11, "offMinute": 22 },
//     { "onHour": 11, "onMinute": 23, "offHour": 11, "offMinute": 24 }
//   ]
// }

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);  // Chế độ WiFi station
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectWiFi();
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);  // Tắt relay ban đầu
  EEPROM.begin(512);

  pinMode(SW_USER_PIN, INPUT_PULLUP);

  while (getTime() < 1735664400) {
    Serial.println("Sync time ... ");
    delay(1000);
    wifiRetryCount++;
    if (wifiRetryCount >= retryConnection) {
      Serial.println("[WiFi] connection failed, unable to sync time, reset ESP...");
      delay(2000);
      ESP.restart();  // Reset ESP32
    }
  }
  Serial.println("----- EEPROM Address Info -----");
  Serial.print("MODE_ADDR: ");
  Serial.println(MODE_ADDR);
  Serial.print("Size of currentMode: ");
  Serial.println(sizeof(currentMode));

  Serial.print("ON_DURATION_ADDR: ");
  Serial.println(ON_DURATION_ADDR);
  Serial.print("Size of onDuration: ");
  Serial.println(sizeof(onDuration));

  Serial.print("OFF_DURATION_ADDR: ");
  Serial.println(OFF_DURATION_ADDR);
  Serial.print("Size of offDuration: ");
  Serial.println(sizeof(offDuration));
  Serial.println("--------------------------------");
  // loadAllSettingsFromEEPROM();
}

// Loop xử lý theo từng trạng thái
void loop() {
  switch (currentState) {
    case STATE_WIFI_CONNECTING:
      connectWiFi();
      delay(1000);
      break;

    case STATE_MQTT_CONNECTING:
      connectMQTT();
      delay(1000);
      break;

    case STATE_RUNNING:
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("wifi is disconnected, return wifi connection status");
        currentState = STATE_WIFI_CONNECTING;

        break;
      }

      if (!client.connected()) {
        Serial.println("MQTT is disconnected. return MQTT connection status");
        currentState = STATE_MQTT_CONNECTING;
        break;
      }
      handleRelayControl();
      client.loop();
      handleUserButton();

      break;
  }
}
