#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>      // https://github.com/tzapu/WiFiManager
#include <Preferences.h>      // ESP32 Flash storage
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <time.h>

// --- Pin Definitions ---
const int RED_LED_PIN   = 2;  // GPIO 2 (D2)
const int GREEN_LED_PIN = 4;  // GPIO 4 (D4)
const int SERVO_PIN     = 5;  // GPIO 5 (D5)
// OLED I2C: SDA -> GPIO 21, SCL -> GPIO 22

// --- OLED Display Settings (128x32) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C  // Standard 128x32 I2C address
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Servo & Scale Bounds ---
const int SERVO_MIN_ANGLE   = 0;     // 0 meters
const int SERVO_MAX_ANGLE   = 180;   // 6 meters
const float MAX_TIDE_METERS = 6.0f;

// --- Timing Intervals ---
const unsigned long REFRESH_API_INTERVAL   = 12UL * 60UL * 60UL * 1000UL; // 12 Hours
const unsigned long REFRESH_SERVO_INTERVAL = 5UL * 60UL * 1000UL;         // 5 Minutes

// --- Globals & Storage ---
Preferences preferences;
char stationId[10] = "0206";

struct TideEvent {
  time_t epochTime;
  int eventType; // 0 = High, 1 = Low
};

struct HeightOccurrence {
  time_t epochTime;
  float height;
};

TideEvent tideEvents[40];
int totalEvents = 0;

HeightOccurrence heightList[150];
int totalHeights = 0;

unsigned long lastApiFetch = 0;
unsigned long lastServoUpdate = 0;

Servo tideServo;
WiFiManager wm;

// --- Helper Functions ---

time_t parseISO8601(const char* dateStr) {
  struct tm t;
  memset(&t, 0, sizeof(struct tm));
  int year, month, day, hour, min, sec = 0;
  if (sscanf(dateStr, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &min, &sec) >= 5) {
    t.tm_year = year - 1900;
    t.tm_mon  = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min  = min;
    t.tm_sec  = sec;
    return mktime(&t);
  }
  return 0;
}

void syncTime() {
  Serial.print("Synchronizing UTC Time");
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println("\nUTC Time synchronized.");
}

void saveParamCallback() {
  WiFiManagerParameter* custom_station = wm.getParameters()[0];
  if (custom_station != nullptr) {
    strncpy(stationId, custom_station->getValue(), sizeof(stationId) - 1);
    stationId[sizeof(stationId) - 1] = '\0';

    preferences.begin("tide_config", false);
    preferences.putString("stationId", stationId);
    preferences.end();
    
    Serial.printf("New Station ID saved: %s\n", stationId);
  }
}

bool fetchTideData() {
  if (WiFi.status() != WL_CONNECTED) return false;

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://easytide.admiralty.co.uk/Home/GetPredictionData?stationId=" + String(stationId);

  Serial.printf("Fetching Admiralty Tide API (Station: %s)...\n", stationId);
  if (!http.begin(client, url)) {
    Serial.println("Unable to connect to HTTPS endpoint.");
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("HTTP GET failed, error code: %d\n", httpCode);
    http.end();
    return false;
  }

  StaticJsonDocument<300> filter;
  filter["tidalEventList"][0]["dateTime"] = true;
  filter["tidalEventList"][0]["eventType"] = true;
  filter["tidalHeightOccurrenceList"][0]["dateTime"] = true;
  filter["tidalHeightOccurrenceList"][0]["height"] = true;

  DynamicJsonDocument doc(24576);
  WiFiClient* stream = http.getStreamPtr();
  DeserializationError error = deserializeJson(doc, *stream, DeserializationOption::Filter(filter));
  http.end();

  if (error) {
    Serial.print("JSON parsing failed: ");
    Serial.println(error.c_str());
    return false;
  }

  // Parse Events
  JsonArray events = doc["tidalEventList"].as<JsonArray>();
  totalEvents = 0;
  for (JsonObject ev : events) {
    if (totalEvents >= 40) break;
    const char* dt = ev["dateTime"];
    if (dt && !ev["eventType"].isNull()) {
      time_t t = parseISO8601(dt);
      if (t > 0) {
        tideEvents[totalEvents].epochTime = t;
        tideEvents[totalEvents].eventType = ev["eventType"].as<int>();
        totalEvents++;
      }
    }
  }

  // Parse Heights
  JsonArray heights = doc["tidalHeightOccurrenceList"].as<JsonArray>();
  totalHeights = 0;
  for (JsonObject h : heights) {
    if (totalHeights >= 150) break;
    const char* dt = h["dateTime"];
    if (dt && !h["height"].isNull()) {
      time_t t = parseISO8601(dt);
      float val = h["height"].as<float>();
      if (t > 0) {
        heightList[totalHeights].epochTime = t;
        heightList[totalHeights].height = val;
        totalHeights++;
      }
    }
  }

  Serial.printf("Parsed %d tide events & %d height points.\n", totalEvents, totalHeights);
  return (totalEvents > 0 && totalHeights > 0);
}

void updateOLED(float currentHeight, bool isRising, time_t nextLowTime) {
  time_t now = time(nullptr);
  long secondsToLow = nextLowTime - now;

  int hours = 0;
  int minutes = 0;
  if (secondsToLow > 0) {
    hours = secondsToLow / 3600;
    minutes = (secondsToLow % 3600) / 60;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Water State & Current Height
  display.setTextSize(1);
  display.setCursor(0, 0);
  if (isRising) {
    display.printf("TIDE: RISING (%.2fm)", currentHeight);
  } else {
    display.printf("TIDE: FALLING(%.2fm)", currentHeight);
  }

  // Line 2: Separator Line
  display.drawFastHLine(0, 11, 128, SSD1306_WHITE);

  // Line 3: Countdown Header
  display.setCursor(0, 15);
  display.print("LOW TIDE IN:");

  // Line 4: Large Countdown Readout (e.g., "04h 12m")
  display.setTextSize(2);
  display.setCursor(0, 24);
  if (secondsToLow <= 0) {
    display.print("NOW");
  } else {
    display.printf("%02dh %02dm", hours, minutes);
  }

  display.display();
}

void updateTideSystem() {
  time_t now = time(nullptr);
  if (totalEvents == 0 || totalHeights == 0) return;

  // --- 1. Find Next Event & Determine State ---
  int nextIdx = -1;
  for (int i = 0; i < totalEvents; i++) {
    if (tideEvents[i].epochTime > now) {
      nextIdx = i;
      break;
    }
  }

  bool isRising = false;
  if (nextIdx > 0) {
    int nextType = tideEvents[nextIdx].eventType; // 0 = High, 1 = Low
    digitalWrite(GREEN_LED_PIN, (nextType == 1) ? HIGH : LOW);
    digitalWrite(RED_LED_PIN,   (nextType == 0) ? HIGH : LOW);

    // If next event is High Tide (0), tide is currently RISING
    isRising = (nextType == 0);
  }

  // --- 2. Find Next LOW Tide Event for Countdown ---
  time_t nextLowTime = 0;
  for (int i = 0; i < totalEvents; i++) {
    if (tideEvents[i].epochTime > now && tideEvents[i].eventType == 1) { // 1 = Low
      nextLowTime = tideEvents[i].epochTime;
      break;
    }
  }

  // --- 3. Interpolate Current Height ---
  int hIndex = -1;
  for (int i = 0; i < totalHeights - 1; i++) {
    if (heightList[i].epochTime <= now && heightList[i + 1].epochTime > now) {
      hIndex = i;
      break;
    }
  }

  float currentHeight = 0.0f;
  if (hIndex != -1) {
    time_t t1 = heightList[hIndex].epochTime;
    time_t t2 = heightList[hIndex + 1].epochTime;
    float h1  = heightList[hIndex].height;
    float h2  = heightList[hIndex + 1].height;

    double progress = (double)(now - t1) / (double)(t2 - t1);
    currentHeight = h1 + (float)(progress * (h2 - h1));
  } else {
    if (now < heightList[0].epochTime) currentHeight = heightList[0].height;
    else currentHeight = heightList[totalHeights - 1].height;
  }

  // --- 4. Drive Servo (0-6m clamped to 0-180 degrees) ---
  float rawHeight = currentHeight;
  currentHeight = constrain(currentHeight, 0.0f, MAX_TIDE_METERS);

  long heightInCm = (long)(currentHeight * 100.0f);
  long maxHeightInCm = (long)(MAX_TIDE_METERS * 100.0f);
  int servoAngle = map(heightInCm, 0L, maxHeightInCm, (long)SERVO_MIN_ANGLE, (long)SERVO_MAX_ANGLE);
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  tideServo.write(servoAngle);

  // --- 5. Update 128x32 OLED Display ---
  updateOLED(rawHeight, isRising, nextLowTime);

  Serial.printf("Height: %.2fm | State: %s | Servo Angle: %d°\n",
                rawHeight, isRising ? "RISING" : "FALLING", servoAngle);
}

// --- Setup & Loop ---

void setup() {
  Serial.begin(115200);

  // Initialize Pins
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);

  // Initialize OLED (Wire uses default I2C pins GPIO 21 & GPIO 22 on ESP32)
  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("SSD1306 OLED allocation failed");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("Tide Clock Starting...");
    display.display();
  }

  // Initialize Servo
  tideServo.attach(SERVO_PIN, 500, 2400);
  tideServo.write(SERVO_MIN_ANGLE);
  delay(300);

  // Load Saved Preferences
  preferences.begin("tide_config", true);
  String savedStation = preferences.getString("stationId", "0206");
  strncpy(stationId, savedStation.c_str(), sizeof(stationId) - 1);
  preferences.end();

  // Setup WiFiManager
  WiFiManagerParameter custom_station_id("station_id", "Admiralty Station ID", stationId, 10);
  wm.addParameter(&custom_station_id);
  wm.setSaveParamsCallback(saveParamCallback);

  if (!wm.autoConnect("TideGauge-Setup")) {
    Serial.println("WiFi configuration failed. Restarting...");
    ESP.restart();
  }

  syncTime();

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  if (fetchTideData()) {
    lastApiFetch = millis();
    updateTideSystem();
    lastServoUpdate = millis();
  }
}

void loop() {
  wm.process();

  unsigned long currentMillis = millis();

  // Refresh predictions every 12 hours
  if (currentMillis - lastApiFetch >= REFRESH_API_INTERVAL) {
    if (fetchTideData()) {
      lastApiFetch = currentMillis;
    }
  }

  // Update Servo, LEDs, and OLED every 5 minutes
  if (currentMillis - lastServoUpdate >= REFRESH_SERVO_INTERVAL) {
    updateTideSystem();
    lastServoUpdate = currentMillis;
  }
}
