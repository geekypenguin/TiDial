#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>            // http://tidial.local access
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

// --- OLED Display Settings (128x32) ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
#define OLED_ADDRESS  0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- Servo & Scale Bounds ---
const int SERVO_MIN_ANGLE   = 0;     // Low tide end of gauge
const int SERVO_MAX_ANGLE   = 180;   // High tide end of gauge

// --- Timing Intervals ---
const unsigned long REFRESH_API_INTERVAL    = 12UL * 60UL * 60UL * 1000UL; // 12 Hours
const unsigned long RETRY_API_INTERVAL      = 10UL * 1000UL;               // 10 Seconds
const unsigned long REFRESH_SYSTEM_INTERVAL = 1UL * 60UL * 1000UL;        // 1 Minute
const unsigned long REFRESH_SERVO_INTERVAL  = 5UL * 60UL * 1000UL;       // 5 Minutes

// --- Station Table & Name Lookup ---
struct StationMapping {
  const char* id;
  const char* name;
};

const StationMapping STATION_TABLE[] = {
  {"0206", "Amble"},
  {"0205", "Coquet Is."},
  {"0204", "Blyth"},
  {"0203", "Tyne"},
  {"0202", "N Shields"},
  {"0207", "Seahouses"},
  {"0208", "Holy Is."},
  {"0190", "Sunderland"},
  {"0209", "Berwick"}
};
const int NUM_STATIONS = sizeof(STATION_TABLE) / sizeof(STATION_TABLE[0]);

const char* getStationName(const char* id) {
  for (int i = 0; i < NUM_STATIONS; i++) {
    if (strcmp(STATION_TABLE[i].id, id) == 0) {
      return STATION_TABLE[i].name;
    }
  }
  return id; // Fall back to ID if not in table
}

// --- Crisp Geometric 32x32 Bitmaps (Shifted +2px Right) ---

// Rising: Bold Upward Arrow over Sea Line
const unsigned char PROGMEM icon_rising[] = {
  0x00, 0x03, 0xc0, 0x00,  0x00, 0x07, 0xe0, 0x00,  
  0x00, 0x0f, 0xf0, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x3f, 0xfc, 0x00,  0x00, 0x7f, 0xfe, 0x00,  
  0x00, 0xff, 0xff, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00   
};

// Falling: Bold Downward Arrow over Sea Line
const unsigned char PROGMEM icon_falling[] = {
  0x00, 0x00, 0x00, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x1f, 0xf8, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0xff, 0xff, 0x00,  0x00, 0x7f, 0xfe, 0x00,  
  0x00, 0x3f, 0xfc, 0x00,  0x00, 0x1f, 0xf8, 0x00,  
  0x00, 0x0f, 0xf0, 0x00,  0x00, 0x07, 0xe0, 0x00,  
  0x00, 0x03, 0xc0, 0x00,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00   
};

// Slack: Triple Sea Line
const unsigned char PROGMEM icon_slack[] = {
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x70, 0x07, 0x00,  0x00, 0xf8, 0x0f, 0x80,  
  0x01, 0xdc, 0x1d, 0xc0,  0x03, 0x8e, 0x38, 0xe0,  
  0x07, 0x07, 0x70, 0x70,  0x0e, 0x03, 0xe0, 0x38,  
  0x1c, 0x01, 0xc0, 0x1c,  0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00   
};

// --- Globals & Storage ---
Preferences preferences;
char stationId[10] = "0206";

struct TideEvent {
  time_t epochTime;
  int eventType; // 0 = High, 1 = Low
  float height;  // Exact peak/trough height from API
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
unsigned long lastSystemUpdate = 0;
bool apiSuccess = false;

Servo tideServo;
WiFiManager wm;
WiFiManagerParameter custom_station_id("station_id", "Manual Station ID", stationId, 10);

const char* custom_select_html = 
  "<br/><label for='stn_select'>Select Station</label>"
  "<select id='stn_select' onchange='document.getElementById(\"station_id\").value = this.value;'>"
    "<option value=''>-- Select Preset --</option>"
    "<option value='0206'>0206 - Amble</option>"
    "<option value='0205'>0205 - Coquet Island.</option>"
    "<option value='0204'>0204 - Blyth</option>"
    "<option value='0203'>0203 - Newcastle Upon Tyne</option>"
    "<option value='0202'>0202 - North Shields</option>"
    "<option value='0207'>0207 - Seahouses</option>"
    "<option value='0208'>0208 - Holy Island</option>"
    "<option value='0190'>0190 - Sunderland</option>"
    "<option value='0209'>0209 - Berwick</option>"
  "</select><br/>";

WiFiManagerParameter custom_station_select(custom_select_html);

// --- Helpers ---

void showStatus(const char* line1, const char* line2 = "") {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(line1);
  if (strlen(line2) > 0) {
    display.setCursor(0, 14);
    display.println(line2);
  }
  display.display();
}

void draw7SegmentDigit(int x, int y, char c, int w = 10, int h = 14) {
  if (c < '0' || c > '9') return;
  int digit = c - '0';

  const uint8_t masks[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
  };

  uint8_t m = masks[digit];
  int midY = y + h / 2;

  if (m & (1 << 0)) display.drawFastHLine(x + 1, y, w - 2, SSD1306_WHITE);
  if (m & (1 << 1)) display.drawFastVLine(x + w - 1, y + 1, h / 2 - 1, SSD1306_WHITE);
  if (m & (1 << 2)) display.drawFastVLine(x + w - 1, midY + 1, h / 2 - 1, SSD1306_WHITE);
  if (m & (1 << 3)) display.drawFastHLine(x + 1, y + h - 1, w - 2, SSD1306_WHITE);
  if (m & (1 << 4)) display.drawFastVLine(x, midY + 1, h / 2 - 1, SSD1306_WHITE);
  if (m & (1 << 5)) display.drawFastVLine(x, y + 1, h / 2 - 1, SSD1306_WHITE);
  if (m & (1 << 6)) display.drawFastHLine(x + 1, midY, w - 2, SSD1306_WHITE);
}

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
    t.tm_sec = sec;
    return mktime(&t);
  }
  return 0;
}

bool syncTimeNonBlocking() {
  if (WiFi.status() != WL_CONNECTED) return false;
  
  static bool ntpConfigured = false;
  if (!ntpConfigured) {
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    ntpConfigured = true;
  }
  
  time_t now = time(nullptr);
  return (now > 8 * 3600 * 2);
}

void saveParamCallback() {
  const char* val = custom_station_id.getValue();
  if (val != nullptr && strlen(val) > 0) {
    strncpy(stationId, val, sizeof(stationId) - 1);
    stationId[sizeof(stationId) - 1] = '\0';

    preferences.begin("tide_config", false);
    preferences.putString("stationId", stationId);
    preferences.end();
    
    Serial.printf("[CONFIG] Station ID saved: %s (%s)\n", stationId, getStationName(stationId));
  }
}

bool fetchTideData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[ERROR] WiFi disconnected.");
    return false;
  }

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  String url = "https://easytide.admiralty.co.uk/Home/GetPredictionData?stationId=" + String(stationId);

  Serial.println("\n----------------------------------------");
  Serial.printf("[HTTP] Fetching API for Station: %s (%s)\n", stationId, getStationName(stationId));

  if (!http.begin(client, url)) {
    Serial.println("[ERROR] http.begin() failed!");
    return false;
  }

  http.setUserAgent("Mozilla/5.0 (Windows NT 10.0; Win64; x64)");
  http.addHeader("Host", "easytide.admiralty.co.uk");
  http.addHeader("Accept", "application/json, text/javascript, */*; q=0.01");
  http.addHeader("X-Requested-With", "XMLHttpRequest");
  http.setTimeout(12000);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[ERROR] HTTP failed code: %d\n", httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  if (payload.length() < 200) {
    Serial.println("[ERROR] Payload too small!");
    return false;
  }

  StaticJsonDocument<256> filter;
  filter["tidalEventList"][0]["dateTime"] = true;
  filter["tidalEventList"][0]["eventType"] = true;
  filter["tidalEventList"][0]["height"] = true;
  filter["tidalHeightOccurrenceList"][0]["dateTime"] = true;
  filter["tidalHeightOccurrenceList"][0]["height"] = true;

  JsonDocument doc; 
  DeserializationError error = deserializeJson(doc, payload, DeserializationOption::Filter(filter));

  if (error) {
    Serial.printf("[ERROR] JSON parse failed: %s\n", error.c_str());
    return false;
  }

  JsonArray events = doc["tidalEventList"].as<JsonArray>();
  totalEvents = 0;
  for (JsonObject ev : events) {
    if (totalEvents >= 40) break;
    const char* dt = ev["dateTime"];
    if (dt && !ev["eventType"].isNull() && !ev["height"].isNull()) {
      time_t t = parseISO8601(dt);
      if (t > 0) {
        tideEvents[totalEvents].epochTime = t;
        tideEvents[totalEvents].eventType = ev["eventType"].as<int>();
        tideEvents[totalEvents].height = ev["height"].as<float>();
        totalEvents++;
      }
    }
  }

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

  Serial.printf("[SUCCESS] Parsed %d events & %d height points.\n", totalEvents, totalHeights);
  Serial.println("----------------------------------------\n");

  return (totalEvents > 0 && totalHeights > 0);
}

void updateOLED(float currentHeight, bool isRising, bool isSlack, time_t nextLowTime) {
  time_t now = time(nullptr);
  long secondsToLow = nextLowTime - now;

  int hours = (secondsToLow > 0) ? (secondsToLow / 3600) : 0;
  int minutes = (secondsToLow > 0) ? ((secondsToLow % 3600) / 60) : 0;

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Line 1: Station Name + Height
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%.7s %.2fm", getStationName(stationId), currentHeight);

  // Line 2: Label
  display.setCursor(0, 10);
  display.print("Next Low Tide:");

  // Line 3: 7-Segment Countdown
  if (secondsToLow <= 0) {
    display.setTextSize(2);
    display.setCursor(0, 18);
    display.print("NOW");
  } else {
    char buf[10];
    snprintf(buf, sizeof(buf), "%02d", hours);

    int startX = 0;
    int startY = 18;
    int digitW = 9;
    int digitH = 13;

    draw7SegmentDigit(startX, startY, buf[0], digitW, digitH);
    draw7SegmentDigit(startX + 11, startY, buf[1], digitW, digitH);

    display.setTextSize(1);
    display.setCursor(startX + 22, startY + 5);
    display.print("h");

    snprintf(buf, sizeof(buf), "%02d", minutes);
    draw7SegmentDigit(startX + 30, startY, buf[0], digitW, digitH);
    draw7SegmentDigit(startX + 41, startY, buf[1], digitW, digitH);

    display.setCursor(startX + 52, startY + 5);
    display.print("m");
  }

  // Right-aligned Icon
  int iconX = 96;
  int iconY = 0;

  if (isSlack) {
    display.drawBitmap(iconX, iconY, icon_slack, 32, 32, SSD1306_WHITE);
  } else if (isRising) {
    display.drawBitmap(iconX, iconY, icon_rising, 32, 32, SSD1306_WHITE);
  } else {
    display.drawBitmap(iconX, iconY, icon_falling, 32, 32, SSD1306_WHITE);
  }

  display.display();
}

void updateTideSystem(bool updateServo = true) {
  time_t now = time(nullptr);
  if (totalEvents == 0 || totalHeights == 0) return;

  time_t nextLowTime = 0;
  for (int i = 0; i < totalEvents; i++) {
    if (tideEvents[i].epochTime > now && tideEvents[i].eventType == 1) {
      nextLowTime = tideEvents[i].epochTime;
      break;
    }
  }

  auto getInterpolatedHeight = [](time_t targetTime) -> float {
    int hIndex = -1;
    for (int i = 0; i < totalHeights - 1; i++) {
      if (heightList[i].epochTime <= targetTime && heightList[i + 1].epochTime > targetTime) {
        hIndex = i;
        break;
      }
    }

    if (hIndex != -1) {
      time_t t1 = heightList[hIndex].epochTime;
      time_t t2 = heightList[hIndex + 1].epochTime;
      float h1  = heightList[hIndex].height;
      float h2  = heightList[hIndex + 1].height;

      double progress = (double)(targetTime - t1) / (double)(t2 - t1);
      return h1 + (float)(progress * (h2 - h1));
    } else {
      if (targetTime < heightList[0].epochTime) return heightList[0].height;
      return heightList[totalHeights - 1].height;
    }
  };

float currentHeight = getInterpolatedHeight(now);

  int prevEventIdx = -1;
  int nextEventIdx = -1;

  for (int i = 0; i < totalEvents; i++) {
    if (tideEvents[i].epochTime <= now) {
      prevEventIdx = i;
    } else if (tideEvents[i].epochTime > now && nextEventIdx == -1) {
      nextEventIdx = i;
      break;
    }
  }

  float minHeight = 0.0f;
  float maxHeight = 6.0f;
  bool isRising = false;

  if (prevEventIdx != -1 && nextEventIdx != -1) {
    // 0 = High Tide Event, 1 = Low Tide Event
    isRising = (tideEvents[prevEventIdx].eventType == 1 && tideEvents[nextEventIdx].eventType == 0);

    if (isRising) {
      minHeight = tideEvents[prevEventIdx].height; // Past Low
      maxHeight = tideEvents[nextEventIdx].height; // Next High
    } else {
      minHeight = tideEvents[nextEventIdx].height; // Next Low
      maxHeight = tideEvents[prevEventIdx].height; // Past High
    }
  } else {
    // Fallback if between events
    float futureHeight = getInterpolatedHeight(now + 60);
    isRising = (futureHeight > currentHeight);
  }

  if (maxHeight <= minHeight) {
    maxHeight = minHeight + 0.1f;
  }

  float rawHeight = currentHeight;
  currentHeight = constrain(currentHeight, minHeight, maxHeight);
  float fraction = (currentHeight - minHeight) / (maxHeight - minHeight);

  int servoAngle = (int)(SERVO_MAX_ANGLE - (fraction * (SERVO_MAX_ANGLE - SERVO_MIN_ANGLE)));
  servoAngle = constrain(servoAngle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

  // LED & Slack Logic
  float nearestPeakHeight = -1.0f;
  long minTimeDiff = 0x7FFFFFFF;

  for (int i = 0; i < totalEvents; i++) {
    long diff = abs((long)(tideEvents[i].epochTime - now));
    if (diff < minTimeDiff) {
      minTimeDiff = diff;
      nearestPeakHeight = tideEvents[i].height;
    }
  }

  bool isSlack = false;
  if (nearestPeakHeight >= 0.0f) {
    isSlack = (fabs(currentHeight - nearestPeakHeight) <= 0.1f);
  }

  if (isSlack) {
    digitalWrite(GREEN_LED_PIN, HIGH);
    digitalWrite(RED_LED_PIN,   HIGH);
  } else {
    digitalWrite(GREEN_LED_PIN, !isRising ? HIGH : LOW);
    digitalWrite(RED_LED_PIN,   isRising ? HIGH : LOW);
  }

  if (updateServo) {
  tideServo.attach(SERVO_PIN, 500, 2500);
  tideServo.write(servoAngle);
  delay(500); // Allow physical movement
  tideServo.detach();
}
  updateOLED(rawHeight, isRising, isSlack, nextLowTime);

  Serial.printf("Height: %.2fm (Range: %.2fm-%.2fm) | State: %s | Servo: %d°\n",
                rawHeight, minHeight, maxHeight, isSlack ? "SLACK" : (isRising ? "RISING" : "FALLING"), servoAngle);
}

// --- Setup & Loop ---

void setup() {
  Serial.begin(115200);

  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, HIGH);

  Wire.begin(21, 22);
  Wire.setClock(100000);
  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    showStatus("Connecting WiFi...");
  }

  preferences.begin("tide_config", true);
  String savedStation = preferences.getString("stationId", "0206");
  strncpy(stationId, savedStation.c_str(), sizeof(stationId) - 1);
  preferences.end();

  WiFi.setHostname("tidial"); 
  WiFi.mode(WIFI_STA);

  wm.addParameter(&custom_station_select);
  wm.addParameter(&custom_station_id);
  wm.setSaveParamsCallback(saveParamCallback);
  
  std::vector<const char*> menuItems = {"wifi", "param", "info", "restart", "exit"};
  wm.setMenu(menuItems); 

  wm.setConfigPortalBlocking(false);
  wm.setCaptivePortalEnable(true);

  if (!wm.autoConnect("TiDial-Setup")) {
    Serial.println("[WiFi] Portal active. Access at http://192.168.4.1 or http://tidial.local");
    showStatus("Connect to:", "TiDial-Setup");
    wm.startWebPortal();
  } else {
    Serial.println("[WiFi] Connected to network!");
    showStatus("WiFi Connected", "Syncing Time...");
    wm.startWebPortal();
  }

  if (MDNS.begin("tidial")) {
    Serial.println("[mDNS] Responder started: http://tidial.local");
  }

  tideServo.attach(SERVO_PIN, 500, 2500);
  tideServo.write(SERVO_MAX_ANGLE);
  delay(500);
  tideServo.write(SERVO_MIN_ANGLE);
  delay(500);
  tideServo.detach();

  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
}

void loop() {
  wm.process();

  unsigned long currentMillis = millis();

  static bool timeSynced = false;
  if (WiFi.status() == WL_CONNECTED && !timeSynced) {
    if (syncTimeNonBlocking()) {
      timeSynced = true;
      Serial.println("[Time] UTC Time synchronized successfully.");
      showStatus("Time Synced", "Fetching API...");
      lastApiFetch = millis() - REFRESH_API_INTERVAL; // Force immediate fetch in loop
    }
  }

  static bool wasConnected = false;
  if (WiFi.status() == WL_CONNECTED && !wasConnected) {
    wasConnected = true;
    showStatus("WiFi Connected", "Syncing Time...");
  }

  unsigned long requiredInterval = apiSuccess ? REFRESH_API_INTERVAL : RETRY_API_INTERVAL;
  if (WiFi.status() == WL_CONNECTED && (currentMillis - lastApiFetch >= requiredInterval)) {
    if (!apiSuccess && timeSynced) {
      showStatus("Fetching API...");
    }
    
    apiSuccess = fetchTideData();
    lastApiFetch = currentMillis;

    if (apiSuccess) {
      updateTideSystem();
      lastSystemUpdate = currentMillis;
    } else {
      showStatus("API Fetch Failed", "Retrying...");
    }
  }

  static unsigned long lastServoUpdate = 0;
    if (apiSuccess && (currentMillis - lastSystemUpdate >= REFRESH_SYSTEM_INTERVAL)) {
      bool moveServo = (currentMillis - lastServoUpdate >= REFRESH_SERVO_INTERVAL);
      updateTideSystem(moveServo);
      lastSystemUpdate = currentMillis;
    if (moveServo) lastServoUpdate = currentMillis;
  }
}