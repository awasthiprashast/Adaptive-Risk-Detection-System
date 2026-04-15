/*************************************************
 * ADRS — ESP32
 * LittleFS Web Dashboard  ·  v3.0
 *
 * Static files served from /data via LittleFS.
 * Live sensor data exposed on GET /api  (JSON).
 * Actions: POST /silence  |  POST /reset
 *************************************************/

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ── WiFi ──────────────────────────────────────────────────────
const char* ssid     = "banana";
const char* password = "12345678";

WebServer server(80);

// ── Pin config ─────────────────────────────────────────────────
#define TRIG_PIN    5
#define ECHO_PIN    18
#define PIR_PIN     19
#define REED_PIN    13
#define VIB_PIN     23
#define MQ135_PIN   34
#define DHT_PIN     4
#define BUZZER_PIN  27

// ── OLED ───────────────────────────────────────────────────────
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── DHT ────────────────────────────────────────────────────────
#define DHTTYPE DHT22
DHT dht(DHT_PIN, DHTTYPE);

// ── Thresholds ─────────────────────────────────────────────────
#define DIST_NEAR    30
#define DIST_DANGER  15
#define AIR_WARN_MIN 100
#define AIR_CRIT_MIN 200
#define FIRE_TEMP    40

// ── Globals ────────────────────────────────────────────────────
float distanceCM = 400;
float temperature = 0;
int   mqBaseline = 0;
int   mqValue    = 0;

bool pirState          = false;
bool doorClosed        = true;
bool vibrationDetected = false;
bool alarmSilenced     = false;

unsigned long lastMotionTime = 0;
int           vibHits        = 0;
unsigned long vibWindow      = 0;
unsigned long buzzTimer      = 0;
bool          buzzState      = false;

// ── Forward declarations ───────────────────────────────────────
void readUltrasonic();
void readPIR();
void readVibration();
void readReed();
void readEnvironment();
void updateOLED();
void handleBuzzer();


// ═══════════════════════════════════════════════════════════════
//  RISK & ALERT HELPERS
// ═══════════════════════════════════════════════════════════════
String getRisk()
{
  int d = mqValue - mqBaseline;
  if (d > AIR_CRIT_MIN && temperature > FIRE_TEMP)  return "CRITICAL";
  if (!doorClosed && vibrationDetected)              return "HIGH";
  if (distanceCM < DIST_NEAR || d > AIR_WARN_MIN)   return "MEDIUM";
  return "LOW";
}

String getAlertReason()
{
  int d = mqValue - mqBaseline;
  if (d > AIR_CRIT_MIN && temperature > FIRE_TEMP)  return "FIRE / GAS CRITICAL";
  if (!doorClosed && vibrationDetected)              return "FORCED ENTRY";
  if (distanceCM < DIST_DANGER)                      return "VERY CLOSE OBJECT";
  if (distanceCM < DIST_NEAR)                        return "OBJECT NEAR";
  if (d > AIR_WARN_MIN)                              return "AIR QUALITY WARNING";
  return "ALL CLEAR";
}


// ═══════════════════════════════════════════════════════════════
//  HANDLE ROOT  —  serve index.html from LittleFS
// ═══════════════════════════════════════════════════════════════
void handleRoot()
{
  File f = LittleFS.open("/index.html", "r");
  if (!f) {
    server.send(500, "text/plain", "index.html not found in LittleFS");
    return;
  }
  server.streamFile(f, "text/html");
  f.close();
}


// ═══════════════════════════════════════════════════════════════
//  HANDLE /api  —  JSON snapshot of all sensor data
//
//  Called by the dashboard every 2 s via fetch().
//  Returns a single compact JSON object; no HTML involved.
// ═══════════════════════════════════════════════════════════════
void handleAPI()
{
  int    airDelta  = mqValue - mqBaseline;
  String riskVal   = getRisk();
  String alertVal  = getAlertReason();

  // Add CORS header so the page can call this even during dev
  server.sendHeader("Access-Control-Allow-Origin", "*");

  String json = "{";
  json += "\"distance\":"   + String(distanceCM, 1)          + ",";
  json += "\"pir\":"        + String(pirState ? "true" : "false") + ",";
  json += "\"door\":"       + String(doorClosed ? "true" : "false") + ",";
  json += "\"vibration\":"  + String(vibrationDetected ? "true" : "false") + ",";
  json += "\"airDelta\":"   + String(airDelta)               + ",";
  json += "\"temperature\":" + String(temperature, 1)        + ",";
  json += "\"risk\":\""     + riskVal  + "\","               ;
  json += "\"alert\":\""    + alertVal + "\""                ;
  json += "}";

  server.send(200, "application/json", json);
}


// ═══════════════════════════════════════════════════════════════
//  HANDLE /silence  —  stop buzzer until next trigger cycle
// ═══════════════════════════════════════════════════════════════
void handleSilence()
{
  alarmSilenced = true;
  server.send(200, "text/plain", "OK");
}


// ═══════════════════════════════════════════════════════════════
//  HANDLE /reset  —  recalibrate MQ135 air baseline
// ═══════════════════════════════════════════════════════════════
void handleReset()
{
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(MQ135_PIN);
    delay(50);
  }
  mqBaseline    = sum / 20;
  alarmSilenced = false;
  server.send(200, "text/plain", "OK");
}


// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);

  pinMode(TRIG_PIN,   OUTPUT);
  pinMode(ECHO_PIN,   INPUT);
  pinMode(PIR_PIN,    INPUT);
  pinMode(REED_PIN,   INPUT_PULLUP);
  pinMode(VIB_PIN,    INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  dht.begin();

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { while (1); }
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  // LittleFS init
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed — check /data upload");
  } else {
    Serial.println("LittleFS mounted OK");
  }

  // MQ135 warm-up + baseline
  display.clearDisplay();
  display.println("MQ135 WARM-UP...");
  display.display();
  delay(10000);

  long sum = 0;
  for (int i = 0; i < 40; i++) { sum += analogRead(MQ135_PIN); delay(100); }
  mqBaseline = sum / 40;

  display.clearDisplay();
  display.println("SYSTEM READY");
  display.display();
  delay(1500);

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(300);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nCONNECTED!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    server.on("/",        HTTP_GET,  handleRoot);
    server.on("/api",     HTTP_GET,  handleAPI);
    server.on("/silence", HTTP_POST, handleSilence);
    server.on("/reset",   HTTP_POST, handleReset);
    server.begin();
  } else {
    Serial.println("\nWiFi failed — restarting");
    ESP.restart();
  }
}


// ═══════════════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════════════
void loop()
{
  readUltrasonic();
  readPIR();
  readVibration();
  readReed();
  readEnvironment();
  updateOLED();
  handleBuzzer();
  server.handleClient();
  delay(120);
}


// ═══════════════════════════════════════════════════════════════
//  SENSORS
// ═══════════════════════════════════════════════════════════════
void readUltrasonic()
{
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(12);
  digitalWrite(TRIG_PIN, LOW);

  long d = pulseIn(ECHO_PIN, HIGH, 35000);
  if (d > 100 && d < 30000)
    distanceCM = d * 0.034 / 2.0;
}

void readPIR()
{
  if (digitalRead(PIR_PIN)) {
    pirState       = true;
    lastMotionTime = millis();
  }
  if (millis() - lastMotionTime > 1500) pirState = false;
}

void readVibration()
{
  if (digitalRead(VIB_PIN)) {
    if (vibHits == 0) vibWindow = millis();
    vibHits++;
  }
  if (millis() - vibWindow > 1500) {
    vibrationDetected = (vibHits >= 3);
    vibHits = 0;
  }
}

void readReed()
{
  doorClosed = (digitalRead(REED_PIN) == LOW);
}

void readEnvironment()
{
  mqValue = analogRead(MQ135_PIN);
  float t = dht.readTemperature();
  if (!isnan(t)) temperature = t;
}


// ═══════════════════════════════════════════════════════════════
//  OLED
// ═══════════════════════════════════════════════════════════════
void updateOLED()
{
  int airDelta = mqValue - mqBaseline;

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Dist:"); display.println(distanceCM, 0);
  display.print("PIR: "); display.println(pirState ? "MOTION" : "CLEAR");
  display.print("Air: "); display.println(airDelta);
  display.print("Temp:"); display.println(temperature, 1);
  display.print("Door:"); display.println(doorClosed ? "CLOSED" : "OPEN");
  display.print("Risk:"); display.println(getRisk());
  display.display();
}


// ═══════════════════════════════════════════════════════════════
//  BUZZER
// ═══════════════════════════════════════════════════════════════
void handleBuzzer()
{
  if (alarmSilenced) {
    digitalWrite(BUZZER_PIN, LOW);
    if (getRisk() == "LOW") alarmSilenced = false;
    return;
  }

  int  airDelta = mqValue - mqBaseline;
  bool alert    = false;
  int  interval = 1000;

  if      (airDelta > AIR_CRIT_MIN && temperature > FIRE_TEMP) { alert = true; interval = 120; }
  else if (!doorClosed && vibrationDetected)                    { alert = true; interval = 200; }
  else if (distanceCM < DIST_DANGER)                            { alert = true; interval = 180; }
  else if (distanceCM < DIST_NEAR)                              { alert = true; interval = 400; }
  else if (airDelta > AIR_WARN_MIN)                             { alert = true; interval = 600; }

  if (alert && millis() - buzzTimer > (unsigned long)interval) {
    buzzTimer = millis();
    buzzState = !buzzState;
    digitalWrite(BUZZER_PIN, buzzState);
  }
  if (!alert) digitalWrite(BUZZER_PIN, LOW);
}
