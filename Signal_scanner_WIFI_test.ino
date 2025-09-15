// ====== ESP32-WROOM-DA Wi-Fi Scanner (LCD + OLED + Buttons) ======
// - Matches your Wi-Fi scanner menu style
// - MODE button (pin 2): Rescan Wi-Fi
// - SCROLL button (pin 4): Scroll AP list (hold to auto-scroll)
// - LCD 20x4 shows details for the selected AP
// - OLED 128x64 shows bars for top-N strongest APs
//
// Libraries needed:
//   LiquidCrystal_I2C by Frank de Brabander (or compatible)
//   Adafruit GFX Library
//   Adafruit SSD1306
//
// I2C addresses assumed: LCD=0x27, OLED=0x3C

#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>
#include <algorithm>

// ---------------- Pins ----------------
#define MODE_BUTTON_PIN    2   // Rescan
#define SCROLL_BUTTON_PIN  4   // Scroll within results

// ---------------- LCD ----------------
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ---------------- OLED ----------------
#define OLED_RESET   -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------------- Scroll timings ----------------
const uint32_t scrollHoldTime = 500;  // ms to start auto-scroll
const uint32_t scrollInterval = 500;  // ms between auto-scroll steps

// ---------------- Data model ----------------
struct APInfo {
  String ssid;
  int32_t rssi;
  int32_t channel;
  wifi_auth_mode_t enc;
};
std::vector<APInfo> aps;

// Scroll state
uint8_t wifiPage = 0;
uint32_t wifiHoldStart = 0, wifiLastScroll = 0;

// ---------------- Utils ----------------
static const char* encToStr(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN:           return "OPEN";
    case WIFI_AUTH_WEP:            return "WEP";
    case WIFI_AUTH_WPA_PSK:        return "WPA";
    case WIFI_AUTH_WPA2_PSK:       return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:   return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE:return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK:       return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:  return "WPA2/3";
    default:                       return "?";
  }
}

static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) {
    String t = s;
    t.reserve(20);
    while (t.length() < 20) t += ' ';
    return t;
  }
  // trim with ellipsis
  return s.substring(0, 17) + "...";
}

// Debounced edge detect for buttons (LOW = pressed)
static bool buttonPressed(uint8_t pin) {
  static uint32_t lastT[40] = {0};
  static uint8_t  lastS[40] = {HIGH};
  uint32_t now = millis();
  uint8_t s = digitalRead(pin);
  if (s != lastS[pin] && (now - lastT[pin]) > 30) {
    lastT[pin] = now;
    lastS[pin] = s;
    if (s == LOW) return true;
  }
  return false;
}

// ---------------- Rendering ----------------
static void drawBarsOnOLED(const std::vector<APInfo>& v) {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  if (v.empty()) { oled.display(); return; }

  const int maxBars = 8;                   // show top N strongest
  const int n = (int)std::min((size_t)maxBars, v.size());
  const int w = SCREEN_WIDTH / n;
  const int topMargin = 12;                // room for indices
  const int usableHeight = SCREEN_HEIGHT - topMargin - 2;

  for (int i = 0; i < n; ++i) {
    int rssi = v[i].rssi;
    rssi = std::max(-100, std::min(0, rssi)); // clamp
    int bar = map(rssi, -100, 0, 0, usableHeight);
    int x = i * w;

    // index labels (1..n) at top
    oled.setCursor(x + (w / 2) - 3, 0);
    oled.print(i + 1);

    // bar from bottom up
    oled.fillRect(x + 1, SCREEN_HEIGHT - bar - 1, w - 3, bar, SSD1306_WHITE);
  }

  oled.display();
}

static void renderLCD(const std::vector<APInfo>& v, size_t idx) {
  lcd.setCursor(0, 0); lcd.print(padOrTrim20("Mode: WiFi Scan"));
  char buf[21];

  // Total APs
  snprintf(buf, sizeof(buf), "Total APs:%-9d", (int)v.size());
  lcd.setCursor(0, 1); lcd.print(padOrTrim20(String(buf)));

  if (v.empty()) {
    lcd.setCursor(0, 2); lcd.print(padOrTrim20("No networks found"));
    lcd.setCursor(0, 3); lcd.print(padOrTrim20(""));
    return;
  }

  const APInfo& a = v[idx];

  // Line 3: "N: SSID"
  String s1 = String(idx + 1) + ": " + (a.ssid.length() ? a.ssid : String("<hidden>"));
  lcd.setCursor(0, 2); lcd.print(padOrTrim20(s1));

  // Line 4: "-42dBm Ch11 WPA2"
  snprintf(buf, sizeof(buf), "%ddBm Ch%02ld %s", (int)a.rssi, (long)a.channel, encToStr(a.enc));
  lcd.setCursor(0, 3); lcd.print(padOrTrim20(String(buf)));
}

// ---------------- Wi-Fi scan ----------------
static void doScan() {
  // Header
  lcd.clear();
  renderLCD({}, 0);   // show "Mode: WiFi Scan" + "Total APs:0" temporarily
  oled.clearDisplay(); oled.display();

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true, true);
  delay(120);

  // Synchronous scan: include hidden; active scan with reasonable dwell
  int n = WiFi.scanNetworks(/*async*/false, /*show_hidden*/true, /*passive*/false, /*max_ms_per_chan*/120);

  aps.clear();
  if (n <= 0) {
    // Nothing found
    renderLCD(aps, 0);
    drawBarsOnOLED(aps);
    return;
  }

  aps.reserve(n);
  for (int i = 0; i < n; ++i) {
    APInfo a;
    a.ssid    = WiFi.SSID(i);
    a.rssi    = WiFi.RSSI(i);
    a.channel = WiFi.channel(i);
    a.enc     = WiFi.encryptionType(i);
    aps.push_back(a);
  }

  // Sort strongest first
  std::sort(aps.begin(), aps.end(), [](const APInfo& A, const APInfo& B){
    return A.rssi > B.rssi;
  });

  // Reset scroll state
  wifiPage = 0;
  wifiHoldStart = wifiLastScroll = 0;

  // First render
  renderLCD(aps, wifiPage);
  drawBarsOnOLED(aps);
}

// ---------------- Arduino setup/loop ----------------
void setup() {
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SCROLL_BUTTON_PIN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); oled.display();

  Serial.begin(115200);
  delay(100);

  doScan();
}

void loop() {
  // MODE = rescan
  if (buttonPressed(MODE_BUTTON_PIN)) {
    doScan();
  }

  // SCROLL handling (hold-to-scroll)
  if (digitalRead(SCROLL_BUTTON_PIN) == LOW) {
    if (!wifiHoldStart) {
      // on initial press, move one step immediately
      wifiHoldStart = millis();
      if (!aps.empty()) {
        wifiPage = (wifiPage + 1) % aps.size();
        renderLCD(aps, wifiPage);
        // OLED stays showing top N strongest overall
      }
    } else if (millis() - wifiHoldStart > scrollHoldTime &&
               millis() - wifiLastScroll > scrollInterval) {
      if (!aps.empty()) {
        wifiPage = (wifiPage + 1) % aps.size();
        renderLCD(aps, wifiPage);
      }
      wifiLastScroll = millis();
    }
  } else {
    wifiHoldStart = wifiLastScroll = 0;
  }

  // If nothing found, keep LCD/OLED idle
  delay(20);
}

