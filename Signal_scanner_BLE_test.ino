// ====== ESP32-WROOM-DA BLE Scanner (Classic Arduino BLE lib, fixed) ======
// - Uses ESP32 BLE Arduino (Kolban) library
// - MODE button (pin 2): Rescan (~6s blocking)
// - SCROLL button (pin 4): Browse results (hold to auto-scroll)
// - LCD 20x4 (I2C 0x27) shows one device at a time
// - OLED SSD1306 128x64 (I2C 0x3C) shows bars for top-8 by RSSI

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <vector>
#include <algorithm>

// ---------------- Pins ----------------
#define MODE_BUTTON_PIN    27   // Rescan
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

// ---------------- Scan params ----------------
int scanTime = 6; // seconds (like example)
BLEScan* pBLEScan = nullptr;

// ---------------- Data model ----------------
struct DevInfo {
  String name;
  String addr;
  int rssi;
  bool connectable;  // heuristic (true if name or service UUID present)
};
std::vector<DevInfo> devs;

// Scroll state
uint8_t blePage = 0;
uint32_t bleHoldStart = 0, bleLastScroll = 0;

// ---------------- Utils ----------------
static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) {
    String t = s;
    t.reserve(20);
    while (t.length() < 20) t += ' ';
    return t;
  }
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

static String shortAddrTail(const String& mac) {
  // mac like "aa:bb:cc:dd:ee:ff" -> take last 5 chars "ee:ff"
  if (mac.length() >= 5) return mac.substring(mac.length() - 5);
  return mac;
}

// Upsert device (by MAC); keep strongest RSSI and latest name/connectable
static void upsertDevice(const DevInfo& d) {
  for (auto &x : devs) {
    if (x.addr == d.addr) {
      if (d.rssi > x.rssi) x.rssi = d.rssi;
      if (d.name.length()) x.name = d.name;
      x.connectable = d.connectable;
      return;
    }
  }
  devs.push_back(d);
}

// ---------------- Rendering ----------------
static void drawBarsOnOLED(const std::vector<DevInfo>& v) {
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

static void renderLCD(const std::vector<DevInfo>& v, size_t idx) {
  lcd.setCursor(0, 0); lcd.print(padOrTrim20("Mode: BLE Scan"));
  char buf[21];

  // Total devices
  snprintf(buf, sizeof(buf), "Total Dev:%-8d", (int)v.size());
  lcd.setCursor(0, 1); lcd.print(padOrTrim20(String(buf)));

  if (v.empty()) {
    lcd.setCursor(0, 2); lcd.print(padOrTrim20("No devices found"));
    lcd.setCursor(0, 3); lcd.print(padOrTrim20(""));
    return;
  }

  const DevInfo& d = v[idx];

  // Line 3: "N: name" (or <noname>)
  String dispName = d.name.length() ? d.name : String("<noname>");
  String s1 = String(idx + 1) + ": " + dispName;
  lcd.setCursor(0, 2); lcd.print(padOrTrim20(s1));

  // Line 4: "-55dBm CON ee:ff"
  String tail = shortAddrTail(d.addr);
  const char* con = d.connectable ? "CON" : "NCN";
  snprintf(buf, sizeof(buf), "%ddBm %s %s", d.rssi, con, tail.c_str());
  lcd.setCursor(0, 3); lcd.print(padOrTrim20(String(buf)));
}

// ---------------- Callbacks (classic BLE lib) ----------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    DevInfo d;
    d.name = advertisedDevice.haveName() ? String(advertisedDevice.getName().c_str()) : String("");
    d.addr = advertisedDevice.getAddress().toString().c_str();
    d.rssi = advertisedDevice.getRSSI();
    // Heuristic: assume "connectable" if we have a name or at least one service UUID
    bool hasSvc = advertisedDevice.haveServiceUUID();
    d.connectable = hasSvc || d.name.length() > 0;
    upsertDevice(d);
  }
};

// ---------------- BLE scan (blocking like your example) ----------------
static void doScan() {
  // UI header
  lcd.clear();
  renderLCD({}, 0);   // show "Mode: BLE Scan" + "Total Dev:0"
  oled.clearDisplay(); oled.display();

  // Init BLE once
  static bool inited = false;
  if (!inited) {
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), /*wantDuplicates=*/true);
    pBLEScan->setActiveScan(true);   // like example
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);         // <= interval
    inited = true;
  }

  // Clear previous list
  devs.clear();

  // Start a blocking scan (returns BLEScanResults*)
  BLEScanResults* foundDevices = pBLEScan->start(scanTime, /*is_continue=*/false);
  (void)foundDevices; // we filled devs via callback

  // Sort strongest first
  std::sort(devs.begin(), devs.end(),
            [](const DevInfo& A, const DevInfo& B){ return A.rssi > B.rssi; });

  // Reset scroll & render
  blePage = 0;
  bleHoldStart = bleLastScroll = 0;

  renderLCD(devs, blePage);
  drawBarsOnOLED(devs);

  // Free BLE scan buffer
  pBLEScan->clearResults();
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
    if (!bleHoldStart) {
      // on initial press, move one step immediately
      bleHoldStart = millis();
      if (!devs.empty()) {
        blePage = (blePage + 1) % devs.size();
        renderLCD(devs, blePage);
      }
    } else if (millis() - bleHoldStart > scrollHoldTime &&
               millis() - bleLastScroll > scrollInterval) {
      if (!devs.empty()) {
        blePage = (blePage + 1) % devs.size();
        renderLCD(devs, blePage);
      }
      bleLastScroll = millis();
    }
  } else {
    bleHoldStart = bleLastScroll = 0;
  }

  delay(20);
}