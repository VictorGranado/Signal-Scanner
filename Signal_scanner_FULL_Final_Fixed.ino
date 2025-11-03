// ================== Signal Scanner (ESP32-WROOM-DA) ==================
// Modes: Wi-Fi Scan + BLE Scan + Voltage Meter + Current Meter + Continuity + RF Sniffer (AD8317)
// ---------------------------------------------------------------------
// Hardware
//  - Buttons: MODE=GPIO2, SCROLL=GPIO4  (INPUT_PULLUP, LOW=pressed)
//  - LCD 20x4 I2C @ 0x27 (LiquidCrystal_I2C compatible fork recommended)
//  - OLED SSD1306 128x64 I2C @ 0x3C
//
// Voltage module (kit B08BZKPSFY):
//  - 0–25V module outputs ~Vin/5 (≈5 V at 25 V) -> add extra 2:1 divider to ESP32 ADC
//  - OUT -> 100k -> ADC34; ADC34 -> 100k -> GND  (overall x10 scale from ADC to Vin)
//
// Current module (ACS712-30A):
//  - Power at 5V, GND common with ESP32
//  - Route current IP+ (from supply) -> sensor -> IP- (to load)
//  - Vout -> 2:1 divider -> ADC35 (same 100k/100k idea)
//  - Sensitivity ≈ 66 mV/A, nominal mid ≈ 2.5 V at 0 A
//
// RF Sniffer (AD8317):
//  - Vout (0.2–1.8V typ) -> ADC33 (no divider)
//  - GND common, Vcc per module (~5V), antenna/probe on RF input
//
// Continuity (Option A):
//  - CONT_OUT=GPIO25 -> probe+, CONT_IN=GPIO26 -> probe- (ADC1), 10k pulldown to GND on CONT_IN
//
// Controls
//  - MODE (GPIO2):
//      * Short press: CHANGE MODE (WiFi → BLE → Volt → Curr → Cont → RF → …)
//      * Long press (>1.2s): in Current mode ONLY -> TARE (zero)
//      * Extra in RF mode: long press -> capture P0 baseline (-90 dBm)
//  - SCROLL (GPIO4):
//      * Wi-Fi/BLE: scroll list (hold to auto-scroll)
//      * Volt/Current/RF: pause/resume plot
//      * Extra in RF mode: long press -> capture P1 reference (-30 dBm)
//
// Splash:
//  - LCD: "Hello user", "Welcome to the Signal Scanner V1.0", "Initializing..."
//  - OLED: antenna with concentric waves animation
// ---------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>

// BLE (Kolban)
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <vector>
#include <algorithm>
#include <math.h>

// ================= Pins =================
#define MODE_BUTTON_PIN    27
#define SCROLL_BUTTON_PIN  4
#define PIN_V_ADC          34    // ADC1_CH6 (Voltage)
#define PIN_I_ADC          35    // ADC1_CH7 (Current)
#define PIN_RF_ADC         33    // ADC1_CH5 (RF Vout)

// Continuity pins (Option A)
#define CONT_OUT           25    // drives 3.3V (probe +)
#define CONT_IN            32    // senses via ADC2_CH9 (probe -), external 10k pulldown to GND

// ================= LCD ==================
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ================= OLED =================
#define OLED_RESET   -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================= Helpers =================
static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) { String t=s; while(t.length()<20) t+=' '; return t; }
  return s.substring(0,17) + "...";
}

static bool buttonEdge(uint8_t pin) {
  static uint32_t lastT[48] = {0};
  static uint8_t  lastS[48] = {HIGH};
  uint32_t now = millis();
  uint8_t s = digitalRead(pin);
  if (s != lastS[pin] && (now - lastT[pin]) > 30) {
    lastT[pin] = now;
    lastS[pin] = s;
    return (s == LOW);
  }
  return false;
}

static bool buttonHeld(uint8_t pin, uint32_t ms) {
  static uint32_t tDown[48] = {0};
  static uint8_t  wasDown[48] = {0};
  uint8_t s = (digitalRead(pin) == LOW);
  uint32_t now = millis();
  if (s && !wasDown[pin]) { wasDown[pin]=1; tDown[pin]=now; }
  else if (!s) { wasDown[pin]=0; return false; }
  if (wasDown[pin] && (now - tDown[pin] >= ms)) { return true; }
  return false;
}

// ================= Mode enum (use uint8_t to avoid proto issues) =================
enum ModeVals : uint8_t { WIFI_MODE=0, BLE_MODE, VOLT_MODE, CURR_MODE, CONT_MODE, RF_MODE, MODE_COUNT };
uint8_t currentMode = WIFI_MODE;

// forward declare
static void enterMode(uint8_t m);

// ===================== Wi-Fi Scanner =====================
struct APInfo { String ssid; int32_t rssi; int32_t channel; wifi_auth_mode_t enc; };
std::vector<APInfo> wifiAPs;
uint8_t wifiPage = 0;
uint32_t wifiHoldStart = 0, wifiLastScroll = 0;
const uint32_t scrollHoldTime = 500, scrollInterval = 500;

static const char* encToStr(wifi_auth_mode_t m) {
  switch (m) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2-E";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/3";
    default: return "?";
  }
}

static void oledBarsFromRSSI(const int *vals, int n, int nShow) {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  nShow = min(nShow, n);
  if (nShow <= 0) { oled.display(); return; }
  int w = SCREEN_WIDTH / nShow;
  int usableHeight = SCREEN_HEIGHT - 14;
  for (int i = 0; i < nShow; ++i) {
    int rssi = vals[i];
    rssi = max(-100, min(0, rssi));
    int bar = map(rssi, -100, 0, 0, usableHeight);
    int x = i * w;
    oled.setCursor(x + (w/2) - 3, 0); oled.print(i+1);
    oled.fillRect(x+1, SCREEN_HEIGHT - bar - 1, w - 3, bar, SSD1306_WHITE);
  }
  oled.display();
}

static void renderWiFiLCD(size_t idx) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: WiFi Scan"));
  char buf[21]; snprintf(buf,sizeof(buf),"Total APs:%-9d",(int)wifiAPs.size());
  lcd.setCursor(0,1); lcd.print(padOrTrim20(String(buf)));
  if (wifiAPs.empty()) {
    lcd.setCursor(0,2); lcd.print(padOrTrim20("No networks found"));
    lcd.setCursor(0,3); lcd.print(padOrTrim20(""));
    return;
  }
  const APInfo &a = wifiAPs[idx];
  String s1 = String(idx+1) + ": " + (a.ssid.length()?a.ssid:String("<hidden>"));
  lcd.setCursor(0,2); lcd.print(padOrTrim20(s1));
  snprintf(buf,sizeof(buf),"%ddBm Ch%02ld %s",(int)a.rssi,(long)a.channel,encToStr(a.enc));
  lcd.setCursor(0,3); lcd.print(padOrTrim20(String(buf)));
}

static void wifiScan() {
  lcd.clear(); renderWiFiLCD(0);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(true,true);
  delay(120);
  int n = WiFi.scanNetworks(false, true, false, 120);
  wifiAPs.clear();
  if (n > 0) {
    wifiAPs.reserve(n);
    for (int i=0;i<n;++i) {
      APInfo a; a.ssid=WiFi.SSID(i); a.rssi=WiFi.RSSI(i); a.channel=WiFi.channel(i); a.enc=WiFi.encryptionType(i);
      wifiAPs.push_back(a);
    }
    std::sort(wifiAPs.begin(), wifiAPs.end(), [](const APInfo&A,const APInfo&B){return A.rssi>B.rssi;});
    wifiPage=0; wifiHoldStart=wifiLastScroll=0;
  }
  renderWiFiLCD(wifiPage);
  const int maxBars = 8;
  int vals[maxBars]; int cnt = min((int)wifiAPs.size(), maxBars);
  for(int i=0;i<cnt;++i) vals[i]=wifiAPs[i].rssi;
  oledBarsFromRSSI(vals, cnt, cnt);
}

// ===================== BLE Scanner =====================
struct DevInfo { String name; String addr; int rssi; };
std::vector<DevInfo> bleDevs;
uint8_t blePage = 0;
uint32_t bleHoldStart = 0, bleLastScroll = 0;

int bleScanTime = 6;
BLEScan* pBLEScan = nullptr;

static String tailMac(const String& mac) {
  if (mac.length() >= 5) return mac.substring(mac.length()-5);
  return mac;
}

static void renderBLELCD(size_t idx) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: BLE Scan"));
  char buf[21]; snprintf(buf,sizeof(buf),"Total Dev:%-8d",(int)bleDevs.size());
  lcd.setCursor(0,1); lcd.print(padOrTrim20(String(buf)));
  if (bleDevs.empty()) {
    lcd.setCursor(0,2); lcd.print(padOrTrim20("No devices found"));
    lcd.setCursor(0,3); lcd.print(padOrTrim20(""));
    return;
  }
  const DevInfo &d = bleDevs[idx];
  String nm = d.name.length()? d.name : String("<noname>");
  String s1 = String(idx+1) + ": " + nm;
  lcd.setCursor(0,2); lcd.print(padOrTrim20(s1));
  snprintf(buf,sizeof(buf),"%ddBm %s",(int)d.rssi, tailMac(d.addr).c_str());
  lcd.setCursor(0,3); lcd.print(padOrTrim20(String(buf)));
}

static void oledBarsFromRSSI_BLE(const int *vals, int n, int nShow) {
  oled.clearDisplay(); oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  nShow = min(nShow, n);
  if (nShow <= 0) { oled.display(); return; }
  int w = SCREEN_WIDTH / nShow;
  int usableHeight = SCREEN_HEIGHT - 14;
  for (int i = 0; i < nShow; ++i) {
    int rssi = vals[i];
    rssi = max(-100, min(0, rssi));
    int bar = map(rssi, -100, 0, 0, usableHeight);
    int x = i * w;
    oled.setCursor(x + (w/2) - 3, 0); oled.print(i+1);
    oled.fillRect(x+1, SCREEN_HEIGHT - bar - 1, w - 3, bar, SSD1306_WHITE);
  }
  oled.display();
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice adv) override {
    DevInfo d;
    d.name = adv.haveName()? String(adv.getName().c_str()) : String("");
    d.addr = adv.getAddress().toString().c_str();
    d.rssi = adv.getRSSI();
    // upsert by MAC; keep strongest RSSI and latest name
    for (auto &x : bleDevs) {
      if (x.addr == d.addr) {
        if (d.rssi > x.rssi) x.rssi = d.rssi;
        if (d.name.length()) x.name = d.name;
        return;
      }
    }
    bleDevs.push_back(d);
  }
};

static void bleScan() {
  lcd.clear(); renderBLELCD(0);
  oled.clearDisplay(); oled.display();

  static bool inited=false;
  if (!inited) {
    BLEDevice::init("");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    inited = true;
  }
  bleDevs.clear();
  BLEScanResults* res = pBLEScan->start(bleScanTime, false);
  (void)res;
  std::sort(bleDevs.begin(), bleDevs.end(), [](const DevInfo&A,const DevInfo&B){return A.rssi>B.rssi;});
  blePage=0; bleHoldStart=bleLastScroll=0;
  renderBLELCD(blePage);
  const int maxBars=8; int vals[maxBars]; int cnt=min((int)bleDevs.size(), maxBars);
  for(int i=0;i<cnt;++i) vals[i]=bleDevs[i].rssi;
  oledBarsFromRSSI_BLE(vals, cnt, cnt);
  pBLEScan->clearResults();
}

// ===================== Voltage Meter =====================
static const float VREF_ADC        = 3.30f;   // with 11dB attenuation
static const float MODULE_RATIO    = 5.0f;    // 0–25V module: Vout ≈ Vin/5
static const float EXTRA_DIV       = 1.0f;    // set 2.0f if using extra 2:1 divider
static const float ADC_TO_VIN      = MODULE_RATIO * EXTRA_DIV; // overall scale
static const float V_FULL_SCALE    = 25.0f;   // plot top

static const int   ADC_SAMPLES_V   = 32;
static const float EMA_ALPHA_V     = 0.2f;
static float vTrace[SCREEN_WIDTH];
static bool  vPlotPaused = false;
static float emaV = 0.0f;

// ---- Two-point calibration (from your Fluke comparison) ----
// Vin_calibrated = CAL_GAIN_V * Vin_raw + CAL_OFFSET_V
static const float CAL_GAIN_V    = 1.032303f;
static const float CAL_OFFSET_V  = 0.470927f;   // volts


static int readADCavg(int pin, int n) { uint32_t acc=0; for(int i=0;i<n;++i) acc+=analogRead(pin); return (int)(acc/n); }
static float adcToVolts(int raw) { return (raw/4095.0f)*VREF_ADC; }

/*static float readVin() {
  int raw = readADCavg(PIN_V_ADC, ADC_SAMPLES_V);
  float vAdc = adcToVolts(raw);
  float vin  = vAdc * ADC_TO_VIN;
  if (vin < 0) vin = 0;
  return vin;
}*/

// Replace your readVin() with this calibrated version
static float readVin() {
  // Read averaged raw ADC and convert to volts at the ADC pin
  int   raw  = readADCavg(PIN_V_ADC, ADC_SAMPLES_V);
  float vAdc = adcToVolts(raw);          // volts at ADC pin (after any front-end divider)
  
  // Scale up by your divider/module ratio to get Vin (raw, uncalibrated)
  float vin_raw = vAdc * ADC_TO_VIN;

  // Apply 2-point calibration
  float vin_cal = (CAL_GAIN_V * vin_raw) + CAL_OFFSET_V;

  // Housekeeping
  if (vin_cal < 0)        vin_cal = 0;
  if (vin_cal > V_FULL_SCALE) vin_cal = V_FULL_SCALE;  // optional clamp for the plot

  return vin_cal;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------

static void drawAxesVolt() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(gx-22, gy-1); oled.print((int)V_FULL_SCALE); oled.print("V");
  oled.setCursor(gx-18, gy+gh-8); oled.print("0V");
  oled.display();
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------
static void plotAddV(float v) { for(int i=0;i<SCREEN_WIDTH-1;++i) vTrace[i]=vTrace[i+1]; vTrace[SCREEN_WIDTH-1]=v; }

static void renderPlotV() {
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromV=[&](float v){ if(v<0)v=0; if(v>V_FULL_SCALE)v=V_FULL_SCALE;
    int y= gy+gh-2 - (int)round((v/V_FULL_SCALE)*(gh-3));
    if(y<gy+1)y=gy+1; if(y>gy+gh-2)y=gy+gh-2; return y; };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if(bufStart<1) bufStart=1;
  int prevY=yFromV(vTrace[bufStart-1]);
  for(int x=x0; x<=x1; ++x){
    int idx=(x-x0)+bufStart; int y=yFromV(vTrace[idx]);
    oled.drawLine(x-1,prevY,x,y,SSD1306_WHITE); prevY=y;
  }
  oled.display();
}

static void renderLCDVolt(float vin) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Voltage Meter"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20("BLUE PORT - Polarity"));
  char buf[21]; snprintf(buf,sizeof(buf),"Vin: %6.2f V", vin);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(buf)));
  String s4 = vPlotPaused ? String("Module x5, [PAUSE]") : String("Module x5, ADC x2 ");
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s4));
}

// ===================== Current Meter (ACS712-30A) =====================
static const float DIV_RATIO_I     = 2.0f;     // 2:1 divider on ACS output
static const float ACS_VCC         = 5.00f;
static const float SENS_V_PER_A    = 0.066f;   // 66 mV/A
static const float I_FULL_SCALE    = 30.0f;

static const int   ADC_SAMPLES_I   = 32;
static const float EMA_ALPHA_I     = 0.2f;
static float iTrace[SCREEN_WIDTH];
static bool  iPlotPaused = false;
static float emaI = 0.0f;
static float tareZero_V = ACS_VCC*0.5f; // ~2.5 V nominal mid (sensor domain)

static float adcToSensorV_I(int raw) { float vAdc=adcToVolts(raw); return vAdc*DIV_RATIO_I; }
static float sensorVToAmps(float vSensor) { return (vSensor - tareZero_V) / SENS_V_PER_A; }

static void drawAxesCurr() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(gx-22, gy-1);     oled.print((int)I_FULL_SCALE); oled.print("A");
  oled.setCursor(gx-16, gy+gh-8);  oled.print("0");
  oled.display();
}

static void plotAddI(float aAbs){ for(int i=0;i<SCREEN_WIDTH-1;++i) iTrace[i]=iTrace[i+1]; iTrace[SCREEN_WIDTH-1]=aAbs; }

static void renderPlotI(){
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromA=[&](float a){ if(a<0)a=0; if(a>I_FULL_SCALE)a=I_FULL_SCALE;
    int y= gy+gh-2 - (int)round((a/I_FULL_SCALE)*(gh-3));
    if(y<gy+1)y=gy+1; if(y>gy+gh-2)y=gy+gh-2; return y; };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if(bufStart<1) bufStart=1;
  int prevY=yFromA(iTrace[bufStart-1]);
  for(int x=x0;x<=x1;++x){ int idx=(x-x0)+bufStart; int y=yFromA(iTrace[idx]);
    oled.drawLine(x-1,prevY,x,y,SSD1306_WHITE); prevY=y; }
  oled.display();
}

static void renderLCDCurr(float amps) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Current Meter"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20("GREEN PORT"));
  char buf[21]; snprintf(buf,sizeof(buf),"I: %7.3f A", amps);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(buf)));
  String s4 = iPlotPaused ? String("Plot: [PAUSE]       ") : String("Plot: RUN           ");
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s4));
}

// ===================== RF Sniffer (AD8317) =====================
static const int   ADC_SAMPLES_RF  = 64;
static const float EMA_ALPHA_RF    = 0.18f;

static const float DEFAULT_SLOPE_MV_PER_DB = 22.0f;    // ~22 mV/dB
static const float DEFAULT_REF_DBM         = -30.0f;
static const float DEFAULT_REF_VOLT        = 1.10f;

static float rfTrace[SCREEN_WIDTH];
static bool  rfPlotPaused = false;
static float emaVrf = 0.0f, emaDBrf = -100.0f;

static float cal_a = -1000.0f / DEFAULT_SLOPE_MV_PER_DB;   // ≈ -45.45 dB/V
static float cal_b = DEFAULT_REF_DBM - cal_a * DEFAULT_REF_VOLT;

static bool  haveP0=false, haveP1=false;
static float P0_V=0, P0_dBm=-90.0f;
static float P1_V=0, P1_dBm=-30.0f;

static void rfRecomputeAB() {
  if (haveP0 && haveP1 && fabsf(P1_V - P0_V) > 0.02f) {
    cal_a = (P1_dBm - P0_dBm) / (P1_V - P0_V);
    cal_b = P0_dBm - cal_a * P0_V;
  }
}

static float voltsToDbm(float v) {
  float d = cal_a * v + cal_b;
  if (d < -100.0f) d = -100.0f;
  if (d >   0.0f) d =   0.0f;
  return d;
}

static void drawAxesRF() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(0, gy-1);     oled.print("0");
  oled.setCursor(0, gy+gh-8);  oled.print("100");
  oled.display();
}

static void rfPlotAdd(float dBm) { for(int i=0;i<SCREEN_WIDTH-1;++i) rfTrace[i]=rfTrace[i+1]; rfTrace[SCREEN_WIDTH-1]=dBm; }

static void renderPlotRF() {
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromDb=[&](float d){ if(d<-100)d=-100; if(d>0)d=0;
    float t = (d + 100.0f) / 100.0f; // -100..0 -> 0..1
    int y = gy+gh-2 - (int)round(t*(gh-3));
    if(y<gy+1)y=gy+1; if(y>gy+gh-2)y=gy+gh-2; return y; };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if(bufStart<1) bufStart=1;
  int prevY=yFromDb(rfTrace[bufStart-1]);
  for(int x=x0;x<=x1;++x){ int idx=(x-x0)+bufStart; int y=yFromDb(rfTrace[idx]);
    oled.drawLine(x-1,prevY,x,y,SSD1306_WHITE); prevY=y; }
  oled.setTextSize(1);
  oled.setCursor(gx+2, gy+2); oled.print((int)round(emaDBrf)); oled.print(" dBm");
  oled.display();
}

static void renderLCDRF(float v, float dBm) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: RF Sniffer"));
  char l1[21]; snprintf(l1,sizeof(l1),"Vout: %6.3f V", v);
  lcd.setCursor(0,1); lcd.print(padOrTrim20(String(l1)));
  char l2[21]; snprintf(l2,sizeof(l2),"Pwr : %6.1f dBm", dBm);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(l2)));
  String s;
  if (haveP0 && haveP1) s = rfPlotPaused ? "Cal:2pt  [PAUSE]" : "Cal:2pt  RUN   ";
  else if (haveP0 || haveP1) s = rfPlotPaused ? "Cal:1pt [PAUSE]" : "Cal:1pt RUN   ";
  else s = rfPlotPaused ? "Cal:DEF [PAUSE]" : "Cal:DEF RUN   ";
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s));
}

// ===================== Continuity Tester (EXACT from working test, YES ≤ 600 Ω) =====================
// (Code lifted from your working continuity sketch; only wrapped into functions and mode switch.)

static const float VDD_C           = 3.30f;     // ESP32 3.3V logic
static const float R_PULLDOWN_C    = 10000.0f;  // external pulldown, Ω (10k)
static const int   N_SAMPLES_C     = 16;        // ADC oversampling
static const float OPEN_V_THRESH_C = 0.02f;     // ~20 mV, treat as open
static const float YES_OHM_MAX_C   = 600.0f;    // continuity YES ≤ 600 Ω

static float readSenseVoltsC() {
  // Use calibrated ADC read (ADC2). WiFi must be OFF for reliable ADC2 reads.
  uint32_t mvAcc = 0;
  for (int i=0;i<N_SAMPLES_C;i++) {
    mvAcc += analogReadMilliVolts(CONT_IN);
    delayMicroseconds(200);
  }
  float mv = mvAcc / (float)N_SAMPLES_C;
  return mv / 1000.0f;
}

static float estimateOhmsC(float vNode) {
  if (vNode < OPEN_V_THRESH_C) return INFINITY; // effectively open
  float rx = R_PULLDOWN_C * ((VDD_C - vNode) / vNode);
  if (rx < 0) rx = 0;
  return rx;
}

static void printOhmsLCD_C(float rx) {
  if (!isfinite(rx) || rx > 1e6f) {
    lcd.setCursor(0,3); lcd.print(padOrTrim20("R > 1MOhm"));
  } else if (rx >= 1000.0f) {
    char b[21]; snprintf(b, sizeof(b), "R = %5.2f kOhm", rx/1000.0f);
    lcd.setCursor(0,3); lcd.print(padOrTrim20(String(b)));
  } else {
    char b[21]; snprintf(b, sizeof(b), "R = %6.1f Ohm", rx);
    lcd.setCursor(0,3); lcd.print(padOrTrim20(String(b)));
  }
}

static void drawBrokenWireC() {  // renamed to avoid any accidental clashes
  oled.clearDisplay();
  int y = SCREEN_HEIGHT/2;
  oled.drawLine(10, y, 60, y, SSD1306_WHITE);
  oled.drawLine(68, y, 118, y, SSD1306_WHITE);
  oled.drawLine(60, y, 63, y-6, SSD1306_WHITE);
  oled.drawLine(63, y-6, 66, y+6, SSD1306_WHITE);
  oled.drawLine(66, y+6, 68, y, SSD1306_WHITE);
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(40, y+10); oled.print("OPEN");
  oled.display();
}

static void drawFlowingWireC(uint8_t phase) { // renamed to avoid clashes
  oled.clearDisplay();
  int y = SCREEN_HEIGHT/2;
  oled.drawLine(10, y, 118, y, SSD1306_WHITE);
  for (int x=12; x<116; x+=12) {
    int dx = (x + phase) % 12;
    if (dx < 3) oled.fillCircle(x, y, 2, SSD1306_WHITE);
  }
  oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(34, y+10); oled.print("CONNECTED");
  oled.display();
}

static void doContinuity() {

  // Reassert in case any library reconfigured the pin since last loop iter
  pinMode(CONT_OUT, OUTPUT);
  digitalWrite(CONT_OUT, HIGH);
  pinMode(CONT_IN, INPUT);

  static uint8_t phase = 0;

  // Read using calibrated ADC (ADC1 now)
  uint32_t mvAcc = 0;
  for (int i=0;i<16;i++) {
    mvAcc += analogReadMilliVolts(CONT_IN);
    delayMicroseconds(200);
  }
  float vNode = (mvAcc / 16.0f) / 1000.0f;

  // Rx = Rpd * ((VDD - Vnode) / Vnode), with open clamp
  const float VDD_C           = 3.30f;
  const float R_PULLDOWN_C    = 10000.0f;
  const float OPEN_V_THRESH_C = 0.02f;     // ~20 mV
  const float YES_OHM_MAX_C   = 600.0f;    // continuity YES ≤ 600 Ω

  float rx;
  if (vNode < OPEN_V_THRESH_C) rx = INFINITY;
  else {
    rx = R_PULLDOWN_C * ((VDD_C - vNode) / vNode);
    if (rx < 0) rx = 0;
  }
  bool hasContinuity = isfinite(rx) && (rx <= YES_OHM_MAX_C);

  // LCD
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Continuity"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20(String("Continuity: ") + (hasContinuity ? "YES" : "NO ")));
  lcd.setCursor(0,2); lcd.print(padOrTrim20("ORANGE PORT"));
  if (!isfinite(rx) || rx > 1e6f) {
    lcd.setCursor(0,3); lcd.print(padOrTrim20("R > 1MOhm"));
  } else if (rx >= 1000.0f) {
    char b[21]; snprintf(b, sizeof(b), "R = %5.2f kOhm", rx/1000.0f);
    lcd.setCursor(0,3); lcd.print(padOrTrim20(String(b)));
  } else {
    char b[21]; snprintf(b, sizeof(b), "R = %6.1f Ohm", rx);
    lcd.setCursor(0,3); lcd.print(padOrTrim20(String(b)));
  }

  // OLED
  if (hasContinuity) {
    // flowing wire
    oled.clearDisplay();
    int y = SCREEN_HEIGHT/2;
    oled.drawLine(10, y, 118, y, SSD1306_WHITE);
    for (int x=12; x<116; x+=12) {
      int dx = (x + phase) % 12;
      if (dx < 3) oled.fillCircle(x, y, 2, SSD1306_WHITE);
    }
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(34, y+10); oled.print("CONNECTED");
    oled.display();
    phase = (phase + 1) % 12;
  } else {
    // broken wire
    oled.clearDisplay();
    int y = SCREEN_HEIGHT/2;
    oled.drawLine(10, y, 60, y, SSD1306_WHITE);
    oled.drawLine(68, y, 118, y, SSD1306_WHITE);
    oled.drawLine(60, y, 63, y-6, SSD1306_WHITE);
    oled.drawLine(63, y-6, 66, y+6, SSD1306_WHITE);
    oled.drawLine(66, y+6, 68, y, SSD1306_WHITE);
    oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(40, y+10); oled.print("OPEN");
    oled.display();
  }
}


// ===================== Mode rendering entrypoint =====================
static void enterMode(uint8_t m) {
  oled.clearDisplay(); oled.display();
  lcd.clear();

  switch (m) {
    case WIFI_MODE:
      wifiScan();
      break;

    case BLE_MODE:
      bleScan();
      break;

    case VOLT_MODE:
      for(int i=0;i<SCREEN_WIDTH;++i) vTrace[i]=0.0f;
      drawAxesVolt(); renderLCDVolt(0.0f);
      break;

    case CURR_MODE:
      for(int i=0;i<SCREEN_WIDTH;++i) iTrace[i]=0.0f;
      drawAxesCurr(); renderLCDCurr(0.0f);
      break;

    case CONT_MODE: {
    // Make sure radios aren't touching ADC timing; ADC1 is fine, but this replicates the stand-alone sketch conditions.
    WiFi.mode(WIFI_OFF);            // optional but helps mirror the test sketch environment
    btStop();                       // optional; only affects classic BT

    // Re-assert the exact test wiring every time we enter the mode
    pinMode(CONT_OUT, OUTPUT);
    digitalWrite(CONT_OUT, HIGH);   // drive the source probe HIGH
    pinMode(CONT_IN, INPUT);        // use external 10k pulldown
    analogSetPinAttenuation(CONT_IN, ADC_11db);

    lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Continuity"));
    lcd.setCursor(0,1); lcd.print(padOrTrim20("Continuity: --"));
    lcd.setCursor(0,2); lcd.print(padOrTrim20("No polarity"));
    lcd.setCursor(0,3); lcd.print(padOrTrim20("R = ----"));
    oled.clearDisplay(); oled.display();
    break;
  }


    case RF_MODE:
      for(int i=0;i<SCREEN_WIDTH;++i) rfTrace[i]=-100.0f;
      drawAxesRF(); renderLCDRF(0.0f, -100.0f);
      break;

    default: break;
  }
}

// ===================== Splash (LCD + OLED animation) =====================
static void drawAntennaFrame(uint8_t radius) {
  oled.clearDisplay();
  int bx = SCREEN_WIDTH/2;
  int by = SCREEN_HEIGHT-8;
  oled.drawLine(bx, by-20, bx, by, SSD1306_WHITE);
  oled.drawLine(bx-10, by, bx+10, by, SSD1306_WHITE);
  int tx = bx, ty = by-20;
  oled.fillCircle(tx, ty, 2, SSD1306_WHITE);
  for (uint8_t r=6; r<=radius; r+=6) {
    oled.drawCircle(tx, ty, r, SSD1306_WHITE);
  }
  oled.display();
}

static void showSplash() {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(padOrTrim20("     Hello user"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20("   Welcome to the"));
  lcd.setCursor(0,2); lcd.print(padOrTrim20("Signal Scanner V1.00"));
  lcd.setCursor(0,3); lcd.print(padOrTrim20("   Initializing..."));

  for (int i=0; i<10; ++i) {
    uint8_t r = 6 + (i%5)*6;
    drawAntennaFrame(r);
    delay(300);
  }
}

// ===================== Setup/Loop =====================
void setup() {
  pinMode(MODE_BUTTON_PIN,INPUT_PULLUP);
  pinMode(SCROLL_BUTTON_PIN,INPUT_PULLUP);

  // Continuity IO (as in test)
  pinMode(CONT_OUT, OUTPUT);
  digitalWrite(CONT_OUT, HIGH);
  // was: pinMode(CONT_IN, INPUT_PULLDOWN);
  pinMode(CONT_IN, INPUT); // we already have a real 10k pulldown to GND
  analogSetPinAttenuation(CONT_IN, ADC_11db);

  lcd.init(); lcd.backlight();

  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.clearDisplay(); oled.display();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_V_ADC,  ADC_11db);
  analogSetPinAttenuation(PIN_I_ADC,  ADC_11db);
  analogSetPinAttenuation(PIN_RF_ADC, ADC_11db);

  Serial.begin(115200);
  delay(50);

  showSplash();
  enterMode(currentMode);
}

void loop() {
  // ---- MODE (GPIO2): long press = TARE in Current / capture P0 in RF; short press = CHANGE MODE ----
  if (buttonHeld(MODE_BUTTON_PIN, 1200)) {
    if (currentMode == CURR_MODE) {
      int raw = readADCavg(PIN_I_ADC, 128);
      float vAdc = adcToVolts(raw);
      float vSens = vAdc * DIV_RATIO_I;
      tareZero_V = vSens;
      lcd.setCursor(0,3); lcd.print(padOrTrim20("Tare: zero captured"));
      delay(300);
    } else if (currentMode == RF_MODE) {
      int raw = readADCavg(PIN_RF_ADC, 200);
      float v = adcToVolts(raw);
      P0_V = v; P0_dBm = -90.0f; haveP0 = true; rfRecomputeAB();
      lcd.setCursor(0,3); lcd.print(padOrTrim20("RF: P0 = -90 dBm   "));
      delay(300);
    }
  } else if (buttonEdge(MODE_BUTTON_PIN)) {
    // CHANGE MODE on short press
    currentMode = (uint8_t)((currentMode + 1) % MODE_COUNT);
    enterMode(currentMode);
    delay(150);
  }

  // ---- SCROLL (GPIO4) ----
  if (currentMode==WIFI_MODE) {
    if (digitalRead(SCROLL_BUTTON_PIN)==LOW) {
      if (!wifiHoldStart) {
        wifiHoldStart=millis();
        if (!wifiAPs.empty()) { wifiPage=(wifiPage+1)%wifiAPs.size(); renderWiFiLCD(wifiPage); }
      } else if (millis()-wifiHoldStart>scrollHoldTime && millis()-wifiLastScroll>scrollInterval) {
        if (!wifiAPs.empty()) { wifiPage=(wifiPage+1)%wifiAPs.size(); renderWiFiLCD(wifiPage); }
        wifiLastScroll=millis();
      }
    } else { wifiHoldStart=wifiLastScroll=0; }

  } else if (currentMode==BLE_MODE) {
    if (digitalRead(SCROLL_BUTTON_PIN)==LOW) {
      if (!bleHoldStart) {
        bleHoldStart=millis();
        if (!bleDevs.empty()) { blePage=(blePage+1)%bleDevs.size(); renderBLELCD(blePage); }
      } else if (millis()-bleHoldStart>scrollHoldTime && millis()-bleLastScroll>scrollInterval) {
        if (!bleDevs.empty()) { blePage=(blePage+1)%bleDevs.size(); renderBLELCD(blePage); }
        bleLastScroll=millis();
      }
    } else { bleHoldStart=bleLastScroll=0; }

  } else if (currentMode==VOLT_MODE) {
    if (buttonEdge(SCROLL_BUTTON_PIN)) vPlotPaused = !vPlotPaused;

  } else if (currentMode==CURR_MODE) {
    if (buttonEdge(SCROLL_BUTTON_PIN)) iPlotPaused = !iPlotPaused;

  } else if (currentMode==RF_MODE) {
    // Short press: pause/resume; Long press: capture P1 reference (-30 dBm)
    if (buttonHeld(SCROLL_BUTTON_PIN, 1200)) {
      int raw = readADCavg(PIN_RF_ADC, 200);
      float v = adcToVolts(raw);
      P1_V = v; P1_dBm = -30.0f; haveP1 = true; rfRecomputeAB();
      lcd.setCursor(0,3); lcd.print(padOrTrim20("RF: P1 = -30 dBm   "));
      delay(300);
    } else if (buttonEdge(SCROLL_BUTTON_PIN)) {
      rfPlotPaused = !rfPlotPaused;
    }
  }

  // ---- Mode processing ----
  if (currentMode==VOLT_MODE) {
    float v = readVin();
    emaV = (EMA_ALPHA_V*v) + (1.0f-EMA_ALPHA_V)*emaV;
    renderLCDVolt(emaV);
    if (!vPlotPaused) plotAddV(emaV);
    renderPlotV();
    delay(50);

  } else if (currentMode==CURR_MODE) {
    int raw = readADCavg(PIN_I_ADC, ADC_SAMPLES_I);
    float vSens = adcToSensorV_I(raw);
    float amps = sensorVToAmps(vSens);
    emaI = (EMA_ALPHA_I*amps) + (1.0f-EMA_ALPHA_I)*emaI;
    renderLCDCurr(emaI);
    float aAbs = fabsf(emaI);
    if (!iPlotPaused) plotAddI(aAbs);
    renderPlotI();
    delay(50);

  } else if (currentMode==CONT_MODE) {
    doContinuity();      // EXACT logic from the working test
    delay(90);

  } else if (currentMode==RF_MODE) {
    int raw = readADCavg(PIN_RF_ADC, ADC_SAMPLES_RF);
    float v = adcToVolts(raw);
    emaVrf = (EMA_ALPHA_RF*v) + (1.0f-EMA_ALPHA_RF)*emaVrf;

    float dBm = voltsToDbm(emaVrf);
    emaDBrf = (EMA_ALPHA_RF*dBm) + (1.0f-EMA_ALPHA_RF)*emaDBrf;

    renderLCDRF(emaVrf, emaDBrf);
    if (!rfPlotPaused) rfPlotAdd(emaDBrf);
    renderPlotRF();
    delay(40);

  } else {
    delay(20);
  }
}
