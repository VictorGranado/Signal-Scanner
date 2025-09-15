// ================== RF Sniffer (AD8317) — ESP32-WROOM-DA ==================
// UI: 20x4 I2C LCD (0x27) + SSD1306 128x64 OLED (0x3C)
// Buttons: MODE=GPIO2, SCROLL=GPIO4  (INPUT_PULLUP, LOW=pressed)
//
// Changes per request:
//  - Removed pause/resume entirely. Plot always runs.
//  - Keep MODE short: clear plot
//  - Keep MODE long (>1.2s): capture P0 (baseline, -90 dBm)
//  - Keep SCROLL long (>1.2s): capture P1 (reference, -30 dBm)
// --------------------------------------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -------- Pins --------
#define MODE_BUTTON_PIN    27
#define SCROLL_BUTTON_PIN  4
#define PIN_RF_ADC         33    // ADC1_CH0 input-only; good for AD8317 Vout (≈0.2–1.8V)

// -------- LCD ---------
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// -------- OLED --------
#define OLED_RESET   -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// -------- ADC/Math --------
static const float VREF_ADC    = 3.30f;     // with 11 dB attenuation
static const int   ADC_SAMPLES = 64;        // oversampling
static const float EMA_ALPHA   = 0.18f;     // smoothing

// AD8317 typical slope ~22 mV/dB (negative vs Pin). Use 2-pt calibration when available.
static const float DEFAULT_SLOPE_MV_PER_DB = 22.0f; // mV/dB
static const float DEFAULT_REF_DBM         = -30.0f;
static const float DEFAULT_REF_VOLT        = 1.10f;

// Plot scale in absolute dBm
static const float DB_MIN = -100.0f;
static const float DB_MAX =    0.0f;

// -------- Buffers/UI helpers --------
static float traceBuf[SCREEN_WIDTH];   // dBm samples
static float emaV = 0.0f, emaDB = DB_MIN;

// Buttons debounce/hold
static bool buttonEdge(uint8_t pin) {
  static uint32_t lastT[48] = {0};
  static uint8_t  lastS[48] = {HIGH};
  uint32_t now = millis();
  uint8_t s = digitalRead(pin);
  if (s != lastS[pin] && (now - lastT[pin]) > 30) {
    lastT[pin] = now; lastS[pin] = s;
    return (s == LOW);
  }
  return false;
}
static bool buttonHeld(uint8_t pin, uint32_t ms) {
  static uint32_t tDown[48] = {0};
  static uint8_t  down[48] = {0};
  uint32_t now = millis();
  if (digitalRead(pin) == LOW) {
    if (!down[pin]) { down[pin] = 1; tDown[pin] = now; }
    return (now - tDown[pin]) >= ms;
  } else {
    down[pin] = 0; return false;
  }
}
static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) { String t=s; while(t.length()<20) t+=' '; return t; }
  return s.substring(0,17) + "...";
}

// -------- ADC helpers --------
static int   readADCavg(int pin, int n){ uint32_t acc=0; for (int i=0;i<n;++i) acc+=analogRead(pin); return (int)(acc/n); }
static float adcToVolts(int raw){ return (raw/4095.0f)*VREF_ADC; }

// -------- Calibration storage --------
// dBm = a*V + b
static float cal_a = -1000.0f / DEFAULT_SLOPE_MV_PER_DB; // ≈ -45.45 dB/V
static float cal_b = DEFAULT_REF_DBM - cal_a * DEFAULT_REF_VOLT;

// Two-point capture (V0,dBm0) and (V1,dBm1)
static bool  haveP0=false, haveP1=false;
static float P0_V=0, P0_dBm=-90.0f;   // baseline tag
static float P1_V=0, P1_dBm=-30.0f;   // reference tag

static void recomputeAB() {
  if (haveP0 && haveP1 && fabsf(P1_V - P0_V) > 0.02f) {
    cal_a = (P1_dBm - P0_dBm) / (P1_V - P0_V);
    cal_b = P0_dBm - cal_a * P0_V;
  }
}

static float voltsToDbm(float v) {
  float d = cal_a * v + cal_b;
  if (d < DB_MIN) d = DB_MIN;
  if (d > DB_MAX) d = DB_MAX;
  return d;
}

// -------- UI drawing --------
static void drawAxes() {
  oled.clearDisplay(); oled.setTextColor(SSD1306_WHITE); oled.setTextSize(1);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  // y labels (approx)
  oled.setCursor(0, gy-1);    oled.print("0");
  oled.setCursor(0, gy+gh-8); oled.print("100");
  oled.display();
}
static void plotShiftAdd(float dBm) {
  for (int i=0;i<SCREEN_WIDTH-1;++i) traceBuf[i] = traceBuf[i+1];
  traceBuf[SCREEN_WIDTH-1] = dBm;
}
static void renderPlot() {
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromDb=[&](float d){ if(d<DB_MIN)d=DB_MIN; if(d>DB_MAX)d=DB_MAX;
    float t = (d - DB_MIN) / (DB_MAX - DB_MIN);
    int y = gy+gh-2 - (int)round(t*(gh-3));
    if (y<gy+1) y=gy+1; if (y>gy+gh-2) y=gy+gh-2; return y; };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if (bufStart<1) bufStart=1;
  int prevY = yFromDb(traceBuf[bufStart-1]);
  for (int x=x0; x<=x1; ++x) {
    int idx=(x-x0)+bufStart; int y = yFromDb(traceBuf[idx]);
    oled.drawLine(x-1, prevY, x, y, SSD1306_WHITE); prevY=y;
  }
  // live dBm text
  oled.setTextSize(1);
  oled.setCursor(gx+2, gy+2);
  oled.print((int)round(emaDB)); oled.print(" dBm");
  oled.display();
}
static void renderLCD(float v, float dBm) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: RF Sniffer"));
  char line1[21]; snprintf(line1, sizeof(line1), "Vout: %6.3f V", v);
  lcd.setCursor(0,1); lcd.print(padOrTrim20(String(line1)));
  char line2[21]; snprintf(line2, sizeof(line2), "Pwr : %6.1f dBm", dBm);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(line2)));

  // Calibration status (no pause text anymore)
  String s;
  if (haveP0 && haveP1) s = "Cal:2pt RUN       ";
  else if (haveP0 || haveP1) s = "Cal:1pt RUN       ";
  else s = "Cal:DEF RUN       ";
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s));
}

// -------- Setup / Loop --------
void setup() {
  pinMode(MODE_BUTTON_PIN,   INPUT_PULLUP);
  pinMode(SCROLL_BUTTON_PIN, INPUT_PULLUP);

  lcd.init(); lcd.backlight();
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.clearDisplay(); oled.display();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RF_ADC, ADC_11db);

  for (int i=0;i<SCREEN_WIDTH;++i) traceBuf[i] = DB_MIN;
  drawAxes();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: RF Sniffer"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20("Hold MODE: set P0"));
  lcd.setCursor(0,2); lcd.print(padOrTrim20("Hold SCROLL: set P1"));
  lcd.setCursor(0,3); lcd.print(padOrTrim20("Plot: ALWAYS RUN"));
  delay(1000);
}

void loop() {
  // ---------- Buttons ----------
  // MODE long: capture P0 (baseline/noise, -90 dBm)
  if (buttonHeld(MODE_BUTTON_PIN, 1200)) {
    int raw = readADCavg(PIN_RF_ADC, 200);
    float v = adcToVolts(raw);
    P0_V = v; P0_dBm = -90.0f; haveP0 = true;
    recomputeAB();
    lcd.setCursor(0,3); lcd.print(padOrTrim20("Captured P0=-90 dBm "));
    delay(300);
  } else if (buttonEdge(MODE_BUTTON_PIN)) {
    // MODE short: clear plot
    for (int i=0;i<SCREEN_WIDTH;++i) traceBuf[i]=DB_MIN;
    drawAxes();
    lcd.setCursor(0,3); lcd.print(padOrTrim20("Plot cleared        "));
    delay(120);
  }

  // SCROLL long: capture P1 (reference, -30 dBm)
  if (buttonHeld(SCROLL_BUTTON_PIN, 1200)) {
    int raw = readADCavg(PIN_RF_ADC, 200);
    float v = adcToVolts(raw);
    P1_V = v; P1_dBm = -30.0f; haveP1 = true;
    recomputeAB();
    lcd.setCursor(0,3); lcd.print(padOrTrim20("Captured P1=-30 dBm "));
    delay(300);
  }
  // NOTE: SCROLL short press is intentionally ignored (no pause/resume).

  // ---------- Sampling ----------
  int raw = readADCavg(PIN_RF_ADC, ADC_SAMPLES);
  float v = adcToVolts(raw);
  emaV  = EMA_ALPHA*v  + (1.0f-EMA_ALPHA)*emaV;

  float dBm = voltsToDbm(emaV);
  emaDB = EMA_ALPHA*dBm + (1.0f-EMA_ALPHA)*emaDB;

  // ---------- UI ----------
  renderLCD(emaV, emaDB);
  plotShiftAdd(emaDB);       // always running
  renderPlot();

  delay(40);
}

