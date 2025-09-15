// ====== ESP32 Voltage Meter for 0–25V Module (B08BZKPSFY) ======
// - LCD 20x4: mode, probe wiring, live Vin
// - OLED 128x64: real-time plot (0..25 V), no "V" symbol
// - MODE button (pin 2): Clear plot
// - SCROLL button (pin 4): Pause/Resume plot
//
// Voltage module note: OUT ≈ Vin/5 (at 25 V -> ~5 V), so add a small
// extra 2:1 divider from OUT to ESP32 ADC (e.g., 100k:100k) to keep <=3.3 V.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pins ----------
#define MODE_BUTTON_PIN    2
#define SCROLL_BUTTON_PIN  4
#define PIN_V_ADC          34    // ADC1_CH0 (GPIO36)

// ---------- LCD ----------
#define LCD_ADDR 0x27
#define LCD_COLS 20
#define LCD_ROWS 4
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ---------- OLED ----------
#define OLED_RESET   -1
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------- Scaling ----------
static const float VREF          = 3.30f;   // ESP32 ADC full-scale (~3.30 V with 11dB)
static const float MODULE_RATIO  = 5.0f;    // 0–25V module: Vout ≈ Vin / 5
static const float EXTRA_DIV     = 2.0f;    // extra 2:1 divider to ADC
static const float ADC_TO_VIN    = MODULE_RATIO * EXTRA_DIV; // overall x10
static const float FULL_SCALE_V  = 25.0f;   // plot top

// Calibration trims
static const float CAL_K = 1.000f;  // gain
static const float CAL_B = 0.000f;  // offset (V)

// Smoothing & sampling
static const int   ADC_SAMPLES = 32;
static const float EMA_ALPHA   = 0.2f;

// Plot buffer
static float trace[SCREEN_WIDTH];
static bool  plotPaused = false;
static float emaV = 0.0f;

// ---------- Utils ----------
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

static int readADCavg(int pin, int n) {
  uint32_t acc = 0;
  for (int i = 0; i < n; ++i) acc += analogRead(pin);
  return (int)(acc / n);
}
static float adcToVolts(int raw) { return (raw / 4095.0f) * VREF; }

static float measureVin() {
  int raw = readADCavg(PIN_V_ADC, ADC_SAMPLES);
  float vAdc = adcToVolts(raw);
  float vin  = (vAdc * ADC_TO_VIN) * CAL_K + CAL_B;
  if (vin < 0) vin = 0;
  return vin;
}

static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) { String t=s; while(t.length()<20) t+=' '; return t; }
  return s.substring(0,17) + "...";
}

// ---------- UI ----------
static void renderLCD(float vin) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Voltage Meter"));
  lcd.setCursor(0,1); lcd.print(padOrTrim20("RED=VIN BLK=GND"));
  char buf[21]; snprintf(buf,sizeof(buf),"Vin: %6.2f V", vin);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(buf)));
  String s4 = String("Module x5, ADC x2 ");
  if (plotPaused) s4 = "Module x5, [PAUSE]";
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s4));
}

static void drawAxes() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(gx-22, gy-1); oled.print((int)FULL_SCALE_V); oled.print("V");
  oled.setCursor(gx-18, gy+gh-8); oled.print("0V");
  oled.display();
}

static void plotAddPoint(float vin){
  for(int i=0;i<SCREEN_WIDTH-1;++i) trace[i]=trace[i+1];
  trace[SCREEN_WIDTH-1]=vin;
}
static void renderPlot(){
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromV = [&](float v){
    if(v<0) v=0; if(v>FULL_SCALE_V) v=FULL_SCALE_V;
    int y = gy+gh-2 - (int)round((v/FULL_SCALE_V)*(gh-3));
    if(y<gy+1) y=gy+1; if(y>gy+gh-2) y=gy+gh-2; return y;
  };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if(bufStart<1) bufStart=1;
  int prevY=yFromV(trace[bufStart-1]);
  for(int x=x0; x<=x1; ++x){
    int idx=(x-x0)+bufStart; int y=yFromV(trace[idx]);
    oled.drawLine(x-1, prevY, x, y, SSD1306_WHITE);
    prevY=y;
  }
  oled.display();
}

// ---------- Setup/Loop ----------
void setup(){
  pinMode(MODE_BUTTON_PIN,INPUT_PULLUP);
  pinMode(SCROLL_BUTTON_PIN,INPUT_PULLUP);

  lcd.init(); lcd.backlight();
  Wire.begin();
  oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.clearDisplay(); oled.display();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_V_ADC, ADC_11db); // up to ~3.3 V

  for(int i=0;i<SCREEN_WIDTH;++i) trace[i]=0.0f;
  drawAxes();

  Serial.begin(115200);
}

void loop(){
  if(buttonPressed(MODE_BUTTON_PIN)){
    for(int i=0;i<SCREEN_WIDTH;++i) trace[i]=0.0f;
    drawAxes();
  }
  if(buttonPressed(SCROLL_BUTTON_PIN)){
    plotPaused = !plotPaused;
  }

  float v = measureVin();
  emaV = EMA_ALPHA*v + (1.0f-EMA_ALPHA)*emaV;

  renderLCD(emaV);
  if(!plotPaused) plotAddPoint(emaV);
  renderPlot();

  delay(50);
}
