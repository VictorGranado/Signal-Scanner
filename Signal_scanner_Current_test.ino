// ====== ESP32 Current Meter for ACS712-30A (B08BZKPSFY) ======
// - LCD 20x4: mode, probe wiring, live current with sign
// - OLED 128x64: real-time plot of |I| from 0..30 A (no symbol)
// - MODE button (pin 2): Zero/Tare (set current offset)
// - SCROLL button (pin 4): Pause/Resume plot
//
// Wiring:
//   ACS712 @ 5V:  Vcc=5V, GND=GND
//   Current path in series: IP+ (from supply) -> sensor -> IP- (to load)
//   Sensor OUT -> 2:1 divider -> ESP32 ADC (GPIO39 recommended)
//     e.g., OUT --100k--+-- ADC39
//                        |
//                       100k
//                        |
//                       GND
//
// Notes:
//   ACS712-30A sensitivity ~ 66 mV/A @ 5V supply.
//   Vout = 2.5V + (66 mV/A)*I  (I positive for IP+ -> IP-).
//   After 2:1 divider, ADC sees ~ (Vout/2).
//   We'll tare the midpoint with MODE to cancel offset/drift.

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ---------- Pins ----------
#define MODE_BUTTON_PIN    2
#define SCROLL_BUTTON_PIN  4
#define PIN_I_ADC          34    // ADC1_CH3 (GPIO39)

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

// ---------- ACS712 scaling ----------
static const float ESP_VREF          = 3.30f;   // ADC full-scale volts (11 dB)
static const float DIV_RATIO         = 2.0f;    // 2:1 divider from sensor Vout to ADC
static const float ACS_VCC           = 5.00f;   // ACS supply
static const float ACS_ZERO_V        = ACS_VCC * 0.5f;   // ~2.5 V mid
static const float SENS_mV_PER_A     = 66.0f;   // ACS712-30A sensitivity
static const float SENS_V_PER_A      = SENS_mV_PER_A / 1000.0f; // 0.066 V/A
static const float FULL_SCALE_A      = 30.0f;   // plot top (0..30A magnitude)

// Calibration trims
static const float GAIN_TRIM         = 1.000f;  // multiply current
static const float OFFSET_TRIM_A     = 0.000f;  // add/subtract amps after tare if needed

// Sampling / smoothing
static const int   ADC_SAMPLES       = 32;
static const float EMA_ALPHA         = 0.2f;

// Plot buffer & state
static float trace[SCREEN_WIDTH];
static bool  plotPaused = false;
static float emaI = 0.0f;

// Tare (zero) state (in Volts at the sensor output, pre-divider domain)
static float tareZero_V = ACS_ZERO_V;  // start at nominal 2.5V mid

// ---------- Helpers ----------
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
static float adcToVolts(int raw) { return (raw / 4095.0f) * ESP_VREF; }

// Convert ADC reading -> sensor output volts (undo divider)
static float adcToSensorV(int raw) {
  float vAdc = adcToVolts(raw);
  return vAdc * DIV_RATIO; // back to sensor-output domain
}

// Compute current (amps) from sensor output volts with tare & trims
static float sensorVToAmps(float vSensor) {
  float deltaV = vSensor - tareZero_V;       // remove offset (tare)
  float amps   = deltaV / SENS_V_PER_A;      // I = dV / (0.066 V/A)
  amps = amps * GAIN_TRIM + OFFSET_TRIM_A;
  return amps;
}

static String padOrTrim20(const String& s) {
  if (s.length() == 20) return s;
  if (s.length() < 20) { String t=s; while(t.length()<20) t+=' '; return t; }
  return s.substring(0,17) + "...";
}

// ---------- UI ----------
static void renderLCD(float amps) {
  lcd.setCursor(0,0); lcd.print(padOrTrim20("Mode: Current Meter"));
  // Wiring hint
  lcd.setCursor(0,1); lcd.print(padOrTrim20("IP+=SUP IP-=LOAD"));
  // Live current with sign
  char buf[21]; snprintf(buf, sizeof(buf), "I: %7.3f A", amps);
  lcd.setCursor(0,2); lcd.print(padOrTrim20(String(buf)));
  // State line
  String s4 = plotPaused ? String("Plot: [PAUSE]       ")
                         : String("Plot: RUN           ");
  lcd.setCursor(0,3); lcd.print(padOrTrim20(s4));
}

static void drawAxes() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.drawRect(gx, gy, gw, gh, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setCursor(gx-22, gy-1);     oled.print((int)FULL_SCALE_A); oled.print("A");
  oled.setCursor(gx-16, gy+gh-8);  oled.print("0");
  oled.display();
}

static void plotAddPoint(float ampsAbs){
  for(int i=0;i<SCREEN_WIDTH-1;++i) trace[i]=trace[i+1];
  trace[SCREEN_WIDTH-1]=ampsAbs;
}

static void renderPlot(){
  const int gx=10, gy=6; const int gw=SCREEN_WIDTH-gx-2, gh=SCREEN_HEIGHT-gy-8;
  oled.fillRect(gx+1, gy+1, gw-2, gh-2, SSD1306_BLACK);
  auto yFromA = [&](float a){
    if(a<0) a=0; if(a>FULL_SCALE_A) a=FULL_SCALE_A;
    int y = gy+gh-2 - (int)round((a/FULL_SCALE_A)*(gh-3));
    if(y<gy+1) y=gy+1; if(y>gy+gh-2) y=gy+gh-2; return y;
  };
  int x0=gx+1, x1=gx+gw-2; int bufStart=SCREEN_WIDTH-(gw-2); if(bufStart<1) bufStart=1;
  int prevY=yFromA(trace[bufStart-1]);
  for(int x=x0; x<=x1; ++x){
    int idx=(x-x0)+bufStart; int y=yFromA(trace[idx]);
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
  analogSetPinAttenuation(PIN_I_ADC, ADC_11db); // up to ~3.3 V on ADC pin

  // Init plot buffer
  for(int i=0;i<SCREEN_WIDTH;++i) trace[i]=0.0f;
  drawAxes();

  Serial.begin(115200);
  delay(100);
}

void loop(){
  // MODE -> Tare (capture current zero point)
  if (buttonPressed(MODE_BUTTON_PIN)) {
    // Average a few samples to set tareZero_V
    int raw = readADCavg(PIN_I_ADC, 128);
    tareZero_V = adcToSensorV(raw);
  }

  // SCROLL -> Pause/Resume
  if (buttonPressed(SCROLL_BUTTON_PIN)) {
    plotPaused = !plotPaused;
  }

  // Measure current
  int raw = readADCavg(PIN_I_ADC, ADC_SAMPLES);
  float vSensor = adcToSensorV(raw);
  float amps    = sensorVToAmps(vSensor);

  // Smooth
  emaI = EMA_ALPHA*amps + (1.0f-EMA_ALPHA)*emaI;

  // Update LCD
  renderLCD(emaI);

  // Update plot with magnitude
  float aAbs = fabsf(emaI);
  if (!plotPaused) plotAddPoint(aAbs);
  renderPlot();

  // Debug (optional)
  // Serial.printf("raw=%d  vADC=%.3f  vSens=%.3f  I=%.3f A  tare=%.3f V\n",
  //               raw, adcToVolts(raw), vSensor, emaI, tareZero_V);

  delay(50);
}
