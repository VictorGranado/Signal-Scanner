// Dummy UI Demo for Signal Scanner with Separate Mode and Scroll Buttons
// Shows static samples and scrolling of sample lists on LCD and OLED
// Adds SOS Morse animation and numeric overlay

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <vector>

// Pin definitions
#define MODE_BUTTON_PIN    2   // Change demo mode
#define SCROLL_BUTTON_PIN  4   // Scroll within current menu

// LCD parameters
#define LCD_ADDR           0x27
#define LCD_COLS           20
#define LCD_ROWS           4

// OLED parameters
#define OLED_RESET         -1
#define SCREEN_WIDTH       128
#define SCREEN_HEIGHT      64

// Scroll timing
const uint32_t scrollHoldTime = 500;
const uint32_t scrollInterval  = 500;

// Morse timings
const uint32_t dotDuration   = 200;
const uint32_t dashDuration  = 600;
const uint32_t symbolGap     = 200;

// Morse sequence for SOS
const char* morseSeq = "...---...";

// Demo modes
enum DemoMode { WIFI_DEMO = 0, BLE_DEMO, DISRUPTOR_DEMO, RF_DEMO, SOS_DEMO, DEMO_COUNT };
volatile DemoMode currentMode = WIFI_DEMO;
DemoMode prevMode = DEMO_COUNT;

// Scroll state
uint8_t wifiPage = 0; uint32_t wifiHoldStart = 0, wifiLastScroll = 0;
uint8_t blePage  = 0; uint32_t bleHoldStart  = 0, bleLastScroll  = 0;
uint8_t disruptPage = 0; uint32_t disruptHoldStart = 0, disruptLastScroll = 0;

// SOS state
size_t morseIndex = 0;
bool   ledOn      = false;
uint32_t morseTimer = 0;
uint32_t successCount = 12;
uint32_t failCount    = 1;

// Displays
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Sample Wi-Fi data
const char* sampleAPs[] = {"Home_Net","Guest","IoT_Sensor","Office_WiFi"};
int sampleRSSI[] = {-42, -67, -80, -55};
int sampleCh[]   = {11, 6, 1, 36};
const size_t totalAPs = sizeof(sampleAPs)/sizeof(sampleAPs[0]);

// Sample BLE data
const char* sampleDevs[] = {"FitTrack-1234","Sensor-X","Beacon_A","Tracker-B"};
int sampleBLE[] = {-51, -74, -60, -85};
const size_t totalDevs = sizeof(sampleDevs)/sizeof(sampleDevs[0]);

// Disruptor animation state
std::vector<uint8_t> waves;

// ISR: Mode button pressed
void IRAM_ATTR handleModeButton() {
  static uint32_t last=0; uint32_t now=millis();
  if(now-last>200) { currentMode=DemoMode((currentMode+1)%DEMO_COUNT); last=now; }
}

void setup() {
  // Buttons
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MODE_BUTTON_PIN), handleModeButton, FALLING);
  pinMode(SCROLL_BUTTON_PIN, INPUT_PULLUP);
  // LCD
  lcd.init(); lcd.backlight();
  // OLED
  Wire.begin(); oled.begin(SSD1306_SWITCHCAPVCC,0x3C);
  oled.clearDisplay(); oled.display();
  morseTimer = millis();
}

void loop() {
  // On mode change
  if(currentMode!=prevMode) {
    lcd.clear(); oled.clearDisplay(); oled.display(); morseIndex=0; ledOn=false; morseTimer=millis();
    wifiPage=blePage=disruptPage=0;
    wifiHoldStart=wifiLastScroll=bleHoldStart=bleLastScroll=disruptHoldStart=disruptLastScroll=0;
    waves.clear(); prevMode=currentMode;
  }

  switch(currentMode) {
    case WIFI_DEMO: {
      // LCD
      size_t idx = wifiPage % totalAPs;
      lcd.setCursor(0,0); lcd.print("Mode: WiFi Scan");
      lcd.setCursor(0,1); lcd.printf("Total APs:%u",totalAPs);
      lcd.setCursor(0,2); lcd.printf("%u: %s",idx+1,sampleAPs[idx]);
      lcd.setCursor(0,3); lcd.printf("%ddBm Ch%u",sampleRSSI[idx],sampleCh[idx]);
      // OLED bars & labels
      oled.clearDisplay(); uint8_t w=SCREEN_WIDTH/totalAPs;
      oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
      for(size_t i=0;i<totalAPs;i++){
        int bar=map(sampleRSSI[i],-100,0,0,SCREEN_HEIGHT/2);
        uint8_t x=i*w; oled.setCursor(x+(w/2)-2,0); oled.print(i+1);
        oled.fillRect(x,SCREEN_HEIGHT-bar-1,w-2,bar,SSD1306_WHITE);
      }
      oled.display();
      // Scroll
      if(digitalRead(SCROLL_BUTTON_PIN)==LOW) {
        if(!wifiHoldStart) wifiHoldStart=millis();
        else if(millis()-wifiHoldStart>scrollHoldTime && millis()-wifiLastScroll>scrollInterval) { wifiPage++; wifiLastScroll=millis(); }
      } else wifiHoldStart=wifiLastScroll=0;
      break;
    }
    case BLE_DEMO: {
      size_t idx=blePage%totalDevs;
      lcd.setCursor(0,0); lcd.print("Mode: BLE Scan");
      lcd.setCursor(0,1); lcd.printf("Total Dev:%u",totalDevs);
      lcd.setCursor(0,2); lcd.printf("%u: %s",idx+1,sampleDevs[idx]);
      lcd.setCursor(0,3); lcd.printf("%ddBm",sampleBLE[idx]);
      oled.clearDisplay(); uint8_t w2=SCREEN_WIDTH/totalDevs;
      oled.setTextSize(1); oled.setTextColor(SSD1306_WHITE);
      for(size_t i=0;i<totalDevs;i++){
        int bar=map(sampleBLE[i],-100,0,0,SCREEN_HEIGHT/2);
        uint8_t x=i*w2; oled.setCursor(x+(w2/2)-2,0); oled.print(i+1);
        oled.fillRect(x,SCREEN_HEIGHT-bar-1,w2-2,bar,SSD1306_WHITE);
      }
      oled.display();
      if(digitalRead(SCROLL_BUTTON_PIN)==LOW) {
        if(!bleHoldStart) bleHoldStart=millis();
        else if(millis()-bleHoldStart>scrollHoldTime && millis()-bleLastScroll>scrollInterval) { blePage++; bleLastScroll=millis(); }
      } else bleHoldStart=bleLastScroll=0;
      break;
    }
    case DISRUPTOR_DEMO: {
      size_t idx=disruptPage%totalAPs;
      lcd.setCursor(0,0); lcd.print("Mode: Disruptor");
      lcd.setCursor(0,1); lcd.printf("Targeting: %s",sampleAPs[idx]);
      lcd.setCursor(0,2); lcd.printf("%ddBm Ch%u",sampleRSSI[idx],sampleCh[idx]);
      lcd.setCursor(0,3); lcd.print("Deauth 802.11 sent");
      oled.clearDisplay(); static uint32_t lw=0; uint32_t now=millis();
      if(now-lw>300){waves.push_back(0); lw=now;} int cx=SCREEN_WIDTH/2, cy=SCREEN_HEIGHT/2;
      for(auto it=waves.begin();it!=waves.end();){*it+=4; if(*it>SCREEN_WIDTH/2)it=waves.erase(it);else{oled.drawCircle(cx,cy,*it,SSD1306_WHITE);++it;}}
      oled.display();
      if(digitalRead(SCROLL_BUTTON_PIN)==LOW){ if(!disruptHoldStart)disruptHoldStart=millis(); else if(millis()-disruptHoldStart>scrollHoldTime && millis()-disruptLastScroll>scrollInterval){disruptPage++;disruptLastScroll=millis();} } else disruptHoldStart=disruptLastScroll=0;
      break;
    }
    case RF_DEMO: {
      lcd.setCursor(0,0); lcd.print("Mode: RF Sniffer");
      lcd.setCursor(0,1); lcd.print("Top:Ch42 pkts=12");
      lcd.setCursor(0,2); lcd.print("Now:Ch08 pkts=3");
      lcd.setCursor(0,3); lcd.print("PktTot:37");
      oled.clearDisplay(); int counts[5]={5,2,1,4,3};
      for(int i=0;i<5;i++){int bar=map(counts[i],0,10,0,SCREEN_HEIGHT/2); oled.fillRect(10+i*24,SCREEN_HEIGHT-bar-1,20,bar,SSD1306_WHITE);} oled.display();
      break;
    }
    case SOS_DEMO: {
      // LCD display (unchanged)
      lcd.setCursor(0, 0); lcd.print("Mode: SOS Signal");
      lcd.setCursor(0, 1); lcd.print("SOS Sent @Ch05");
      lcd.setCursor(0, 2); lcd.printf("Succ:%u Fail:%u", successCount, failCount);
      
      // OLED numeric overlay
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0); oled.printf("S:%u", successCount);
      oled.setCursor(SCREEN_WIDTH - 40, 0); oled.printf("F:%u", failCount);

      // Morse animation timing
      uint32_t now = millis();
      char sym = morseSeq[morseIndex];
      uint32_t dur = (sym == '.') ? dotDuration : dashDuration;
      if (ledOn) {
        if (now - morseTimer >= dur) {
          ledOn = false; morseTimer = now;
        }
      } else {
        if (now - morseTimer >= symbolGap) {
          ledOn = true; morseTimer = now;
          morseIndex = (morseIndex + 1) % strlen(morseSeq);
        }
      }
      
      // Display SOS text in Morse
      if (ledOn) {
        oled.setTextSize(3);
        uint8_t tx = (SCREEN_WIDTH - 3*12) / 2;
        uint8_t ty = (SCREEN_HEIGHT - 16) / 2;
        oled.setCursor(tx, ty);
        oled.print("SOS");
      }

      // Channel indicator at bottom center
      oled.setTextSize(1);
      oled.setCursor((SCREEN_WIDTH - 5*6) / 2, SCREEN_HEIGHT - 8);
      oled.print("Ch05");
      
      oled.display();
      break;
    
    }
  }
  delay(50);
}
