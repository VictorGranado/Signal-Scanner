# Signal Scanner v1.0

The first version of the Signal Scanner, a handheld multi-mode device for wireless signal scanning and basic electrical measurements.

---

## Project Overview
The Signal Scanner v1.0 is a multifunction diagnostic tool built on the **ESP32-WROOM-DA** module.  
It integrates wireless scanning (Wi-Fi, BLE, RF sniffer, EMF meter) with analog measurements (voltage, current) into a single portable platform.  
The system is designed for hobbyist RF exploration, embedded systems testing, and field diagnostics.

The user interface combines a **20×4 I²C LCD** (for text menus and status) and a **0.96″ I²C OLED** (for bar graphs and plots). Navigation is controlled with two buttons: **MODE** and **SCROLL**.

---

## Features

### Multi-Mode Integration
- **Wi-Fi Scanner**  
  Scans for nearby access points, displaying SSID, channel, RSSI, and encryption type.  
  *LCD*: list view of APs.  
  *OLED*: bar graph of strongest networks.

- **BLE Scanner**  
  Detects advertising devices, showing device name (or MAC tail) and RSSI.  
  *LCD*: device details.  
  *OLED*: RSSI bar graph.

- **RF Sniffer (AD8317)**  
  Reads RF power from 1 MHz–10 GHz via AD8317 logarithmic detector.  
  Converts Vout to approximate dBm after calibration.  
  *LCD*: live dBm value.  
  *OLED*: scrolling RF strength plot.

- **EMF Meter (Parallel)**  
  Simple EMF detection circuit (antenna + rectifier) connected in parallel with the main board.  
  Provides relative measurement of electromagnetic field strength.  
  Displayed via LEDs.

- **Voltage Meter (0–25 V)**  
  Uses a divider module and ADC scaling.  
  *LCD*: probe wiring guide + measured Vin.  
  *OLED*: scrolling Vin plot.

- **Current Meter (ACS712-30A)**  
  Measures DC/AC current with tare/zero capability.  
  *LCD*: wiring guide + amps value.  
  *OLED*: scrolling absolute current plot.

---

### Dynamic User Interface
- **Displays**  
- LCD: menus, instructions, measurement values  
- OLED: bar graphs (Wi-Fi/BLE/EMF), live plots (V, A, RF)  
- **Controls**  
- `MODE` (GPIO2):  
  * Short press → rescan/reset/tare depending on mode  
  * Long press (>1.2s, in Current mode) → tare capture  
- `SCROLL` (GPIO4):  
  * Wi-Fi/BLE: scroll device list (with hold-to-autoscroll)  
  * Voltage/Current/EMF: pause/resume plots  
- `MODE + SCROLL` held → cycle to next mode  

---

## System Specifications

### Hardware Components
- **ESP32-WROOM-DA** (MCU + Wi-Fi + BLE + ADC)  
- **AD8317 RF Power Detector** (1 MHz–10 GHz)  
- **EMF Meter Circuit** (antenna + rectifier + amplifier, parallel connection)  
- **ACS712-30A Current Sensor**  
- **0–25 V Voltage Divider Module**  
- **20×4 I²C LCD** (0x27)  
- **0.96″ OLED SSD1306** (0x3C)  
- **Push Buttons**: Mode + Scroll  
- **Li-Ion Battery + Charger**  

### Operating Constraints
- **Voltage readings**: require calibration (divider tolerance, ADC scaling).  
- **Current readings**: tare function required for best accuracy.  
- **RF Sniffer**: approximate dBm values, best used as relative strength indicator.  
- **EMF Meter**: relative-only, no absolute units; indicates field activity.  
- **Display refresh**: OLED plots update in real time, LCD menus updated per mode action.  

### Known Limitations
- ESP32 ADC is nonlinear; calibration improves accuracy.  
- AD8317 sniffer requires 2-point calibration with known RF sources for accurate dBm values.  
- EMF meter is qualitative (good for “near/far” detection, not absolute field strength).  
- No internal data logging in this version.  

---

## Setup and Usage

### 1. Hardware Connections
- LCD and OLED connected via shared I²C (SDA, SCL).  
- Voltage divider → ESP32 ADC pin (GPIO34).  
- ACS712 current sensor → ESP32 ADC pin (GPIO35).  
- AD8317 Vout → ESP32 ADC pin (GPIO36).  
- EMF circuit → spare ADC pin (or analog mux) in parallel.  
- Buttons → GPIO2 (MODE), GPIO4 (SCROLL).  

### 2. Software Requirements
- **Arduino IDE** with ESP32 board support installed.  
- Required Libraries:  
- `LiquidCrystal_I2C` (ESP32-compatible fork)  
- `Adafruit_GFX` + `Adafruit_SSD1306`  
- `WiFi` (ESP32 core)  
- `BLEDevice` (ESP32 core, Kolban’s BLE)  

### 3. Steps to Run
1. Connect components as listed.  
2. Open project code in Arduino IDE.  
3. Set **Partition Scheme → Huge APP** (to fit firmware).  
4. Upload code to ESP32.  
5. Power device via USB or Li-Ion battery.  
6. Use buttons to navigate between modes.  

---

## Test Plan and Results

### Test Cases
- **Wi-Fi Scan**: detects nearby APs, displays SSID + RSSI.  
- **BLE Scan**: finds BLE beacons, displays names/addresses.  
- **Voltage Meter**: reads batteries; shows ~correct voltage after divider calibration.  
- **Current Meter**: tare works, responds to load changes.  
- **RF Sniffer**: baseline ~0.2 V at noise floor, rises with RF signals.  
- **EMF Meter**: detects field activity when placed near active electronics.  
- **UI Navigation**: LCD + OLED synchronized, buttons responsive.  

### Results
- All scanning and measurement modes functional.  
- Startup welcome screen and OLED animation working.  
- Voltage measurement initially over-reports; corrected with calibration.  
- RF sniffer shows clear relative signal strength, requires calibration for absolute values.  
- EMF reader works as expected for relative detection.  

---

## Future Enhancements
- SD card logging for Wi-Fi/BLE/RF/EMF scans.  
- Data streaming via BLE to remote dashboard.  
- Improved ADC calibration (using ESP32 eFuse).  
- Optional enclosure with integrated antenna.  

---

## Updated Parts List
| Component            | Approx. Cost |
|----------------------|--------------|
| ESP32-WROOM-DA       | $5–10        |
| AD8317 RF Detector   | $15–20       |
| EMF Detector Circuit | $14         |
| ACS712-30A           | $3–5         |
| Voltage Divider      | $2–3         |
| 20×4 I²C LCD         | $5–10        |
| 0.96″ OLED SSD1306   | $2–5         |
| Push Buttons/Wiring  | <$2          |
| Li-Ion Battery + PCB | $8–12        |
| Charger Module       | $2–4         |
| **Total**            | ~$56–85      |

---

## Contributors
- **Victor Stafussi Granado** – Embedded Systems Design, Firmware Development, System Integration  

---

## License
This project is licensed under the **MIT License**. See the LICENSE file for details.  

---

## Acknowledgments
- AD8317 datasheet (Analog Devices)  
- ESP32 documentation and Arduino core support  
- Arduino community libraries (Adafruit, Kolban, etc.)  
- EMF detector circuits and open-source hardware references  

