Continuity = **YES** if Rx ≤ 600 Ω

---

### **6. RF Sniffer (AD8317)**
Measures relative RF power between 1 MHz–10 GHz using an AD8317 logarithmic detector.  
**LCD:** shows Vout voltage and equivalent dBm (calibrated or default).  
**OLED:** scrolling signal strength plot.

**Connection:**
- **AD8317 Vout → GPIO33 (ADC1_CH5)**  
- **Vcc:** 5 V  
- **GND:** common ground  
- **Antenna Input:** SMA connector (external antenna)

**Calibration:**
- **Long-press MODE** → capture P0 baseline (−90 dBm)  
- **Long-press SCROLL** → capture P1 reference (−30 dBm)

---

### **7. EMF Reader (Parallel)**
A parallel passive EMF detector monitors ambient electromagnetic activity.  
- Uses a **rectified antenna circuit** (antenna + diode + capacitor + amplifier).  
- Output is proportional to environmental EMF intensity.  
- **Display:** OLED bars or LED array (if connected).  

**Use Case:** detecting field sources (motors, transmitters, microwaves, etc.)

---

### **8. Frequency Counter**
Measures signal frequency via SMA or wired input.  
**LCD:** displays frequency in Hz/MHz.  
**OLED:** real-time bar indicator or waveform.  
**Ideal for:** oscillators, PWM generators, communication signals.

**Connection:**
- **SMA Input:** strong RF or square-wave sources  
- **Wired Input:** logic-level test points  
- **Input → GPIO15 (recommended)**  
(Use a voltage limiter for >3.3 V inputs)

---

## **Dynamic User Interface**

- **LCD (20×4)** — primary text interface: mode, wiring, and live values  
- **OLED (0.96″)** — graphical interface: signal bars, plots, or animation per mode  

**Controls:**
- **MODE (GPIO2):**  
  - Short press → change mode  
  - Long press (>1.2 s): special actions (tare, calibrate RF)
- **SCROLL (GPIO4):**  
  - Wi-Fi/BLE → scroll or auto-scroll lists  
  - Voltage/Current/RF → pause/resume plot  
  - RF → long press → capture calibration point

---

## **System Specifications**

| Component | Function | Interface |
|------------|-----------|-----------|
| ESP32-WROOM-DA | MCU + Wi-Fi + BLE + ADC | — |
| 20×4 LCD | Menu / Text | I²C (0x27) |
| 0.96″ OLED SSD1306 | Graphical plots | I²C (0x3C) |
| AD8317 | RF power detector (1 MHz–10 GHz) | Analog |
| ACS712-30A | Current sensing | Analog |
| Voltage Divider Module | Scales input to ADC range | Analog |
| Continuity Circuit | Resistance / continuity check | GPIO25 + ADC32 |
| EMF Detector | Field strength indication | Parallel analog |
| Frequency Counter | Pulse input (SMA / wired) | GPIO15 |
| Mode & Scroll Buttons | UI control | GPIO2, GPIO4 |
| Li-Ion Battery + Charger | Power source | — |

---

## **Connection Configurations**

| Mode | Signal | ESP32 Pin | Notes |
|------|---------|-----------|-------|
| Voltage | Divider output | GPIO34 | Up to 25 V, 100k/100k divider |
| Current | ACS712 output | GPIO35 | 5 V module, 66 mV/A |
| Continuity | Probe+ = GPIO25, Probe− = GPIO32 | 10 kΩ pulldown to GND |
| RF Sniffer | AD8317 Vout | GPIO33 | 0.2–1.8 V typical |
| EMF Reader | Parallel analog | Spare ADC | Relative measurement |
| Frequency Counter | SMA / wire input | GPIO15 | Pulse input with limiter |

---

## **Setup and Usage**

1. Wire components according to the connection table.  
2. Install required Arduino libraries:  
   - `LiquidCrystal_I2C`  
   - `Adafruit_GFX`, `Adafruit_SSD1306`  
   - `WiFi`, `BLEDevice` (Kolban’s BLE)  
3. Open the main `.ino` file in Arduino IDE.  
4. Select **Partition Scheme → Huge APP**.  
5. Upload firmware to the ESP32.  
6. Power via USB or Li-Ion battery.  
7. Navigate using the MODE and SCROLL buttons.

---

## **Displayed Values and Their Meaning**

| Mode | Display Field | Meaning |
|------|----------------|---------|
| Wi-Fi | SSID, RSSI, CH, ENC | Network information |
| BLE | Name/MAC, RSSI | Device info & signal strength |
| Voltage | Vin (V) | DC input voltage |
| Current | I (A) | Load current |
| Continuity | R (Ω), Continuity: YES/NO | Resistance or connection status |
| RF Sniffer | Vout (V), Power (dBm) | RF signal level |
| EMF | Bar level / LED | Field intensity |
| Frequency | f (Hz / MHz) | Signal frequency |

---

## **Test Plan and Results**

| Mode | Expected Behavior | Verified |
|------|-------------------|-----------|
| Wi-Fi Scan | Detects SSIDs with correct RSSI and channels | ✅ |
| BLE Scan | Lists BLE devices with signal strength | ✅ |
| Voltage Meter | Reports accurate DC voltage after calibration | ✅ |
| Current Meter | Tare and dynamic response verified | ✅ |
| Continuity Tester | Detects shorts <600 Ω; OLED animation works | ✅ |
| RF Sniffer | Detects transmitters; relative dBm accurate | ✅ |
| EMF Reader | Responds to nearby EMF sources | ✅ |
| Frequency Counter | Stable frequency readings | ✅ |

---

## **Future Enhancements**

- SD card logging for data collection  
- BLE data streaming to dashboards  
- Automatic range adjustment  
- RF sweep analyzer capability  
- Enclosure with integrated antennas  

---

## **Updated Parts List**

| Component | Approx. Cost |
|------------|--------------|
| ESP32-WROOM-DA | $5–10 |
| AD8317 RF Detector | $15–20 |
| ACS712-30A | $3–5 |
| Voltage Divider Module | $2–3 |
| Continuity Components | <$2 |
| EMF Detector Circuit | $14 |
| Frequency Counter (SMA Module) | $8–12 |
| 20×4 LCD | $5–10 |
| 0.96″ OLED SSD1306 | $2–5 |
| Buttons / Wiring | <$2 |
| Li-Ion Battery + Charger | $8–12 |
| **Total** | **~$70–100** |

---

## **Contributors**

**Victor Stafussi Granado** – Embedded Systems Design, Firmware Development, System Integration

---

## **License**

This project is licensed under the **MIT License**.  
See the LICENSE file for details.

---

## **Acknowledgments**

- AD8317 datasheet (Analog Devices)  
- ACS712, ESP32 ADC documentation  
- Arduino community libraries (Adafruit, Kolban, etc.)  
- EMF detector and frequency counter circuit references  
- Open-source hardware design resources


