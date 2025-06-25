# Signal Scanner

**Project Title**
Wi-Fi & BLE Scanner/Monitor/Disruptor + nRF24 Sniffer on ESP32

**Objective**
Build a handheld device that can:

* **Scan** nearby Wi-Fi and Bluetooth Low Energy (BLE) signals
* **Monitor** and graph signal strength over time
* **Disrupt** Wi-Fi associations via de-authentication frames
* **Sniff** raw nRF24L01+ packets across all 2.4 GHz channels

**Key Concepts**

* **Scanner**: passive listener that captures beacon or advertising frames without transmitting
* **Monitor**: time-series logging of RSSI values, visualized as bar-graphs on an OLED
* **Disruptor**: active transmitter that injects 802.11 de-auth frames to disconnect clients from an AP (Wi-Fi only)
* **Sniffer**: promiscuous-mode nRF24L01+ receiver that counts or logs any GFSK packets on each 1 MHz-spaced channel

**Hardware Components**

* **ESP32-WROOM-DA**: central MCU with dual SPI buses
* **nRF24L01+ (×2 optional)**: SPI-based 2.4 GHz transceivers (one for sniffing, one for relay/full-duplex use)
* **20×4 I²C LCD**: text menu, mode/status display
* **0.96″ I²C OLED**: graphical RSSI/packet-count bar-graphs (128×64 px)
* **Momentary button**: single-pin mode selector
* **LiPo battery & charger**: portable power (battery-monitor omitted)

**Software Modes (cycle with button)**

1. **Wi-Fi Scan** → `WiFi.scanNetworks()`, list SSID/BSSID/RSSI; OLED shows top-3 RSSI bars
2. **BLE Scan** → NimBLE passive scan for advertising packets; OLED bars of RSSI for first few devices
3. **Disruptor** → craft/send 802.11 de-auth frames to AP BSSIDs from last scan
4. **RF Sniffer** → nRF24L01+ cycles channels 0–125, counts packets per channel, graphs counts
5. **SOS Signal** → countdown then periodic SOS packets; OLED bar shows success tally

**System Block Diagram**

```text
  [LiPo + Charger]
          ↓
  [   ESP32-WROOM-DA   ]
   ├─ I²C → LCD @0x27
   ├─ I²C → OLED @0x3C
   ├─ SPI → nRF24L01+ (CE, CSN)
   ├─ GPIO → Mode Button (IRQ)
   └─ Wi-Fi & BLE radios built-in
```

**Operation Flow**

1. **Setup:** init displays, Wi-Fi, BLE, nRF24 in promiscuous/scan modes
2. **Loop:**

   * Read current `mode` (button-ISR updated)
   * Call handler for that mode
   * Update LCD text and OLED graphics
   * Brief delay, repeat

**Environment & Range**

* **Range Definition:** distance over which signals can be reliably sent or received; depends on transmit power, antenna gains, receiver sensitivity, frequency, and environment.

**Key Formula (Friis Transmission Equation):**

```math
P_r = P_t G_t G_r \left(\frac{\lambda}{4\pi d}\right)^2
```

* Solve for distance $d$:

```math
d = \frac{\lambda}{4\pi} \sqrt{\frac{P_t G_t G_r}{P_{r,\min}}}
```

**Component Range Summary**

| Component         | Frequency        | Tx Power          | Rx Sensitivity | Indoor Range | Outdoor Range |
| ----------------- | ---------------- | ----------------- | -------------- | ------------ | ------------- |
| ESP32 Wi-Fi       | 2.4 GHz (802.11) | +20 dBm           | –90 dBm        | 30–50 m      | 100–150 m     |
| ESP32 BLE         | 2.4 GHz (BLE)    | –12 dBm to +8 dBm | –90 dBm        | 5–20 m       | 30–50 m       |
| nRF24L01+ Sniffer | 2.4 GHz GFSK     | –18 dBm to 0 dBm  | –82 dBm        | 30–80 m      | 100–200 m     |

**Disruptor Range**

* Limited by the smaller of the AP’s own coverage and your transmit reach (usually matches ESP32 Wi-Fi range above).

**Environmental Effects**

* Obstacles (walls, metal) reduce range by 50–90%.
* Antenna gains/orientation can double or triple range.
* Power beyond legal limits (>+30 dBm EIRP in the US) is prohibited.

**Responsible Use**

* Stay within range of **your own** networks/devices.
* Avoid disrupting bystanders—use deauth only on networks you own.
* Dial down `esp_wifi_set_max_tx_power()` or nRF24 power if you need shorter, safer range.

