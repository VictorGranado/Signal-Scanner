# ESP32-Scanner-Sniffer

* **Project Title**
  Wi-Fi & BLE Scanner/Monitor/Disruptor + nRF24 Sniffer on ESP32

* **Objective**
  Build a handheld device that can:

  * **Scan** nearby Wi-Fi and Bluetooth Low Energy (BLE) signals
  * **Monitor** and graph signal strength over time
  * **Disrupt** Wi-Fi associations via de-authentication frames
  * **Sniff** raw nRF24L01+ packets across all 2.4 GHz channels

* **Key Concepts**

  * **Scanner**: passive listener that captures beacon or advertising frames without transmitting
  * **Monitor**: time-series logging of RSSI values, visualized as bar-graphs on an OLED
  * **Disruptor**: active transmitter that injects 802.11 de-auth frames to disconnect clients from an AP (Wi-Fi only)
  * **Sniffer**: promiscuous-mode nRF24L01+ receiver that counts or logs any GFSK packets on each 1 MHz-spaced channel

* **Hardware Components**

  * **ESP32-WROOM-DA**: central MCU with dual SPI buses
  * **nRF24L01+ (×2 optional)**: SPI-based 2.4 GHz transceivers (one for sniffing, one for relay/full-duplex use)
  * **20×4 I²C LCD**: text menu, mode/status display
  * **0.96″ I²C OLED**: graphical RSSI/packet-count bar-graphs (128×64 px)
  * **Momentary button**: single-pin mode selector
  * **LiPo battery & charger**: portable power (battery-monitor omitted)

* **Software Modes (cycle with button)**

  1. **Wi-Fi Scan** → `WiFi.scanNetworks()`, list SSID/BSSID/RSSI; OLED shows top-3 RSSI bars
  2. **BLE Scan** → NimBLE passive scan for advertising packets; OLED bars of RSSI for first few devices
  3. **Disruptor** → craft/send 802.11 de-auth frames to AP BSSIDs from last scan
  4. **RF Sniffer** → nRF24L01+ cycles channels 0–125, counts packets per channel, graphs counts

* **System Block Diagram**

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

* **Operation Flow**

  1. **Setup:** init displays, Wi-Fi, BLE, nRF24 in promiscuous/scan modes
  2. **Loop:**

     * Read current `mode` (button-ISR updated)
     * Call handler for that mode
     * Update LCD text and OLED graph
     * Delay briefly, repeat

* **Legal Note**

  * **Passive scanning** is legal; **active jamming/disruption** (deauth frames) violates FCC rules in most jurisdictions—use responsibly and only on networks you own.
 
  * * **Range (Definition):** the maximum distance over which your device can reliably send or receive signals, heavily influenced by transmit power, antenna gains, receiver sensitivity, frequency, and environmental factors.

* **Key Formula (Friis Transmission Equation):**

  $$
    P_r = P_t\,G_t\,G_r\!\left(\frac{\lambda}{4\pi d}\right)^{\!2}
  $$

  * $P_r$: received power (W)
  * $P_t$: transmit power (W)
  * $G_t,G_r$: transmit/receive antenna gains (unitless)
  * $\lambda$: wavelength (m), $\lambda = \tfrac{c}{f}$
  * $d$: distance between antennas (m)
  * Solve for $d$:

    $$
      d = \frac{\lambda}{4\pi}\sqrt{\frac{P_t G_t G_r}{P_{r,\min}}}
    $$
  * $P_{r,\min}$: minimum detectable power (receiver sensitivity).

* **ESP32 Wi-Fi Scanner/Disruptor:**

  * **Frequency:** 2.4 GHz (IEEE 802.11b/g/n) → $\lambda\approx0.125$ m
  * **Tx Power:** up to +20 dBm (100 mW) by default
  * **Rx Sensitivity:** around –90 dBm (for 1 Mbps)
  * **Typical Range:**

    * **Indoor:** 30–50 m (walls, furniture)
    * **Outdoor/Line-of-sight:** 100–150 m

* **ESP32 BLE Scanner:**

  * **Frequency:** 2.4 GHz BLE advertising → same $\lambda$
  * **Tx Power:** configurable from –12 dBm to +8 dBm
  * **Rx Sensitivity:** around –90 dBm
  * **Typical Range:**

    * **Indoor:** 5–20 m
    * **Outdoor/Line-of-sight:** 30–50 m

* **nRF24L01+ Sniffer:**

  * **Frequency:** 2.4 GHz GFSK
  * **Tx Power:** selectable –18 dBm to 0 dBm
  * **Rx Sensitivity:** around –82 dBm
  * **Typical Range:**

    * **Indoor:** 30–80 m
    * **Outdoor/Line-of-sight:** 100–200 m

* **Disruptor (“De-auth”) Range:**

  * Limited to the smaller of:

    * **AP’s coverage area** (where clients can hear you)
    * **Your own Tx power reach**
  * Practically matches the ESP32 Wi-Fi range above.

* **Environmental Effects:**

  * **Obstacles:** walls, metal, and interference reduce range by 50–90%.
  * **Antenna Orientation/Gain:** higher-gain or external antennas can double or triple reach.
  * **Regulations:** increasing power beyond legal limits (e.g. >+30 dBm EIRP in the US) is illegal.

---

**Responsible Use:**

* Stay well within range of **your own** networks or devices.
* Avoid transmitting disruptive frames at the edge of their coverage—it may inadvertently affect bystanders’ networks.
* If you need shorter range for safety, dial down the ESP32’s `esp_wifi_set_max_tx_power()` and/or nRF24’s power setting.

