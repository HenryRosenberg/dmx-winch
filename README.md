# DMX Winch Controller
ESP32-S3–based DMX512 winch controller for theater prop lifting. Supports web configuration, real-time position control, and comprehensive safety features.

**⚠️ For lifting props only. No humans below during movement.**

**These are not comprehensive instructions! This is a project overview**

<img width="1303" height="942" alt="Roxie Sign Winch v14" src="https://github.com/user-attachments/assets/4e138365-bdda-4bf0-a8fb-b5939c284ab9" />


<br>

## Resources

| Resource | Link | Notes |
|-----------|------|-------|
| Mechanical CAD Design | [Fusion360](https://a360.co/45cJdRQ) | Current, up-to-date design files |
| Project Overview Drawings | [Roxie Sign Winch Drawing v4](https://github.com/user-attachments/files/25563970/Roxie.Sign.Winch.Drawing.v4.pdf) | Outdated! For reference only. See CAD link for latest design. |
| Electrical Schematics | TBD | Not yet created. Refer to block diagram below |

## Controller Block Diagram
[High Resolution Linked Here](https://github.com/user-attachments/files/25567567/DMX.Winch.Block.Diagram.-.Google.Drawings.pdf)

<img width="888" height="647" alt="DMX Winch Block Diagram" src="https://github.com/user-attachments/assets/ce3cf80b-8b9b-4436-a296-09cc8d26d10a" />

<br>

## Controller Parts List

![controller_inside](https://github.com/user-attachments/assets/7e719816-2242-4ac4-8e1b-4f8e5a703a09)
<p align="center">
  <img src="https://github.com/user-attachments/assets/b768c09d-fd3b-4b5e-a548-402da7661af9" width="48%" />
  <img src="https://github.com/user-attachments/assets/82fef94e-797c-4b01-8674-bfe05170a1f3" width="48%" />
</p>


| Component | Part | Notes |
|------------|------|-------|
| MCU | [ESP32-S3-N16R8](https://www.aliexpress.us/item/3256808865544909.html) + [Carrier Board](https://www.aliexpress.us/item/3256807127113425.html) | Main controller |
| RS485 Transceiver (DMX) | [MAX-485 Module](https://www.aliexpress.us/item/3256809269106959.html) | DMX interface |
| Control PSU | [5V 2A (10W)](https://www.aliexpress.us/item/3256807161025414.html) | Powers MCU + other control circuitry |
| Motor Driver | [DM542T](https://www.omc-stepperonline.com/digital-stepper-driver-1-0-4-2a-20-50vdc-for-nema-17-23-24-stepper-motor-dm542t) | |
| Motor PSU | [48V 5.2A (250W)](https://www.omc-stepperonline.com/250w-48v-5-2a-115-230v-switching-power-supply-stepper-motor-cnc-router-kits-le-250-48) | Not auto-ranging |
| Brake Relay | [3V3 control, 24VDC 1A](https://www.aliexpress.us/item/3256807412441110.html) | |
| Brake PSU | [24V 1A (24W)](https://www.aliexpress.us/item/3256807161025414.html) | |
| Enclosure | [40×30×20 cm fiberglass](https://www.aliexpress.us/item/3256802803154446.html) | Easier to drill/file than steel |
| DIN Rail Mounts | See CAD files | 3D-printed mounts for MCU, relay, PSUs, MAX-485, driver |
| Misc. | WiFi antenna, AC/DC bus bars, PowerCon + 5-pin DMX connectors, panel-mount terminals, enclosure fans + filters, wiring | Source as available |

<br>

## Winch Parts List

![Winch_on_floor](https://github.com/user-attachments/assets/b843363a-e96c-45dd-af35-f631c3de3ccf)

| Component | Part | Notes |
|------------|------|-------|
| Motor | [Dual Shaft NEMA 24 Stepper 4.2A 4Nm](https://www.omc-stepperonline.com/dual-shaft-nema-23-stepper-motor-4-2a-4nm-566oz-in-60x60x100mm-4-wires-24hs39-4204d) | Dual shaft required for brake |
| Electromagnetic Brake | [24V 2.0Nm for NEMA 23/24](https://www.omc-stepperonline.com/dc-electromagnetic-brake-24v-2-0nm-283oz-in-for-nema-23-24-stepper-motor-swb-03) | Failsafe/power loss holding |
| Gearbox | [10:1 Planetary](https://www.omc-stepperonline.com/eg-series-planetary-gearbox-gear-ratio-10-1-backlash-15-arc-min-for-8mm-shaft-nema-23-stepper-motor-eg23-g10-d8) | Low backlash, 8mm input directly from motor |
| Endstop | [ME-8166 Long Arm, N/O](https://www.aliexpress.us/item/3256804700433315.html) | Homing + safety |
| Lifting Drums | [400-12 Garage Door Cable Spools](https://www.amazon.com/dp/B09T5RJZRY) | 4.3" OD 1" ID (use 3d-printed 1/2 shaft adapters) |
| Bearings | [1/2" Bore Pillow Blocks](https://www.amazon.com/dp/B0C61FMWHY) | Self-centering |
| Slip Ring | [1/2" Bore, 6 wire, 10A](https://www.amazon.com/dp/B07XGPVKZK) | Power for light-up props |
| Shaft | [1/2" dia, 6' cold rolled](https://www.mcmaster.com/catalog/4437T11) | |
| Frame | [Engineering Drawing](https://github.com/user-attachments/files/25563765/Roxie.Sign.Winch.Drawing.for.construction.v3.pdf) | 2×4 construction + structural screws |
| Misc. | 14mm×12.7mm shaft coupler, Grade 12.9 hardware, lag bolts, motor mounts | |

<br>

## Engineering Calculations
[Original Calculation Spreadsheet](https://github.com/user-attachments/files/25564224/Chicago.Musical.Sign.Hoist.Calculations.xlsx)

### 1. Mass & Load Estimation (for ROXIE sign)
* **$A$** = $13.5 \text{ ft}^2$ (Area of 3/4" Plywood)
* **$\sigma$** = $2 \text{ lb/ft}^2$ (Density of Plywood)
* **$L$** = $16 \text{ ft}$ (Length of low profile steel Unistrut)
* **$\lambda$** = $0.8 \text{ lb/ft}$ (Linear Density of Low Profile Steel Unistrut)
* **$m_{\text{misc}}$** = $3 \text{ lb}$ (Bulbs & Wiring)

$$m_{\text{est}} = (A \cdot \sigma) + (L \cdot \lambda) + m_{\text{misc}}$$
$$m_{\text{est}} = (13.5 \cdot 2) + (16 \cdot 0.8) + 3 = \mathbf{42.8 \text{ lb}}$$
* **Design Load Limit ($m$):** $\mathbf{50 \text{ lb} \ (22.68 \text{ kg})}$ (before dynamic saftey factors)

---

### 2. Kinematics (Velocity & Gearing)
* **$\Delta y$** = $10 \text{ ft} \ (3.048 \text{ m})$ (Target Lifting Height)
* **$\Delta t$** = $15 \text{ s}$ (Target Lifting Time)
* **$d$** = $3 \text{ in} \ (0.0762 \text{ m})$ (Drum Diameter)
* **$N_{\text{motor}}$** = $500 \text{ RPM}$ (Target Stepper Speed for optimal torque)

**Required Linear Velocity:**
$$v = \frac{\Delta y}{\Delta t} = \frac{3.048}{15} = \mathbf{0.203 \text{ m/s}}$$

**Required Drum RPM:**
$$N_{\text{drum}} = \frac{60 \cdot v}{\pi \cdot d} = \frac{60 \cdot 0.203}{\pi \cdot 0.0762} = \mathbf{50.93 \text{ RPM}}$$

**Required Gear Ratio:**
$$i = \frac{N_{\text{motor}}}{N_{\text{drum}}} = \frac{500}{50.93} = \mathbf{9.82}$$
* **Selected Gearbox:** 10:1 Planetary

---

### 3. Dynamics (Torque & Power)
* **$SF$** = $2$ (Mechanical Safety Factor)
* **$g$** = $9.81 \text{ m/s}^2$
* **$\eta$** = $0.95$ (Gearbox Efficiency)

**Required Drum Torque:**
$$\tau_{\text{drum}} = (m \cdot g \cdot SF) \cdot \frac{d}{2}$$
$$\tau_{\text{drum}} = (22.68 \cdot 9.81 \cdot 2) \cdot \left(\frac{0.0762}{2}\right) = \mathbf{16.95 \text{ Nm}}$$

**Required Motor Torque:**
$$\tau_{\text{motor}} = \frac{\tau_{\text{drum}}}{i \cdot \eta} = \frac{16.95}{10 \cdot 0.95} = \mathbf{1.82 \text{ Nm}}$$
* **Selected Motor:** NEMA 24, Provides 2.2x torque headroom over required

**Required Mechanical Power:**
$$P = \tau_{\text{motor}} \cdot \left( \frac{N_{\text{motor}} \cdot 2\pi}{60} \right)$$
$$P = 1.82 \cdot \left( \frac{500 \cdot 2\pi}{60} \right) = \mathbf{95.16 \text{ W}}$$

---

### 4. Solid Mechanics (Shaft Deflection)
Checking to ensure the 2-foot driveshaft will not bend and bind the pillow block bearings under load. 



* **$E$** = $200 \text{ GPa} \ (2 \times 10^{11} \text{ Pa})$ (Young's Modulus of Steel)
* **$L_{\text{span}}$** = $2 \text{ ft} \ (0.6096 \text{ m})$ (Distance between bearings)
* **$d_{\text{shaft}}$** = $0.5 \text{ in} \ (0.0127 \text{ m})$ (Shaft Diameter)

**Area Moment of Inertia (Solid Circular Shaft):**
$$I = \frac{\pi \cdot d_{\text{shaft}}^4}{64} = \mathbf{1.28 \times 10^{-9} \text{ m}^4}$$

**Maximum Mid-Span Deflection (Worst-case point load assumption):**
$$\delta_{\text{max}} = \frac{F \cdot L_{\text{span}}^3}{48 \cdot E \cdot I}$$
$$\delta_{\text{max}} = \mathbf{0.29 \text{ mm}}$$
* **Conclusion:** Deflection is negligible; shaft is essentially rigid.

---
<br>

# Features

## Motion Control
- Smooth acceleration/deceleration
- Dual-core architecture (Core 0 dedicated to step generation)
- Non-blocking homing with configurable drop distance
- Tick-level position tracking

## DMX
- Single channel (0-255)
- Configurable address (1-512)
- 50ms dropout protection
- Optional auto-home on DMX=0
- Signal loss detection

## Safety
- Emergency stop (power cycle to clear)
- Limit switch monitoring
- Rope loop detection (E-STOP on downward limit hit)
- Auto-recovery (re-home on accidental upward limit hit)
- Electromagnetic brake control

## Web Interface
<img width="1728" height="1085" alt="Screenshot 2026-02-27 at 1 08 30 PM" src="https://github.com/user-attachments/assets/017abb60-902c-44ac-8a21-a5341644cbf2" />
Access at http://DMXWinch.local if mDNS is supported on the network, else use the IP address assigned by DHCP.

- Real-time position display
- Stroke length, velocity, acceleration adjustment
- Brake modes (stop-on-zero, always-brake, motor-hold)
- Homing configuration
- Manual positioning for setup
- Persistent settings (survives power loss)
- Instant emergency stop button
- Build using the ESP-DASH library

## Status LED (NeoPixel, inside controll cabinet)

- 🔴 Red blink: E-STOP
- 🟣 Purple: Homing
- 🟢 Green: DMX active
- 🟡 Yellow blink: DMX signal lost
- 🟠 Orange: Manual mode
- 🔵 Cyan: Waiting for DMX
- 🔵 Blue: WiFi connecting

## DMX Mapping

- **0**: Auto-home (if enabled)
- **1**: Top (0")
- **255**: Bottomn (max stroke)
- Linear between 1-255

## Homing Sequence

1. Move up until the limit switch
2. Drop configured distance (default 2")
3. Set zero at the drop point
4. Repeat drop if still on limit switch

# Configuration

## Parameters
Edit WiFi in `main.cpp`:
```cpp
const char* ssid = "YourNetwork";
const char* password = "YourPassword";
```
Adjust other parameters in `main.cpp` as needed to match the physical setup.

# Safety Notes

## Automatic E-STOP triggers:
- Limit hit during downward motion (rope loop)
- Manual E-STOP pressed
- Position exceeds 1.5× stroke during homing

## Automatic recovery:
- Limit hit during upward motion (re-homes)
- DMX=0 received (if auto-home enabled)
