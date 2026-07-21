# Sabo — Bill of Materials & Cost Estimate

Self-sourced build. Prices = typical maker USD (~2025–26), low–high (vendor/region vary). Estimate for planning, not a live cart.

## Compute

| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |
|---|--:|--:|--:|---|
| Jetson Orin Nano Super Dev Kit (8 GB) | 1 | 249–249 | 249–249 | on-device AI brain; incl. carrier+cooling. (module+compact carrier ~similar) |
| NVMe SSD 256 GB (M.2) | 1 | 22–35 | 22–35 | OS + models |
| Wi-Fi/BT M.2 card (AX210) | 1 | 15–22 | 15–22 | app link / telemetry |
| **Compute subtotal** | | | **286–306** | |

## Sensors

| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |
|---|--:|--:|--:|---|
| Wide-FOV CSI camera (IMX219, ~120°) | 2 | 25–35 | 50–70 | eyes — stereo pair |
| BNO085 IMU (fusion) | 1 | 15–25 | 15–25 | inner ear — balance + gimbal + EIS |
| VL53L1X ToF distance | 2 | 10–16 | 20–32 | nose (fwd) + chin (cliff) |
| I2S MEMS mic (ICS-43434) | 2 | 6–9 | 12–18 | ears — stereo hearing |
| BME688 gas/VOC e-nose | 1 | 18–28 | 18–28 | nose — scent classifier |
| MAX98357A I2S amp | 1 | 5–8 | 5–8 | mouth — audio out |
| Mini speaker 8Ω | 1 | 2–5 | 2–5 | mouth — meow/trill/TTS |
| **Sensors subtotal** | | | **122–186** | |

## Actuators

| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |
|---|--:|--:|--:|---|
| Feetech STS3215 serial bus servo (30 kg·cm) | 14 | 14–18 | 196–252 | 14 joints: 8 leg + waist + head pan/pitch/tilt + ears + tail; TTL daisy-chain, position feedback, torque control |
| TTL bus servo adapter (Waveshare / FE-URT-1) | 1 | 5–12 | 5–12 | UART↔half-duplex TTL bus for the STS3215 chain (replaces PCA9685) |
| LED-eye driver (MOSFET + eye LEDs) | 1 | 2–6 | 2–6 | eyes off the servo bus → Jetson hardware-PWM pin + MOSFET |
| **Actuators subtotal** | | | **203–270** | |

## Power

| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |
|---|--:|--:|--:|---|
| 3S LiPo 5000 mAh | 1 | 25–40 | 25–40 | ~1.5–2 h active |
| Buck 5 V/5 A (Jetson rail) | 1 | 8–14 | 8–14 |  |
| Buck/BEC 7.4 V/≥15 A (STS3215 bus rail) | 1 | 12–22 | 12–22 | separate rail + bulk cap; sized for realistic simultaneous servo current |
| Bulk cap + XT60 + wiring/connectors | 1 | 15–30 | 15–30 | power distribution |
| **Power subtotal** | | | **60–106** | |

## Mechanical

| Item | Qty | Unit $ (lo–hi) | Subtotal $ | Note |
|---|--:|--:|--:|---|
| M2/M3 screws + heat-set inserts | 1 | 10–18 | 10–18 | assembly |
| Servo horns / pins / small bearings | 1 | 12–25 | 12–25 | joint hardware |
| TPU for foot pads | 1 | 5–10 | 5–10 | grippy toe caps |
| Faux-fur / silicone skin (optional) | 1 | 0–25 | 0–25 | cosmetic over-skin, PLAN §3.3 (optional) |
| 3D-print filament (~605 g PLA/PETG) | 1 | 12–17 | 12–17 | computed from CAD mass |
| **Mechanical subtotal** | | | **39–95** | |

## Total (one robot)

| | Low | Mid | High |
|---|--:|--:|--:|
| **Build cost (USD)** | **$710** | **$836** | **$963** |

### One-time tools (excluded from build cost)

- LiPo balance charger: $20–40
- 3D printer: $0–0 (assumed owned)
- Soldering iron + supplies: $0–0 (assumed owned)

_Cost drivers: 14× STS3215 servos and the Jetson dominate (~57% of a mid build). Cutting servo count or grade, or a cheaper SBC, moves the total most._
