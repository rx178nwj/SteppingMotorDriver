# Hardware Specification
ESP32-S3 + DRV8825 Stepper Controller (3ch)

---

# Overview

KiCad 9 hierarchical schematic and PCB project.

Target system:

- ESP32-S3-WROOM-1 MCU
- DRV8825 StepStick compatible modules ×3
- Quadrature Encoder with A/B/Z phases ×3 (differential, AM26LV32 receiver)
- I2C bus ×1
- ADC input ×5 (all ADC1): POT×3, CURRENT×1, VIN_MON×1
- Native USB
- 24V motor supply

DRV8825 modules are standard StepStick form factor.

Reference:
DRV8825 modules operate at 8.2–45V motor supply and support up to ~1.5A continuous per phase without extra cooling.

---

# Project Structure

Hierarchical sheets:

- SteppingMotorDriver.kicad_sch (root)
  - mcu_usb.kicad_sch (MCU, USB, power regulators, LEDs)
  - drivers.kicad_sch (3× DRV8825 modules, motor connectors)
  - encoders_io.kicad_sch (3× AM26LV32 differential receivers, encoder connectors)
  - adc_io.kicad_sch (5ch ADC: 3× potentiometer + current sense + VBUS monitor)
  - power.kicad_sch (24V → Buck → 5V → LDO → 3.3V)

---

# GPIO Assignment

## ADC Inputs (ADC1 only — no ADC2)

POT0    → GPIO1  (ADC1_CH0)
POT1    → GPIO2  (ADC1_CH1)
POT2    → GPIO3  (ADC1_CH2)
CURRENT → GPIO4  (ADC1_CH3) — current sense via shunt + op-amp
VIN_MON→ GPIO5  (ADC1_CH4) — 24V supply monitor via voltage divider

## STEP/DIR (3 channels)

STEP0 → GPIO6
DIR0  → GPIO7

STEP1 → GPIO8
DIR1  → GPIO9

STEP2 → GPIO10
DIR2  → GPIO11

## Shared DRV8825 Control

DRV_EN    → GPIO12
DRV_RESET → GPIO13
DRV_SLEEP → GPIO14

## Microstep Selection (GPIO controlled)

M0 → GPIO41
M1 → GPIO42
M2 → GPIO45

## Encoder Inputs (A/B/Z phases, 3ch)

ENC0_A → GPIO15
ENC0_B → GPIO16
ENC0_Z → GPIO17

ENC1_A → GPIO18
ENC1_B → GPIO21
ENC1_Z → GPIO35

ENC2_A → GPIO36
ENC2_B → GPIO37
ENC2_Z → GPIO40

## USB (fixed)

USB_D- → GPIO19
USB_D+ → GPIO20

## I2C (fixed)

SDA → GPIO38
SCL → GPIO39

## UART (debug)

TX → GPIO43
RX → GPIO44

---

# Motor Driver Modules

Use DRV8825 StepStick compatible modules.
3 modules total: DRV8825_CH0, DRV8825_CH1, DRV8825_CH2

Socket footprint: PinSocket_2x08_P2.54mm_Vertical

DRV8825 Standard StepStick layout:
Left side:  EN, M0, M1, M2, RESET, SLEEP, STEP, DIR
Right side: VMOT, GND, B2, B1, A1, A2, FAULT, GND, VDD

Shared signals (all 3 channels):
- EN    → DRV_EN    (GPIO12)
- RESET → DRV_RESET (GPIO13)
- SLEEP → DRV_SLEEP (GPIO14)
- M0    → M0        (GPIO41)
- M1    → M1        (GPIO42)
- M2    → M2        (GPIO45)

Per-channel:
- CH0: STEP=STEP0(GPIO6), DIR=DIR0(GPIO7)
- CH1: STEP=STEP1(GPIO8), DIR=DIR1(GPIO9)
- CH2: STEP=STEP2(GPIO10), DIR=DIR2(GPIO11)

---

# Motor Power

Input: VIN_24V

For each DRV8825: 100µF electrolytic decoupling cap between VMOT and GND.

---

# Motor Connectors

For each channel: 4-pin JST-XH connector
Pins: A1, A2, B1, B2

---

# Encoder Interface

3× AM26LV32 quad differential line receiver
Each IC handles A+/A-, B+/B-, Z+/Z- for one encoder channel

Input (differential): ENC_n_EA+/-, ENC_n_EB+/-, ENC_n_EZ+/-
Output (single-ended to MCU): ENC0_A/B/Z, ENC1_A/B/Z, ENC2_A/B/Z

Encoder connector (JST-XH 6-pin per channel):
- VCC (5V)
- GND
- A+ / A-
- B+ / B-
- Z+ / Z-

---

# ADC Inputs

## Potentiometers (POT0, POT1, POT2)

Voltage divider: 20kΩ (top) / 10kΩ (bottom)
RC filter: 0.1µF to GND
Input range: 0–3.3V

## Current Sense (CURRENT)

Shunt resistor (low-side) + op-amp gain stage
Output conditioned to 0–3.3V range for ADC1_CH3 (GPIO4)

## Motor Supply Monitor (VIN_MON)

Monitors VIN_24V (motor power input) via resistor voltage divider.
R_top = 68kΩ, R_bot = 10kΩ → 24V × 10/(68+10) ≈ 3.08V (within 3.3V ADC range)
Max safe input: 3.3V × 7.8 ≈ 25.7V
RC filter: 0.1µF to GND → ADC1_CH4 (GPIO5)
Conversion: VIN_24V[V] = ADC_raw × 3.3 / 4095 × 7.8

---

# Logic Power

VIN_24V → Buck (MP1584 or LM2596) → 5V
5V → LDO (AP2112K or AMS1117-3.3) → 3.3V

---

# I2C Connector

Pins: 3V3, GND, SDA (GPIO38), SCL (GPIO39)
Pull-ups: 4.7kΩ to 3V3

---

# USB Interface

Native USB: GPIO19 = USB D−, GPIO20 = USB D+
Connector: USB-C receptacle
CC1 → 5.1kΩ → GND
CC2 → 5.1kΩ → GND
Add USB ESD protection (NCP361SN or equivalent)

---

# PCB Layout Rules

Motor power tracks: ≥ 1.0mm
24V input: ≥ 1.0mm
5V: ≥ 0.5mm
3V3: ≥ 0.3mm
Signals: ≥ 0.2mm
USB: differential pair routing

Use solid GND plane. Separate motor current paths. Keep VMOT loops short.

Add silkscreen labels: CH0, CH1, CH2
Add mounting holes ×4 (3.2mm)

---

# End of specification
