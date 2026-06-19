
Generate:

- .kicad_pro
- .kicad_sch
- .kicad_pcb

---

# MCU

Device:

ESP32-S3-WROOM-1

Power:

3V3 → VDD  
GND → GND  

Enable circuit:

EN → 10kΩ → 3V3  
EN → 0.1µF → GND  

Boot:

GPIO0 → Push Button → GND  
GPIO0 → 10kΩ → 3V3  

---

# USB Interface

Native USB:

GPIO19 → USB D−  
GPIO20 → USB D+

Connector:

USB-C receptacle

Add:

CC1 → 5.1kΩ → GND  
CC2 → 5.1kΩ → GND  

Add USB ESD protection.

---

# Motor Driver Modules

Use:

DRV8825 StepStick compatible modules.

DO NOT place DRV8825 IC directly.

Use socket footprint:

PinSocket_2x08_P2.54mm_Vertical

3 modules total:

DRV8825_CH0  
DRV8825_CH1  
DRV8825_CH2  

---

# DRV8825 Pin Mapping

Standard StepStick layout:

Left side:

EN  
M0  
M1  
M2  
RESET  
SLEEP  
STEP  
DIR  

Right side:

VMOT  
GND  
B2  
B1  
A1  
A2  
FAULT  
GND  
VDD  

---

# GPIO Assignment

## ADC Inputs (all ADC1)

POT0     → GPIO1  (ADC1_CH0)
POT1     → GPIO2  (ADC1_CH1)
POT2     → GPIO3  (ADC1_CH2)
CURRENT  → GPIO4  (ADC1_CH3)
VIN_MON → GPIO5  (ADC1_CH4)

---

## STEP/DIR (3 channels)

STEP0 → GPIO6  
DIR0  → GPIO7  

STEP1 → GPIO8  
DIR1  → GPIO9  

STEP2 → GPIO10  
DIR2  → GPIO11  

---

## Shared Control

DRV_EN    → GPIO12  
DRV_RESET → GPIO13  
DRV_SLEEP → GPIO14  

---

## Microstep Selection

M0 → GPIO41  
M1 → GPIO42  
M2 → GPIO45  

---

## Encoder Inputs (A/B/Z, 3ch)

ENC0_A → GPIO15  
ENC0_B → GPIO16  
ENC0_Z → GPIO17  

ENC1_A → GPIO18  
ENC1_B → GPIO21  
ENC1_Z → GPIO35  

ENC2_A → GPIO36  
ENC2_B → GPIO37  
ENC2_Z → GPIO40  

Differential encoder input via AM26LV32 line receiver.

---

## I2C

SDA → GPIO38  
SCL → GPIO39  

Add:

4.7kΩ pull-up resistors to 3V3.

---

## UART (debug)

TX → GPIO43  
RX → GPIO44  

---

# ADC Inputs

## Potentiometer (POT0, POT1, POT2)

Voltage divider:

20kΩ (top)  
10kΩ (bottom)

RC filter:

0.1µF to GND.

---

## Current Sense (CURRENT)

Shunt resistor + op-amp gain stage.
Output conditioned to 0–3.3V → GPIO4 (ADC1_CH3).

---

## Power Supply Monitor (VIN_MON)

Voltage divider to scale 24V → 0–3.3V.
RC filter: 0.1µF to GND → GPIO5 (ADC1_CH4).

---

# Motor Power

Input:

VIN_24V

For each DRV8825:

Add:

100µF electrolytic capacitor

Between:

VMOT  
GND  

Must be placed close to module.

---

# Logic Power

VIN_24V → Buck → 5V  

5V → LDO → 3V3  

Recommended:

Buck:

MP1584  
or LM2596  

LDO:

AP2112K  
or AMS1117-3.3  

---

# Motor Connectors

For each channel:

Connector:

4-pin JST-XH

Pins:

A1  
A2  
B1  
B2  

---

# Encoder Connectors

For each channel:

Connector: JST-XH 6-pin

Pins:

VCC (5V)  
GND  
A+  
A-  
B+  
B-  
Z+  
Z-  

Note: connector is 8-pin for full differential (A+/A-, B+/B-, Z+/Z-)

Differential input through AM26LV32 line receiver IC.

---

# I2C Connector

Pins:

3V3  
GND  
SDA  
SCL  

---

# PCB Layout Rules

Place:

ESP32-S3 center.

DRV8825 modules on right side.

Motor connectors on board edge.

Encoder connectors on left side.

USB-C bottom edge.

Power input top edge.

---

# Routing Rules

Motor power:

Track width ≥ 1.0mm

24V power:

≥ 1.0mm

5V:

≥ 0.5mm

3V3:

≥ 0.3mm

Signals:

≥ 0.2mm

USB:

Differential pair routing.

---

# Grounding

Use:

Solid GND plane.

Separate:

Motor current paths.

Keep:

VMOT loops short.

---

# Required Outputs

Generate:

- Full KiCad project
- Schematics
- PCB layout
- Net labels
- Footprints assigned

Project must be ready for PCB fabrication.

---

# Additional Requirements

Add silkscreen labels:

CH0  
CH1  
CH2  

Add:

Mounting holes ×4

Hole size:

3.2mm

Add:

Board outline.

---

# End of specification
