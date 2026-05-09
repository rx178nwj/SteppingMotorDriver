# Schematic Plan

Status: APPROVED

Project: `SteppingMotorDriver`
Root schematic: `SteppingMotorDriver.kicad_sch`
Page size: A3
KiCad version target: 7+ compatible structure

## Goal

Create a hierarchical schematic for a 4-channel stepper motor controller based on:

- ESP32-S3-WROOM-1
- 4x DRV8825 StepStick-compatible plug-in modules
- 24V motor input
- Buck 24V to 5V
- LDO 5V to 3V3
- Native USB-C
- 4 quadrature encoder inputs
- I2C expansion connector
- 4 filtered ADC inputs

## Hierarchy

Root sheet:

- Power
- MCU_USB
- Drivers
- Encoders_IO
- ADC_IO

Suggested child files:

- `power.kicad_sch`
- `mcu_usb.kicad_sch`
- `drivers.kicad_sch`
- `encoders_io.kicad_sch`
- `adc_io.kicad_sch`

## Global Nets

Power:

- `VIN_24V`
- `+5V`
- `+3V3`
- `GND`

USB:

- `USB_D+`
- `USB_D-`
- `USB_CC1`
- `USB_CC2`

Shared driver control:

- `DRV_EN`
- `DRV_RESET`
- `DRV_SLEEP`
- `MS0`
- `MS1`
- `MS2`

Per-channel motor control:

- `STEP0`, `DIR0`
- `STEP1`, `DIR1`
- `STEP2`, `DIR2`
- `STEP3`, `DIR3`

Per-channel encoder:

- `ENC0_A`, `ENC0_B`
- `ENC1_A`, `ENC1_B`
- `ENC2_A`, `ENC2_B`
- `ENC3_A`, `ENC3_B`

I2C:

- `I2C_SDA`
- `I2C_SCL`

ADC:

- `ADC0`
- `ADC1`
- `ADC2`
- `ADC3`

## Root Sheet Connectivity

Root only contains hierarchical sheet symbols and top-level power net labels.

Sheet pins:

Power:

- in: `VIN_24V`
- out: `+5V`
- out: `+3V3`
- passive: `GND`

MCU_USB:

- in: `+3V3`
- passive: `GND`
- bidirectional: `I2C_SDA`
- bidirectional: `I2C_SCL`
- input: `ENC0_A`, `ENC0_B`, `ENC1_A`, `ENC1_B`, `ENC2_A`, `ENC2_B`, `ENC3_A`, `ENC3_B`
- input: `ADC0`, `ADC1`, `ADC2`, `ADC3`
- output: `STEP0`, `DIR0`, `STEP1`, `DIR1`, `STEP2`, `DIR2`, `STEP3`, `DIR3`
- output: `DRV_EN`, `DRV_RESET`, `DRV_SLEEP`
- output: `MS0`, `MS1`, `MS2`
- bidirectional: `USB_D+`, `USB_D-`

Drivers:

- in: `VIN_24V`
- in: `+3V3`
- passive: `GND`
- input: `STEP0`, `DIR0`, `STEP1`, `DIR1`, `STEP2`, `DIR2`, `STEP3`, `DIR3`
- input: `DRV_EN`, `DRV_RESET`, `DRV_SLEEP`
- input: `MS0`, `MS1`, `MS2`

Encoders_IO:

- in: `+3V3`
- passive: `GND`
- output: `ENC0_A`, `ENC0_B`, `ENC1_A`, `ENC1_B`, `ENC2_A`, `ENC2_B`, `ENC3_A`, `ENC3_B`

ADC_IO:

- in: `+3V3`
- passive: `GND`
- output: `ADC0`, `ADC1`, `ADC2`, `ADC3`
- input: `AIN0`, `AIN1`, `AIN2`, `AIN3`

## Sheet Plan: Power

### Functional blocks

1. `VIN_24V` input connector
2. Bulk input capacitor near connector
3. Buck regulator: MP1584 preferred, LM2596 acceptable fallback
4. 5V bulk and ceramic output capacitors
5. LDO regulator: AP2112K-3.3 preferred, AMS1117-3.3 fallback
6. 3V3 bulk and ceramic output capacitors
7. Power flags on `VIN_24V`, `+5V`, and `+3V3`

### Nets

- Connector positive pin to `VIN_24V`
- Connector negative pin to `GND`
- Buck input to `VIN_24V`
- Buck output to `+5V`
- LDO input to `+5V`
- LDO output to `+3V3`

### Notes

- Keep `VIN_24V` and buck input caps grouped tightly.
- Add one `PWR_FLAG` each on `VIN_24V`, `+5V`, and `+3V3`.

## Sheet Plan: MCU_USB

### Main device

- ESP32-S3-WROOM-1 module symbol with footprint assigned to the chosen module footprint in the project library set

### Support circuits

1. `EN` pull-up: 10k to `+3V3`
2. `EN` capacitor: 0.1uF to `GND`
3. `GPIO0` boot button to `GND`
4. `GPIO0` pull-up: 10k to `+3V3`
5. Local 0.1uF decoupling on each module supply pin group
6. USB-C receptacle for device mode
7. `CC1` and `CC2` each 5.1k to `GND`
8. USB ESD protection array on D+ and D-
9. I2C pull-ups: 4.7k from `I2C_SDA` and `I2C_SCL` to `+3V3`

### GPIO mapping

- `GPIO4` -> `STEP0`
- `GPIO5` -> `DIR0`
- `GPIO6` -> `STEP1`
- `GPIO7` -> `DIR1`
- `GPIO8` -> `STEP2`
- `GPIO9` -> `DIR2`
- `GPIO10` -> `STEP3`
- `GPIO11` -> `DIR3`
- `GPIO12` -> `DRV_EN`
- `GPIO13` -> `DRV_RESET`
- `GPIO14` -> `DRV_SLEEP`
- `GPIO15` -> `ENC0_A`
- `GPIO16` -> `ENC0_B`
- `GPIO17` -> `ENC1_A`
- `GPIO18` -> `ENC1_B`
- `GPIO19` -> `USB_D-`
- `GPIO20` -> `USB_D+`
- `GPIO21` -> `ENC2_A`
- `GPIO35` -> `ENC2_B`
- `GPIO36` -> `ENC3_A`
- `GPIO37` -> `ENC3_B`
- `GPIO38` -> `I2C_SDA`
- `GPIO39` -> `I2C_SCL`
- `GPIO1` -> `ADC0`
- `GPIO2` -> `ADC1`
- `GPIO3` -> `ADC2`
- `GPIO40` -> `ADC3`

### Microstep default

`MS0`, `MS1`, `MS2` are driven by a 3-position DIP switch bank that selects each net to either `GND` or `+3V3`.
Default state is LOW via pull-down or direct switch-to-GND default orientation.

## Sheet Plan: Drivers

### Channel replication

Create 4 identical driver channels named:

- `DRV8825_CH0`
- `DRV8825_CH1`
- `DRV8825_CH2`
- `DRV8825_CH3`

### Per-channel socket

Use socket footprint:

- `Connector_PinSocket_2.54mm:PinSocket_2x08_P2.54mm_Vertical`

### Required logical pins per module

Control side:

- `EN`
- `M0`
- `M1`
- `M2`
- `RESET`
- `SLEEP`
- `STEP`
- `DIR`

Power/motor side:

- `VMOT`
- `GND`
- `B2`
- `B1`
- `A1`
- `A2`
- `FAULT`
- `GND`
- `VDD`

### Shared connections

All 4 channels:

- `EN` -> `DRV_EN`
- `M0` -> `MS0`
- `M1` -> `MS1`
- `M2` -> `MS2`
- `RESET` -> `DRV_RESET`
- `SLEEP` -> `DRV_SLEEP`
- `VMOT` -> `VIN_24V`
- `VDD` -> `+3V3`
- both ground pins -> `GND`

Per-channel unique connections:

- CH0 `STEP` -> `STEP0`, `DIR` -> `DIR0`
- CH1 `STEP` -> `STEP1`, `DIR` -> `DIR1`
- CH2 `STEP` -> `STEP2`, `DIR` -> `DIR2`
- CH3 `STEP` -> `STEP3`, `DIR` -> `DIR3`

### Per-channel support parts

For each channel:

1. 100uF electrolytic capacitor from `VIN_24V` to `GND`, placed adjacent to module
2. 4-pin motor connector exposing `A1`, `A2`, `B1`, `B2`
3. Optional pull-up for `FAULT` only if required by selected system behavior; otherwise expose as no-connect

### No-connect handling

If `FAULT` is unused, explicitly mark it `no_connect`.

## Sheet Plan: Encoders_IO

For each of 4 channels:

1. 5-pin connector with `+3V3`, `GND`, `A`, `B`, and optional `Z`
2. Connect `A` and `B` to the corresponding `ENCx_A` / `ENCx_B`
3. Leave `Z` intentionally unconnected and mark `no_connect`

Optional note text:

- "Use 74LVC14 level shifting if encoder outputs are 5V."

## Sheet Plan: ADC_IO

For each input channel 0 to 3:

1. 2-pin input connector: `AINx`, `GND`
2. Divider top resistor 20k from `AINx` to filtered node
3. Divider bottom resistor 10k from filtered node to `GND`
4. 0.1uF capacitor from filtered node to `GND`
5. Filtered node connects to MCU net `ADCx`

## Footprint Intent

Assign footprints during implementation:

- USB-C receptacle: ESD-capable USB2.0 device receptacle footprint available in installed libraries
- Buck and LDO: choose footprints matching selected symbols
- Encoder connectors: JST-XH or equivalent 2.54mm header if no exact connector constraint exists
- Motor connectors: JST-VH or 5.08mm screw terminal, 4 positions
- DIP switch: 3-position through-hole or SMD switch footprint
- Electrolytic capacitor footprint sized for at least 100uF VMOT decoupling
- Mounting holes: 3.2mm NPTH or mounting-hole footprint in PCB stage

## Annotation / ERC Targets

- No `?` references remain
- No unconnected required pins remain
- `FAULT` pins and encoder `Z` pins may be marked with no-connect flags
- Zero ERC violations after adding required `PWR_FLAG`s

## Execution Notes

- Prefer hierarchical labels over root-sheet daisy-chaining.
- Keep DRV8825 channels visually uniform and aligned.
- Place USB and MCU support parts tightly around the MCU block.
- Keep VMOT decoupling local to each driver socket.
