# Hardware Specification
ESP32-S3 + DRV8825 Stepper Controller (4ch)

---

# Overview

Create a KiCad 7 hierarchical schematic and PCB project.

Target system:

- ESP32-S3-WROOM-1 MCU
- DRV8825 StepStick compatible modules ×4
- Quadrature Encoder ×4
- I2C bus ×1
- ADC input ×4
- Native USB
- 24V motor supply

DRV8825 modules are standard StepStick form factor.

Reference:
DRV8825 modules operate at 8.2–45V motor supply and support up to ~1.5A continuous per phase without extra cooling.  
(See DRV8825 carrier specifications.)  
Source: https://www.pololu.com/product/2133 :contentReference[oaicite:0]{index=0}

---

# Project Structure

Create hierarchical sheets:
