# BMS CAN Configuration Reference

## CAN ID Format

All IDs are 29-bit extended format. Module ID is encoded in bits 15:12.

**Base ID + (Module ID << 12)** → Final CAN ID

Example: Module 5 config command = `0x08F00F00 | (5 << 12)` = `0x08F05F00`

---

## Configuration Commands

Send to: `0x08F0XF00` (where X = target module ID)

| Command | Byte 0 | Byte 1 | Description |
|---------|--------|--------|-------------|
| Set Module ID | `0x01` | New ID (0-15) | Sets module ID, saves to flash, triggers reset |
| Set Max Temp | `0x02` | Temp (°C) | Sets max thermistor temp threshold (0-127°C) |
| Set Min Temp | `0x03` | Temp (°C, signed) | Sets min thermistor temp threshold (-128 to 127°C) |
| Set Min Voltage | `0x04` | Value (×100=mV) | Sets min cell voltage (e.g., 25 = 2500mV) |
| Set Max Voltage | `0x05` | Value (×100=mV) | Sets max cell voltage (e.g., 42 = 4200mV) |

**Note:** Temperature and voltage thresholds are temporary settings that reset to defaults on power cycle.

### Config ACK Response

Sent from: `0x08F0XF01` (where X = module ID)

| Byte | Description |
|------|-------------|
| 0 | Command echo |
| 1 | Status: `0x00`=Success, `0x01`=Fail, `0x02`=Success/Reset Required |
| 2 | Old value |
| 3 | New value (actual, after validation) |
| 4-7 | Reserved |

---

## Control Commands

| Message | CAN ID Base | Data | Response |
|---------|-------------|------|----------|
| STM32 Reset | `0x08F0XF02` | None required | None (device resets immediately) |
| BQ76952 Chip Reset | `0x08F0XF03` | None required | ACK on `0x08F0XF04` |

### BMS Chip Reset ACK

Sent from: `0x08F0XF04`

| Byte | Description |
|------|-------------|
| 0 | Status: `0x00`=Queued, `0x01`=Fail/Already Pending |
| 1-7 | Reserved |

---

## Quick Reference

| Function | Send To | Data | Notes |
|----------|---------|------|-------|
| Set Module ID to 5 | `0x08F0XF00` | `01 05` | Device resets after ACK |
| Set Max Temp to 55°C | `0x08F0XF00` | `02 37` | Temporary until reset |
| Set Min Temp to -20°C | `0x08F0XF00` | `03 EC` | Temporary until reset (0xEC = -20 signed) |
| Set Min Voltage to 2500mV | `0x08F0XF00` | `04 19` | Temporary until reset (25 × 100 = 2500) |
| Set Max Voltage to 4200mV | `0x08F0XF00` | `05 2A` | Temporary until reset (42 × 100 = 4200) |
| Reset STM32 | `0x08F0XF02` | — | Immediate reset |
| Reset BQ76952 chips | `0x08F0XF03` | — | ~600ms reset sequence |

---

## Default Values

| Parameter | Default | Range |
|-----------|---------|-------|
| Max Temp Threshold | 60°C | 0-127°C |
| Min Temp Threshold | -20°C | -128 to 127°C |
| Min Cell Voltage | 2500mV | 0-25500mV |
| Max Cell Voltage | 4200mV | 0-25500mV |
