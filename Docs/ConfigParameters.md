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
| Get Value | `0x06` | Param selector | Retrieves current value (see table below) |
| Set BQ Mode | `0x07` | Mode (0 or 1) | Sets BQ76952 power mode: 0=NORMAL, 1=SLEEP |

**Note:** Temperature and voltage thresholds are temporary settings that reset to defaults on power cycle.

### Parameter Selectors for Get Value (0x06)

| Selector | Byte 1 | Returns |
|----------|--------|---------|
| Module ID | `0x01` | Current module ID (0-15) |
| Max Temp | `0x02` | Max temperature threshold (°C) |
| Min Temp | `0x03` | Min temperature threshold (°C, signed) |
| Min Voltage | `0x04` | Min voltage threshold (value × 100 = mV) |
| Max Voltage | `0x05` | Max voltage threshold (value × 100 = mV) |
| BQ Mode | `0x06` | BQ76952 power mode (see BQ Mode ACK format below) |
| BQ Normal Read Interval | `0x07` | BQ normal mode cell read interval (ms, 16-bit) |
| BQ Normal CAN Interval | `0x08` | BQ normal mode CAN reporting interval (ms, 16-bit) |
| BQ Sleep Read Interval | `0x09` | BQ sleep mode cell read interval (ms, 16-bit) |
| BQ Sleep CAN Interval | `0x0A` | BQ sleep mode CAN reporting interval (ms, 16-bit) |
| I2C Timeout | `0x0B` | I2C communication timeout (ms) |
| Balance Cmd Timeout | `0x0C` | Balance command timeout (ms, 16-bit) |
| Balance Reevaluate | `0x0D` | Balance cell re-evaluation interval (ms, 16-bit) |
| Balance Refresh | `0x0E` | Balance CB_ACTIVE_CELLS refresh interval (ms, 16-bit) |
| Balance OCV Settle | `0x0F` | Balance OCV settling delay (ms, 16-bit) |
| Balance Status Interval | `0x10` | Balance status CAN report interval (ms, 16-bit) |
| CAN Heartbeat Interval | `0x11` | CAN heartbeat interval (ms, 16-bit) |
| Temp Summary Interval | `0x12` | Temperature summary CAN report interval (ms, 16-bit) |
| CAN TX Timeout | `0x13` | CAN TX queue timeout (ms) |

### Config ACK Response (Set Commands)

Sent from: `0x08F0XF01` (where X = module ID)

| Byte | Description |
|------|-------------|
| 0 | Command echo |
| 1 | Status: `0x00`=Success, `0x01`=Fail, `0x02`=Success/Reset Required |
| 2 | Old value |
| 3 | New value (actual, after validation) |
| 4-7 | Reserved |

### Config ACK Response (Set BQ Mode Command 0x07)

Sent from: `0x08F0XF01` (where X = module ID)

| Byte | Description |
|------|-------------|
| 0 | Command echo (`0x07`) |
| 1 | Status: `0x00`=Success, `0x01`=Fail |
| 2 | BMS1 actual mode (0=NORMAL, 1=SLEEP) — read from hardware |
| 3 | BMS2 actual mode (0=NORMAL, 1=SLEEP) — read from hardware |
| 4 | Requested mode |
| 5-7 | Reserved |

### Config ACK Response (Get Value Command)

Sent from: `0x08F0XF01` (where X = module ID)

| Byte | Description |
|------|-------------|
| 0 | Command echo (`0x06`) |
| 1 | Status: `0x00`=Success, `0x01`=Invalid parameter |
| 2 | Parameter selector echo |
| 3 | Current value (low byte) |
| 4 | Current value (high byte, for 16-bit values) |
| 5-7 | Reserved |

### Config ACK Response (Get BQ Mode — param 0x06)

Sent from: `0x08F0XF01` (where X = module ID)

| Byte | Description |
|------|-------------|
| 0 | Command echo (`0x06`) |
| 1 | Status: `0x00`=Success, `0x01`=Fail |
| 2 | Parameter selector echo (`0x06`) |
| 3 | BMS1 actual mode (0=NORMAL, 1=SLEEP) — read from hardware |
| 4 | BMS2 actual mode (0=NORMAL, 1=SLEEP) — read from hardware |
| 5 | Cached/commanded mode |
| 6-7 | Reserved |

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
| Get Max Temp | `0x08F0XF00` | `06 02` | Returns current max temp threshold |
| Get Min Voltage | `0x08F0XF00` | `06 04` | Returns current min voltage threshold |
| Set BQ NORMAL | `0x08F0XF00` | `07 00` | Sets both chips to NORMAL mode |
| Set BQ SLEEP | `0x08F0XF00` | `07 01` | Sets both chips to SLEEP mode |
| Get BQ Mode | `0x08F0XF00` | `06 06` | Returns actual mode from both chips |
| Get BQ Normal Read Int | `0x08F0XF00` | `06 07` | Returns 500 (ms, 16-bit: byte3=low, byte4=high) |
| Get BQ Sleep Read Int | `0x08F0XF00` | `06 09` | Returns 5000 (ms, 16-bit) |
| Get Balance Reevaluate | `0x08F0XF00` | `06 0D` | Returns 40000 (ms, 16-bit) |
| Get CAN Heartbeat Int | `0x08F0XF00` | `06 11` | Returns 1000 (ms, 16-bit) |
| Get Temp Summary Int | `0x08F0XF00` | `06 12` | Returns 5000 (ms, 16-bit) |
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
| BQ Power Mode | SLEEP | NORMAL (0) or SLEEP (1) |

### Firmware Intervals and Timeouts (Read-Only)

| Parameter | Value | Description |
|-----------|-------|-------------|
| BQ Normal Read Interval | 500ms | Cell voltage read interval in NORMAL mode |
| BQ Normal CAN Interval | 500ms | CAN report interval in NORMAL mode |
| BQ Sleep Read Interval | 5000ms | Cell voltage read interval in SLEEP mode |
| BQ Sleep CAN Interval | 5000ms | CAN report interval in SLEEP mode |
| I2C Timeout | 100ms | I2C communication timeout |
| Balance Command Timeout | 5000ms | Balance command timeout |
| Balance Reevaluate | 40000ms | Cell selection re-evaluation period |
| Balance Refresh | 5000ms | CB_ACTIVE_CELLS refresh period |
| Balance OCV Settle | 10000ms | OCV settling delay before cell selection |
| Balance Status Interval | 1000ms | Balance status CAN report interval |
| CAN Heartbeat Interval | 1000ms | CAN heartbeat interval |
| Temp Summary Interval | 5000ms | Temperature summary CAN report interval |
| CAN TX Timeout | 100ms | CAN TX queue timeout |

### BQ Power Mode Notes

- **Default at startup**: SLEEP (configured via `BQ_DEFAULT_POWER_MODE` in `bq_handler.h`)
- **NORMAL mode**: Cells read every 500ms, CAN reports every 500ms
- **SLEEP mode**: Cells read every 5000ms (wake-read-sleep pattern), CAN reports every 5000ms
- Both chips are always kept in the same mode
- Balancing automatically forces NORMAL mode; previous mode is restored when balancing stops
- The mode setting is temporary (resets to default on power cycle)
