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
| Set Max Temp | `0x02` | Temp (°C) | Sets max thermistor temp threshold (temporary, resets on power cycle) |

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
| Reset STM32 | `0x08F0XF02` | — | Immediate reset |
| Reset BQ76952 chips | `0x08F0XF03` | — | ~600ms reset sequence |
