# Module Configuration System

## Overview

The BMS firmware uses a flash-based module configuration system where each module's ID is stored in flash memory and read at startup. CAN message IDs are calculated with the module offset built-in at initialization, eliminating runtime overhead.

## How It Works

### 1. Flash Storage Location
- **Address**: `0x08041FFC` (last word before the 16KB storage region)
- **Format**: 32-bit word
  - Upper 16 bits: Magic value `0xBEEF` (indicates initialized flash)
  - Lower 4 bits: Module ID (0-15)
  - Example: `0xBEEF0005` = Module ID 5

### 2. Startup Sequence
1. `Config_Init()` is called during system initialization
2. Module ID is read from flash (or defaults to 5 if uninitialized)
3. All CAN IDs are calculated with module offset and stored in global variables:
   - `CAN_TEMP_ID`
   - `CAN_TEMP_RAW_ID`
   - `CAN_BMS_HEARTBEAT_ID`
   - `CAN_BMS_STATS_ID`
   - `CAN_CONFIG_ACK_ID`

### 3. CAN ID Calculation
CAN IDs use a bitwise offset pattern where the module ID is inserted at bits 12-15:

```
Base ID:     0x08F00000
Module ID:   5 (0x5)
Shift:       << 12 bits
Result:      0x08F05000
```

The macro `CAN_ID(base, module)` performs this calculation:
```c
#define CAN_ID(base, module)    ((base) | ((module) << 12))
```

### 4. Setting Module ID

#### Via CAN Command
Send a CAN message to `0x08F00F00` (CAN_CONFIG_CMD_ID):
```
Byte 0: 0x01 (CONFIG_CMD_SET_MODULE_ID)
Byte 1: Module ID (0-15)
Bytes 2-7: Reserved (0x00)
```

**Response** (from current module ID):
```
ID: 0x08F0XF01 (where X = current module ID)
Byte 0: 0x01 (command echo)
Byte 1: Status
  - 0x02 = Success, reset required
  - 0x01 = Failed (invalid module ID)
Byte 2: Old module ID
Byte 3: New module ID (will be active after reset)
Bytes 4-7: Reserved
```

**Important**: After a successful command, the device will automatically reset after 100ms to apply the new module ID.

#### Programmatically
```c
// Set module ID to 7
if (Config_SetModuleID(7) == 0) {
    // Success - written to flash
    // Device must be reset for changes to take effect
    NVIC_SystemReset();
}
```

## File Structure

### Header Files
- **`can_ids.h`**: Defines base CAN IDs and the `CAN_ID()` macro
- **`config_manager.h`**: Configuration management functions and constants

### Source Files
- **`config_manager.c`**: 
  - Flash read/write functions
  - Module ID management
  - CAN ID initialization
  - CAN command processing

### Key Functions

```c
// Initialize config system (reads flash, sets up CAN IDs)
void Config_Init(void);

// Get current module ID
uint8_t Config_GetModuleID(void);

// Set module ID (writes to flash, device must reset)
int8_t Config_SetModuleID(uint8_t module_id);

// Read module ID from flash
uint8_t Config_ReadModuleIDFromFlash(void);

// Write module ID to flash
int8_t Config_WriteModuleIDToFlash(uint8_t module_id);

// Initialize CAN IDs with module offset
void Config_InitCANIDs(void);

// Process incoming CAN configuration commands
void Config_ProcessCANCommand(uint8_t *data, uint8_t length);
```

## Usage Example

### Using CAN IDs in Your Code

All CAN IDs are pre-calculated global variables. Just use them directly:

```c
// Send temperature data
uint8_t temp_data[8] = {0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0};
CAN_SendMessage(CAN_TEMP_ID, temp_data, 8, CAN_PRIORITY_NORMAL);

// Send heartbeat
uint8_t heartbeat[8] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
CAN_SendMessage(CAN_BMS_HEARTBEAT_ID, heartbeat, 8, CAN_PRIORITY_CRITICAL);
```

### Adding New CAN IDs

1. **Define base ID in `can_ids.h`**:
```c
#define CAN_VOLTAGE_BASE        0x08F00200  /**< Base for voltage messages */
```

2. **Declare global variable in `can_ids.h`**:
```c
extern uint32_t CAN_VOLTAGE_ID;
```

3. **Define variable in `config_manager.c`** (private variables section):
```c
uint32_t CAN_VOLTAGE_ID = 0;
```

4. **Initialize in `Config_InitCANIDs()`**:
```c
void Config_InitCANIDs(void)
{
    uint8_t mod_id = Config_GetModuleID();
    
    // ... existing initializations ...
    CAN_VOLTAGE_ID = CAN_ID(CAN_VOLTAGE_BASE, mod_id);
}
```

5. **Use in your code**:
```c
CAN_SendMessage(CAN_VOLTAGE_ID, data, length, priority);
```

## Benefits of This Approach

1. **No Runtime Overhead**: CAN IDs are calculated once at startup, not on every message
2. **Persistent Configuration**: Module ID survives power cycles
3. **Simple to Use**: Just use the global CAN ID variables directly
4. **Clear Separation**: Base IDs are defined in one place, module offset applied systematically
5. **Thread-Safe**: Module ID protected by mutex
6. **Automatic Reset**: Device automatically resets after module ID change

## Flash Memory Layout

```
0x08000000 - 0x08007FFF:  Bootloader (32KB)
0x08008000 - 0x08041FFF:  Application (208KB)
0x08041FFC:               Module ID Storage (4 bytes)
0x08042000 - 0x08045FFF:  Storage Region (16KB, reserved)
```

## Debugging

### Check Current Module ID
```c
uint8_t current_id = Config_GetModuleID();
```

### Check Flash Value
```c
uint32_t flash_value = *(__IO uint32_t*)CONFIG_FLASH_MODULE_ADDR;
// Should be 0xBEEFXXXX where XXXX contains module ID in lower nibble
```

### Verify CAN IDs
```c
// After Config_Init(), these should be populated:
// CAN_TEMP_ID = 0x08F0X000 where X = module ID
// CAN_BMS_HEARTBEAT_ID = 0x08F0X300 where X = module ID
```

## Important Notes

1. **Module ID Range**: Valid range is 0-15 (0xF)
2. **Reset Required**: Changes only take effect after device reset
3. **Flash Wear**: Minimize module ID changes to preserve flash lifetime
4. **Bootloader Compatibility**: Module ID address is outside application space but before storage region
5. **Default Value**: If flash is uninitialized (no magic value), defaults to module ID 5

## Migration from Old System

The old system used `Config_ApplyModuleOffset()` at runtime for every CAN message. This has been removed. All code now uses the pre-calculated global CAN ID variables.

**Old code** (deprecated):
```c
uint32_t can_id = Config_ApplyModuleOffset(CAN_TEMP_BASE_ID);
CAN_SendMessage(can_id, data, length, priority);
```

**New code**:
```c
CAN_SendMessage(CAN_TEMP_ID, data, length, priority);
```
