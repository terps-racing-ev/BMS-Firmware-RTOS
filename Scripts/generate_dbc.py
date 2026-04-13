#!/usr/bin/env python3
"""
Generate complete DBC file for BMS Firmware with all 6 modules
Each module has:
- 14 temperature messages (56 thermistors total per module)
- Thermistors 54 and 55 (per module) are ambient sensors
- 1 temperature summary message (min/max/avg cell temp, min/max thermistor IDs)
- 6 voltage messages (18 cells total per module, 3 cells per message)
- 2 voltage summary messages (BMS1/BMS2 avg, min, max, min/max cell IDs)
- 1 heartbeat message
- 1 CAN stats message
- 1 config command message
- 1 config ACK message
- 1 reset command message
- 1 BMS chip reset command message
- 1 BMS chip reset ACK message
- 1 debug response message
- 1 I2C diagnostics message
- 1 balance command message (from host to module)
Plus 1 broadcast message (debug request)
"""

def generate_dbc():
    lines = []
    
    # Header
    lines.append('VERSION ""')
    lines.append('')
    lines.append('NS_ : ')
    lines.append('\tNS_DESC_')
    lines.append('\tCM_')
    lines.append('\tBA_DEF_')
    lines.append('\tBA_')
    lines.append('\tVAL_')
    lines.append('\tCAT_DEF_')
    lines.append('\tCAT_')
    lines.append('\tFILTER')
    lines.append('\tBA_DEF_DEF_')
    lines.append('\tEV_DATA_')
    lines.append('\tENVVAR_DATA_')
    lines.append('\tSGTYPE_')
    lines.append('\tSGTYPE_VAL_')
    lines.append('\tBA_DEF_SGTYPE_')
    lines.append('\tBA_SGTYPE_')
    lines.append('\tSIG_TYPE_REF_')
    lines.append('\tVAL_TABLE_')
    lines.append('\tSIG_GROUP_')
    lines.append('\tSIG_VALTYPE_')
    lines.append('\tSIGTYPE_VALTYPE_')
    lines.append('\tBO_TX_BU_')
    lines.append('\tBA_DEF_REL_')
    lines.append('\tBA_REL_')
    lines.append('\tBA_SGTYPE_REL_')
    lines.append('\tSG_MUL_VAL_')
    lines.append('')
    lines.append('BS_:')
    lines.append('')
    
    # Nodes
    lines.append('BU_: BMS_Module_0 BMS_Module_1 BMS_Module_2 BMS_Module_3 BMS_Module_4 BMS_Module_5 CAN_Host Current_Sensor')
    lines.append('')
    
    # Attribute definitions
    lines.append('BA_DEF_ BO_  "VFrameFormat" ENUM  "StandardCAN","ExtendedCAN";')
    lines.append('BA_DEF_ BO_  "GenMsgCycleTime" INT 0 10000;')
    lines.append('BA_DEF_DEF_  "VFrameFormat" "StandardCAN";')
    lines.append('BA_DEF_DEF_  "GenMsgCycleTime" 0;')
    lines.append('')
    
    # Broadcast messages
    # Debug Request: 0x08F00F10 (only this one is truly broadcast)
    can_id = 0x08F00F10
    dbc_id = can_id | 0x80000000
    lines.append(f'BO_ {dbc_id} BMS_Debug_Request: 8 CAN_Host')
    lines.append(' SG_ Request_Type : 0|8@1+ (1,0) [0|255] "" BMS_Module_0,BMS_Module_1,BMS_Module_2,BMS_Module_3,BMS_Module_4,BMS_Module_5')
    lines.append('')

    # Current sensor message (standard CAN ID 0x0CC)
    # Bytes 0-1: LC current, signed int16, little-endian, 0.1 A/bit
    # Bytes 2-3: HC current, signed int16, little-endian, 0.1 A/bit
    # Bytes 4-7: Reserved (currently zero)
    lines.append('BO_ 204 Current_Sensor_Data: 8 Current_Sensor')
    lines.append(' SG_ LC_Current : 0|16@1- (0.1,0) [-3276.8|3276.7] "A" CAN_Host')
    lines.append(' SG_ HC_Current : 16|16@1- (0.1,0) [-3276.8|3276.7] "A" CAN_Host')
    lines.append(' SG_ Reserved_4 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ Reserved_5 : 40|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ Reserved_6 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append(' SG_ Reserved_7 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
    lines.append('')
    
    # Per-module messages
    message_ids = []
    for module in range(6):
        module_offset = module << 12  # module_id << 12
        therm_base = module * 56  # Base thermistor number for this module
        cell_base = module * 18   # Base cell number for this module (18 cells per module)
        
        lines.append(f'// ============== Module {module} (Thermistors {therm_base}-{therm_base+55}) ==============')
        lines.append('')
        
        # Config Command: base 0x08F00F00 + module_offset
        can_id = 0x08F00F00 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Config_Command_{module}: 8 CAN_Host')
        lines.append(f' SG_ Command : 0|8@1+ (1,0) [0|255] "" BMS_Module_{module}')
        lines.append(f' SG_ Value : 8|8@1+ (1,0) [0|255] "" BMS_Module_{module}')
        lines.append('')
        
        # Config ACK: base 0x08F00F01 + module_offset
        # Response format depends on command:
        # - Set commands (0x01-0x05): Byte2=OldValue, Byte3=NewValue
        # - Get command (0x06): Byte2=ParamSelector, Byte3-4=CurrentValue (16-bit)
        can_id = 0x08F00F01 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Config_ACK_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Command_Echo : 0|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Status : 8|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Byte2_OldVal_or_Param : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Byte3_NewVal_or_ValLo : 24|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Byte4_ValHi : 32|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')
        
        # Reset Command: base 0x08F00F02 + module_offset
        can_id = 0x08F00F02 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Reset_Command_{module}: 1 CAN_Host')
        lines.append(f' SG_ Reset_Magic : 0|8@1+ (1,0) [0|255] "" BMS_Module_{module}')
        lines.append('')
        
        # BMS Chip Reset Command: base 0x08F00F03 + module_offset
        can_id = 0x08F00F03 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Chip_Reset_Command_{module}: 1 CAN_Host')
        lines.append(f' SG_ Reset_Request : 0|8@1+ (1,0) [0|255] "" BMS_Module_{module}')
        lines.append('')
        
        # BMS Chip Reset ACK: base 0x08F00F04 + module_offset
        can_id = 0x08F00F04 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Chip_Reset_ACK_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Status : 0|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')
        
        # Debug Response: base 0x08F00F11 + module_offset
        can_id = 0x08F00F11 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Debug_Response_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Module_ID : 0|8@1+ (1,0) [0|15] "" CAN_Host')
        lines.append(' SG_ Free_Heap_KB : 8|8@1+ (0.25,0) [0|63.75] "KB" CAN_Host')
        lines.append(' SG_ Min_Free_Heap_KB : 16|8@1+ (0.25,0) [0|63.75] "KB" CAN_Host')
        lines.append(' SG_ Boot_Valid : 24|8@1+ (1,0) [0|1] "" CAN_Host')
        lines.append(' SG_ Uptime_Seconds : 32|32@1+ (1,0) [0|4294967295] "s" CAN_Host')
        lines.append('')
        
        # I2C Diagnostics: base 0x08F00F12 + module_offset
        can_id = 0x08F00F12 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_I2C_Diagnostics_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ I2C1_Error_Code : 0|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ I2C3_Error_Code : 8|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ I2C1_State : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ I2C3_State : 24|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_1 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_2 : 40|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_3 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_4 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')
        
        # Balance Command: base 0x08F00F05 + module_offset
        # Host sends this message every 5 seconds to keep module in balancing mode
        can_id = 0x08F00F05 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Balance_Command_{module}: 2 CAN_Host')
        lines.append(f' SG_ BMS1_Balance_Enable : 0|8@1+ (1,0) [0|1] "" BMS_Module_{module}')
        lines.append(f' SG_ BMS2_Balance_Enable : 8|8@1+ (1,0) [0|1] "" BMS_Module_{module}')
        lines.append('')
        
        # Balance ACK: base 0x08F00F06 + module_offset
        # Sent by module in response to Balance Command
        can_id = 0x08F00F06 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Balance_ACK_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Balance_Status : 0|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Time_Remaining_ms : 8|16@1+ (1,0) [0|65535] "ms" CAN_Host')
        lines.append('')
        
        # Balance Config: base 0x08F00F07 + module_offset
        # Host sends this message with target voltage and max cells per chip
        # Must be sent every 5 seconds during balancing to continue
        can_id = 0x08F00F07 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Balance_Config_{module}: 8 CAN_Host')
        lines.append(f' SG_ Target_Voltage_mV : 0|16@1+ (1,0) [0|5000] "mV" BMS_Module_{module}')
        lines.append(f' SG_ Max_Cells_Per_Chip : 16|8@1+ (1,0) [1|9] "" BMS_Module_{module}')
        lines.append('')
        
        # Balance Status: base 0x08F00F08 + module_offset
        # Sent by module every 1 second during balancing
        can_id = 0x08F00F08 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Balance_Status_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Target_Voltage_mV : 0|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ Max_Cells_Per_Chip : 16|8@1+ (1,0) [1|9] "" CAN_Host')
        lines.append(' SG_ BMS1_Cell_Count : 24|4@1+ (1,0) [0|9] "" CAN_Host')
        lines.append(' SG_ BMS2_Cell_Count : 28|4@1+ (1,0) [0|9] "" CAN_Host')
        lines.append(' SG_ BMS1_IC_Temp : 32|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ BMS2_IC_Temp : 48|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append('')
        
        # BMS1 Balance Detail: base 0x08F00F09 + module_offset
        # Sent by module every 1 second during balancing - readback from BMS1 chip
        # Byte 0: CBSTATUS1 (cells 1-8 ACTUALLY balancing)
        # Byte 1: CBSTATUS2 (cell 9 in bit 0)
        # Byte 2: CBSTATUS3 (status flags)
        # Bytes 3-4: AlarmRawStatus (16-bit LE)
        can_id = 0x08F00F09 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS1_Balance_Detail_{module}: 8 BMS_Module_{module}')
        # Individual cell balancing bits from CBSTATUS1 (byte 0, bits 0-7 = cells 1-8)
        for cell in range(1, 9):
            bit_pos = cell - 1  # Cell 1 = bit 0, Cell 2 = bit 1, etc.
            lines.append(f' SG_ BMS1_Cell{cell}_Bal : {bit_pos}|1@1+ (1,0) [0|1] "" CAN_Host')
        # Cell 9 from CBSTATUS2 (byte 1, bit 0)
        lines.append(' SG_ BMS1_Cell9_Bal : 8|1@1+ (1,0) [0|1] "" CAN_Host')
        # CBSTATUS3 as single byte with value table (byte 2)
        lines.append(' SG_ BMS1_CB_Status : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        # AlarmRawStatus as 16-bit with value table (bytes 3-4)
        lines.append(' SG_ BMS1_Alarm_Raw : 24|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append('')
        
        # BMS2 Balance Detail: base 0x08F00F0A + module_offset
        # Sent by module every 1 second during balancing - readback from BMS2 chip
        # Byte 0: CBSTATUS1 (cells 10-17 ACTUALLY balancing)
        # Byte 1: CBSTATUS2 (cell 18 in bit 0)
        # Byte 2: CBSTATUS3 (status flags)
        # Bytes 3-4: AlarmRawStatus (16-bit LE)
        can_id = 0x08F00F0A + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS2_Balance_Detail_{module}: 8 BMS_Module_{module}')
        # Individual cell balancing bits from CBSTATUS1 (byte 0, bits 0-7 = cells 10-17)
        for cell in range(10, 18):
            bit_pos = cell - 10  # Cell 10 = bit 0, Cell 11 = bit 1, etc.
            lines.append(f' SG_ BMS2_Cell{cell}_Bal : {bit_pos}|1@1+ (1,0) [0|1] "" CAN_Host')
        # Cell 18 from CBSTATUS2 (byte 1, bit 0)
        lines.append(' SG_ BMS2_Cell18_Bal : 8|1@1+ (1,0) [0|1] "" CAN_Host')
        # CBSTATUS3 as single byte with value table (byte 2)
        lines.append(' SG_ BMS2_CB_Status : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        # AlarmRawStatus as 16-bit with value table (bytes 3-4)
        lines.append(' SG_ BMS2_Alarm_Raw : 24|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append('')
        
        # Heartbeat: base 0x08F00300 + module_offset
        can_id = 0x08F00300 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Heartbeat_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ BMS_State : 0|8@1+ (1,0) [0|7] "" CAN_Host')
        lines.append(' SG_ Error_Flags_Byte0 : 8|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Error_Flags_Byte1 : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Error_Flags_Byte2 : 24|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Error_Flags_Byte3 : 32|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Warning_Summary : 40|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Fault_Count : 48|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append('')
        
        # CAN Stats: base 0x08F00301 + module_offset
        can_id = 0x08F00301 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_CAN_Stats_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ RX_Message_Count : 0|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append(' SG_ TX_Success_Count : 16|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append(' SG_ TX_Error_Count : 32|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ RX_Queue_Full_Count : 40|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Bus_Off_Count : 48|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ TX_Queue_Full_Count : 56|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')
        
        # 14 Temperature messages (0x08F00000 through 0x08F0000D + module_offset)
        for msg_idx in range(14):
            can_id = 0x08F00000 + msg_idx + module_offset
            dbc_id = can_id | 0x80000000
            message_ids.append(dbc_id)
            
            # Calculate thermistor numbers for this message (4 per message)
            therm_start = therm_base + (msg_idx * 4)
            therm_end = therm_start + 3  # 4 thermistors total (start to start+3)
            
            # Message name indicates thermistor range (e.g., Cell_Temp_0_3 for thermistors 0-3)
            # Note: DBC format only allows alphanumeric and underscore in names, no hyphens
            lines.append(f'BO_ {dbc_id} Cell_Temp_{therm_start}_{therm_end}: 8 BMS_Module_{module}')
            for i in range(4):
                therm_num = therm_start + i
                local_therm_num = therm_num - therm_base
                bit_start = i * 16

                # Last two thermistors per module are ambient sensors
                if local_therm_num == 54:
                    signal_name = f'Ambient_Temp_1_{therm_num:03d}'
                elif local_therm_num == 55:
                    signal_name = f'Ambient_Temp_2_{therm_num:03d}'
                else:
                    signal_name = f'Temp_{therm_num:03d}'

                lines.append(f' SG_ {signal_name} : {bit_start}|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
            lines.append('')
        
        # Cell Temperature Summary: base 0x08F00101 + module_offset
        # Bytes 0-1: Min cell temp (0.1 degC), 2-3: Max cell temp (0.1 degC)
        # Bytes 4-5: Avg cell temp (0.1 degC), 6: Min temp thermistor ID, 7: Max temp thermistor ID
        can_id = 0x08F00101 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} Cell_Temp_Summary_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Min_Cell_Temp : 0|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ Max_Cell_Temp : 16|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ Avg_Cell_Temp : 32|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ Min_Cell_Temp_ID : 48|8@1+ (1,0) [1|54] "" CAN_Host')
        lines.append(' SG_ Max_Cell_Temp_ID : 56|8@1+ (1,0) [1|54] "" CAN_Host')
        lines.append('')
        
        # 6 Voltage messages (0x08F00200 through 0x08F00205 + module_offset)
        # Each message contains 3 cell voltages (6 bytes)
        for msg_idx in range(6):
            can_id = 0x08F00200 + msg_idx + module_offset
            dbc_id = can_id | 0x80000000
            message_ids.append(dbc_id)
            
            # Calculate cell numbers for this message (3 per message)
            cell_start = cell_base + (msg_idx * 3) + 1  # +1 because cells are 1-indexed
            cell_end = cell_start + 2  # 3 cells total
            
            # Message name indicates cell range (e.g., Cell_Voltage_1_3 for cells 1-3)
            # Note: DBC format only allows alphanumeric and underscore in names, no hyphens
            lines.append(f'BO_ {dbc_id} Cell_Voltage_{cell_start}_{cell_end}: 6 BMS_Module_{module}')
            for i in range(3):
                cell_num = cell_start + i
                bit_start = i * 16
                lines.append(f' SG_ Cell_{cell_num:03d}_Voltage : {bit_start}|16@1+ (1,0) [0|5000] "mV" CAN_Host')
            lines.append('')
        
        # BMS1 Chip Status: base 0x08F00206 + module_offset
        can_id = 0x08F00206 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS1_Chip_Status_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ BMS1_Stack_Voltage : 0|16@1+ (10,0) [0|655350] "mV" CAN_Host')
        lines.append(' SG_ BMS1_Alarm_Status : 16|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append(' SG_ BMS1_TS2_Temperature : 32|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ Reserved_1 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_2 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')
        
        # BMS2 Chip Status: base 0x08F00207 + module_offset
        can_id = 0x08F00207 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS2_Chip_Status_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ BMS2_Stack_Voltage : 0|16@1+ (10,0) [0|655350] "mV" CAN_Host')
        lines.append(' SG_ BMS2_Alarm_Status : 16|16@1+ (1,0) [0|65535] "" CAN_Host')
        lines.append(' SG_ BMS2_TS2_Temperature : 32|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
        lines.append(' SG_ Reserved_1 : 48|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Reserved_2 : 56|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append('')

        # BMS1 Voltage Summary: base 0x08F00208 + module_offset
        # Bytes 0-1: Average voltage (mV), 2-3: Min voltage (mV), 4-5: Max voltage (mV)
        # Byte 6: Min voltage cell ID (1-9), Byte 7: Max voltage cell ID (1-9)
        can_id = 0x08F00208 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS1_Voltage_Summary_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ BMS1_Voltage_Average : 0|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS1_Voltage_Min : 16|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS1_Voltage_Max : 32|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS1_Min_Voltage_Cell_ID : 48|8@1+ (1,0) [1|9] "" CAN_Host')
        lines.append(' SG_ BMS1_Max_Voltage_Cell_ID : 56|8@1+ (1,0) [1|9] "" CAN_Host')
        lines.append('')

        # BMS2 Voltage Summary: base 0x08F00209 + module_offset
        # Bytes 0-1: Average voltage (mV), 2-3: Min voltage (mV), 4-5: Max voltage (mV)
        # Byte 6: Min voltage cell ID (10-18), Byte 7: Max voltage cell ID (10-18)
        can_id = 0x08F00209 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS2_Voltage_Summary_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ BMS2_Voltage_Average : 0|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS2_Voltage_Min : 16|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS2_Voltage_Max : 32|16@1+ (1,0) [0|5000] "mV" CAN_Host')
        lines.append(' SG_ BMS2_Min_Voltage_Cell_ID : 48|8@1+ (1,0) [10|18] "" CAN_Host')
        lines.append(' SG_ BMS2_Max_Voltage_Cell_ID : 56|8@1+ (1,0) [10|18] "" CAN_Host')
        lines.append('')
    
    # Comments
    lines.append('CM_ BO_ 2297433872 "Debug info request broadcast to all modules";')
    lines.append('CM_ BO_ 204 "Current sensor node data: LC/HC currents, 0.1A signed little-endian";')
    lines.append('')
    
    # VFrameFormat attributes for all messages
    lines.append('// Extended CAN ID markers')
    lines.append('BA_ "VFrameFormat" BO_ 204 0;')
    lines.append('BA_ "VFrameFormat" BO_ 2297433872 1;')
    for msg_id in message_ids:
        lines.append(f'BA_ "VFrameFormat" BO_ {msg_id} 1;')
    lines.append('')
    
    # Message cycle times
    lines.append('// Message cycle times (ms)')
    for msg_id in message_ids:
        msg_base = msg_id & 0xFFFF
        if msg_base >= 0x8300 and msg_base <= 0x8301:  # Heartbeat or CAN Stats
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 1000;')
        elif msg_base >= 0x8000 and msg_base <= 0x800D:  # Temperature messages
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 1000;')
        elif msg_base == 0x8101:  # Temperature summary message
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 5000;')
        elif msg_base >= 0x8200 and msg_base <= 0x8205:  # Voltage messages
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 500;')
        elif msg_base == 0x8208 or msg_base == 0x8209:  # BMS voltage summary messages
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 500;')
        elif (msg_base & 0xFF) == 0x05 and (msg_base & 0x0F00) == 0x0F00:  # Balance Command (0x8F05)
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 5000;')
    lines.append('')
    
    # Value tables
    lines.append('// Enumerations')
    # Config ACK Status for all modules
    for module in range(6):
        can_id = 0x08F00F01 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Status 0 "Success" 1 "Invalid_Value" 2 "Flash_Error" 3 "Reset_Required";')
    
    # BMS Chip Reset ACK Status for all modules
    for module in range(6):
        can_id = 0x08F00F04 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Status 0 "Success" 1 "Failed";')
    
    # Balance Command Enable for all modules
    for module in range(6):
        can_id = 0x08F00F05 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS1_Balance_Enable 0 "Disable" 1 "Enable";')
        lines.append(f'VAL_ {dbc_id} BMS2_Balance_Enable 0 "Disable" 1 "Enable";')
    
    # BMS State for all heartbeat messages
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS_State 0 "INIT" 1 "IDLE" 2 "CHARGING" 3 "DISCHARGING" 4 "BALANCING" 5 "FAULT" 6 "RESERVED";')
    
    lines.append('')
    
    # Error Flags Byte 0 - Temperature Errors (bit flags)
    lines.append('// Error Flags Byte 0 - Temperature Errors (bit flags)')
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Error_Flags_Byte0 1 "OVER_TEMP" 2 "UNDER_TEMP" 4 "TEMP_SENSOR_FAULT" 8 "TEMP_GRADIENT" 16 "RESERVED_4" 32 "RESERVED_5" 64 "RESERVED_6" 128 "RESERVED_7";')
    
    lines.append('')
    
    # Error Flags Byte 1 - Voltage Errors (bit flags)
    lines.append('// Error Flags Byte 1 - Voltage Errors (bit flags)')
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Error_Flags_Byte1 1 "OVER_VOLTAGE" 2 "UNDER_VOLTAGE" 4 "VOLTAGE_IMBALANCE" 8 "VOLTAGE_SENSOR_FAULT" 16 "RESERVED_4" 32 "RESERVED_5" 64 "RESERVED_6" 128 "RESERVED_7";')
    
    lines.append('')
    
    # Error Flags Byte 2 - Current Errors (bit flags)
    lines.append('// Error Flags Byte 2 - Current Errors (bit flags)')
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Error_Flags_Byte2 1 "OVER_CURRENT_CHARGE" 2 "OVER_CURRENT_DISCHARGE" 4 "CURRENT_SENSOR_FAULT" 8 "SHORT_CIRCUIT" 16 "RESERVED_4" 32 "RESERVED_5" 64 "RESERVED_6" 128 "RESERVED_7";')
    
    lines.append('')
    
    # Error Flags Byte 3 - Communication Errors (bit flags)
    lines.append('// Error Flags Byte 3 - Communication Errors (bit flags)')
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Error_Flags_Byte3 1 "CAN_BUS_OFF" 2 "CAN_TX_TIMEOUT" 4 "CAN_RX_OVERFLOW" 8 "I2C1_BMS1" 16 "I2C3_BMS2" 32 "I2C_TIMEOUT" 64 "I2C_FAULT" 128 "WATCHDOG";')
    
    lines.append('')
    
    # I2C Error Codes
    lines.append('// I2C Error Codes (HAL_I2C_ERROR_*)')
    for module in range(6):
        can_id = 0x08F00F12 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} I2C1_Error_Code 0 "NONE" 1 "BERR" 2 "ARLO" 4 "AF" 8 "OVR" 16 "DMA" 32 "TIMEOUT";')
        lines.append(f'VAL_ {dbc_id} I2C3_Error_Code 0 "NONE" 1 "BERR" 2 "ARLO" 4 "AF" 8 "OVR" 16 "DMA" 32 "TIMEOUT";')
    
    lines.append('')
    
    # I2C States
    lines.append('// I2C States (HAL_I2C_STATE_*)')
    for module in range(6):
        can_id = 0x08F00F12 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} I2C1_State 0 "RESET" 32 "READY" 36 "BUSY" 33 "BUSY_TX" 34 "BUSY_RX" 40 "LISTEN" 41 "BUSY_TX_LISTEN" 42 "BUSY_RX_LISTEN" 96 "ABORT";')
        lines.append(f'VAL_ {dbc_id} I2C3_State 0 "RESET" 32 "READY" 36 "BUSY" 33 "BUSY_TX" 34 "BUSY_RX" 40 "LISTEN" 41 "BUSY_TX_LISTEN" 42 "BUSY_RX_LISTEN" 96 "ABORT";')
    
    lines.append('')
    
    # Config Command values
    lines.append('// Config Command values')
    for module in range(6):
        can_id = 0x08F00F00 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Command 1 "SET_MODULE_ID" 2 "SET_MAX_TEMP" 3 "SET_MIN_TEMP" 4 "SET_MIN_VOLTAGE" 5 "SET_MAX_VOLTAGE" 6 "GET_VALUE";')
    
    lines.append('')
    
    # Config ACK Command Echo values (same as commands)
    lines.append('// Config ACK Command Echo values')
    for module in range(6):
        can_id = 0x08F00F01 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Command_Echo 1 "SET_MODULE_ID" 2 "SET_MAX_TEMP" 3 "SET_MIN_TEMP" 4 "SET_MIN_VOLTAGE" 5 "SET_MAX_VOLTAGE" 6 "GET_VALUE";')
        lines.append(f'VAL_ {dbc_id} Status 0 "SUCCESS" 1 "FAIL" 2 "SUCCESS_RESET_REQUIRED";')
    
    lines.append('')
    
    # Config GET_VALUE parameter selectors (for Byte2 when Command_Echo is GET_VALUE)
    lines.append('// Config GET_VALUE parameter selectors (Byte2 when Command_Echo=0x06)')
    for module in range(6):
        can_id = 0x08F00F01 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} Byte2_OldVal_or_Param 1 "MODULE_ID" 2 "MAX_TEMP" 3 "MIN_TEMP" 4 "MIN_VOLTAGE" 5 "MAX_VOLTAGE";')
    
    lines.append('')
    
    # Cell balancing status values (0=off, 1=balancing)
    lines.append('// Cell balancing status values - ACTUAL cells balancing (read from CBSTATUS1/2)')
    for module in range(6):
        # BMS1 Balance Detail
        can_id = 0x08F00F09 + (module << 12)
        dbc_id = can_id | 0x80000000
        for cell in range(1, 10):  # Cells 1-8 from CBSTATUS1, Cell 9 from CBSTATUS2
            lines.append(f'VAL_ {dbc_id} BMS1_Cell{cell}_Bal 0 "Off" 1 "Balancing";')
        # BMS2 Balance Detail
        can_id = 0x08F00F0A + (module << 12)
        dbc_id = can_id | 0x80000000
        for cell in range(10, 19):  # Cells 10-17 from CBSTATUS1, Cell 18 from CBSTATUS2
            lines.append(f'VAL_ {dbc_id} BMS2_Cell{cell}_Bal 0 "Off" 1 "Balancing";')
    
    lines.append('')
    
    # CBSTATUS3 values (Cell Balance Status flags)
    lines.append('// CBSTATUS3 Cell Balance Status flags')
    for module in range(6):
        # BMS1
        can_id = 0x08F00F09 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS1_CB_Status 0 "OK" 1 "OT_PreWarn" 2 "OverTemp" 4 "UnderTemp" 8 "Paused" 3 "OT_PreWarn+OverTemp" 9 "OT_PreWarn+Paused" 10 "OverTemp+Paused" 12 "UnderTemp+Paused";')
        # BMS2
        can_id = 0x08F00F0A + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS2_CB_Status 0 "OK" 1 "OT_PreWarn" 2 "OverTemp" 4 "UnderTemp" 8 "Paused" 3 "OT_PreWarn+OverTemp" 9 "OT_PreWarn+Paused" 10 "OverTemp+Paused" 12 "UnderTemp+Paused";')
    
    lines.append('')
    
    # AlarmRawStatus common values
    lines.append('// AlarmRawStatus common values (bit flags: SSA=1, SSB=2, CB=4, PFA=8, etc.)')
    for module in range(6):
        # BMS1
        can_id = 0x08F00F09 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS1_Alarm_Raw 0 "None" 4 "CB_Active" 1 "SafetyA" 2 "SafetyB" 3 "SafetyA+B" 5 "CB+SafetyA" 6 "CB+SafetyB" 7 "CB+SafetyA+B" 8 "PermFailA" 16 "PermFailB" 32 "PermFailC" 64 "PermFailD" 256 "Charging" 512 "Discharging" 260 "CB+Charging" 516 "CB+Discharging";')
        # BMS2
        can_id = 0x08F00F0A + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS2_Alarm_Raw 0 "None" 4 "CB_Active" 1 "SafetyA" 2 "SafetyB" 3 "SafetyA+B" 5 "CB+SafetyA" 6 "CB+SafetyB" 7 "CB+SafetyA+B" 8 "PermFailA" 16 "PermFailB" 32 "PermFailC" 64 "PermFailD" 256 "Charging" 512 "Discharging" 260 "CB+Charging" 516 "CB+Discharging";')
    
    return '\n'.join(lines)

if __name__ == '__main__':
    import os
    
    dbc_content = generate_dbc()
    
    # Output to parent directory (script is in Scripts folder)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(script_dir)
    output_file = os.path.join(parent_dir, 'BMS-Firmware-RTOS-Complete.dbc')
    
    with open(output_file, 'w') as f:
        f.write(dbc_content)
    
    print(f"Generated {output_file}")
    print(f"Total lines: {len(dbc_content.splitlines())}")
    
    # Count only actual message definition lines (exclude BA_DEF_ BO_, CM_ BO_, etc.)
    message_count = sum(1 for line in dbc_content.splitlines() if line.startswith('BO_ '))
    fixed_messages = 2  # Current sensor + debug request
    module_messages = message_count - fixed_messages
    per_module = module_messages // 6 if module_messages >= 0 else 0

    print(f"Total messages: {message_count}")
    print(f"  - 1 current sensor message (ID 0x0CC, standard CAN)")
    print(f"  - 1 broadcast message (Debug Request)")
    print(f"  - 6 modules × {per_module} messages = {module_messages} messages")
    print(f"")
