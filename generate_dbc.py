#!/usr/bin/env python3
"""
Generate complete DBC file for BMS Firmware with all 6 modules
Each module has:
- 14 temperature messages (56 thermistors total per module)
- 1 heartbeat message
- 1 CAN stats message
- 1 config ACK message
- 1 debug response message
Plus 2 broadcast messages (config command, debug request)
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
    lines.append('BU_: BMS_Module_0 BMS_Module_1 BMS_Module_2 BMS_Module_3 BMS_Module_4 BMS_Module_5 CAN_Host')
    lines.append('')
    
    # Attribute definitions
    lines.append('BA_DEF_ BO_  "VFrameFormat" ENUM  "StandardCAN","ExtendedCAN";')
    lines.append('BA_DEF_ BO_  "GenMsgCycleTime" INT 0 10000;')
    lines.append('BA_DEF_DEF_  "VFrameFormat" "StandardCAN";')
    lines.append('BA_DEF_DEF_  "GenMsgCycleTime" 0;')
    lines.append('')
    
    # Broadcast messages
    # Config Command: 0x08F00F00
    can_id = 0x08F00F00
    dbc_id = can_id | 0x80000000
    lines.append(f'BO_ {dbc_id} BMS_Config_Command: 8 CAN_Host')
    lines.append(' SG_ Command : 0|8@1+ (1,0) [0|255] "" BMS_Module_0,BMS_Module_1,BMS_Module_2,BMS_Module_3,BMS_Module_4,BMS_Module_5')
    lines.append(' SG_ Value : 8|8@1+ (1,0) [0|255] "" BMS_Module_0,BMS_Module_1,BMS_Module_2,BMS_Module_3,BMS_Module_4,BMS_Module_5')
    lines.append('')
    
    # Debug Request: 0x08F00F10
    can_id = 0x08F00F10
    dbc_id = can_id | 0x80000000
    lines.append(f'BO_ {dbc_id} BMS_Debug_Request: 8 CAN_Host')
    lines.append(' SG_ Request_Type : 0|8@1+ (1,0) [0|255] "" BMS_Module_0,BMS_Module_1,BMS_Module_2,BMS_Module_3,BMS_Module_4,BMS_Module_5')
    lines.append('')
    
    # Per-module messages
    message_ids = []
    for module in range(6):
        module_offset = module << 12  # module_id << 12
        therm_base = module * 56  # Base thermistor number for this module
        
        lines.append(f'// ============== Module {module} (Thermistors {therm_base}-{therm_base+55}) ==============')
        lines.append('')
        
        # Config ACK: base 0x08F00F01 + module_offset
        can_id = 0x08F00F01 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Config_ACK_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Command_Echo : 0|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Status : 8|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Old_Module_ID : 16|8@1+ (1,0) [0|15] "" CAN_Host')
        lines.append(' SG_ New_Module_ID : 24|8@1+ (1,0) [0|15] "" CAN_Host')
        lines.append('')
        
        # Debug Response: base 0x08F00F11 + module_offset
        can_id = 0x08F00F11 + module_offset
        dbc_id = can_id | 0x80000000
        message_ids.append(dbc_id)
        lines.append(f'BO_ {dbc_id} BMS_Debug_Response_{module}: 8 BMS_Module_{module}')
        lines.append(' SG_ Module_ID : 0|8@1+ (1,0) [0|15] "" CAN_Host')
        lines.append(' SG_ FW_Version_Major : 8|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ FW_Version_Minor : 16|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ FW_Version_Patch : 24|8@1+ (1,0) [0|255] "" CAN_Host')
        lines.append(' SG_ Uptime_Seconds : 32|32@1+ (1,0) [0|4294967295] "s" CAN_Host')
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
            
            # Global message number (0-85 across all modules)
            global_msg_num = (module * 14) + msg_idx
            
            lines.append(f'BO_ {dbc_id} Cell_Temp_{global_msg_num:02d}: 8 BMS_Module_{module}')
            for i in range(4):
                therm_num = therm_start + i
                bit_start = i * 16
                lines.append(f' SG_ Temp_{therm_num:03d} : {bit_start}|16@1- (0.1,0) [-40|125] "degC" CAN_Host')
            lines.append('')
    
    # Comments
    lines.append('CM_ BO_ 2297433856 "Config command broadcast to all modules - sets module ID";')
    lines.append('CM_ SG_ 2297433856 Command "0x01=Set Module ID";')
    lines.append('CM_ BO_ 2297433872 "Debug info request broadcast to all modules";')
    lines.append('')
    
    # VFrameFormat attributes for all messages
    lines.append('// Extended CAN ID markers')
    lines.append('BA_ "VFrameFormat" BO_ 2297433856 1;')
    lines.append('BA_ "VFrameFormat" BO_ 2297433872 1;')
    for msg_id in message_ids:
        lines.append(f'BA_ "VFrameFormat" BO_ {msg_id} 1;')
    lines.append('')
    
    # Message cycle times
    lines.append('// Message cycle times (ms)')
    for msg_id in message_ids:
        if msg_id & 0xFF00 == 0x0300:  # Heartbeat or CAN Stats
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 1000;')
        elif msg_id & 0xFF00 == 0x0000:  # Temperature messages
            lines.append(f'BA_ "GenMsgCycleTime" BO_ {msg_id} 1000;')
    lines.append('')
    
    # Value tables
    lines.append('// Enumerations')
    lines.append('VAL_ 2297433857 Status 0 "Success" 1 "Invalid_Value" 2 "Flash_Error" 3 "Reset_Required";')
    
    # BMS State for all heartbeat messages
    for module in range(6):
        can_id = 0x08F00300 + (module << 12)
        dbc_id = can_id | 0x80000000
        lines.append(f'VAL_ {dbc_id} BMS_State 0 "INIT" 1 "IDLE" 2 "CHARGING" 3 "DISCHARGING" 4 "BALANCING" 5 "ERROR" 6 "SHUTDOWN" 7 "RESERVED";')
    
    return '\n'.join(lines)

if __name__ == '__main__':
    dbc_content = generate_dbc()
    
    output_file = 'BMS-Firmware-RTOS-Modules-0-5.dbc'
    with open(output_file, 'w') as f:
        f.write(dbc_content)
    
    print(f"Generated {output_file}")
    print(f"Total lines: {len(dbc_content.splitlines())}")
    
    # Count messages
    message_count = dbc_content.count('BO_ ')
    print(f"Total messages: {message_count}")
    print(f"  - 2 broadcast messages")
    print(f"  - 6 modules × 18 messages = 108 messages")
    print(f"  - Total: 110 messages")
