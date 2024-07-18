import struct


M_HEADER_0 = 0xBA
M_HEADER_1 = 0xCC

# Example usage
M_ID_POS_VEL = 0x01
M_ID_ARMED = 0x02
M_ID_SETPOINT = 0x03
M_ID_PID = 0x04
M_ID_TRIM = 0x05

# buttons
M_ID_CONTROLS_STATE = 0x40
# - switch_arm_state    uint8_t
# - button_start_cnt    uint8_t
# - button_stop_cnt     uint8_t
# - encoder trim value  uint8_t
# - button_alarm_state  uint8_t

M_ID_SETTINGS_CHEKSUM =     200
# M_ID_SETTINGS_POS_VEL  =    201  # sending countinuosly
# M_ID_SETTINGS_ARMED  =      202  # sending countinuosly
# M_ID_SETTINGS_SETPOINT  =   203  # sending countinuosly
M_ID_SETTINGS_PID  =        204
M_ID_SETTINGS_TRIM  =       205


def crc8(data):
    POLYNOMIAL = 0x07
    crc = 0x00

    for byte in data:
        crc ^= byte

        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ POLYNOMIAL
            else:
                crc <<= 1

            crc &= 0xFF  # Ensure CRC remains an 8-bit value

    return crc

def pack_floats_with_crc8(floats, message_id):
    # Create a byte array with the headers and message ID
    byte_array = bytearray([M_HEADER_0, M_HEADER_1, message_id])
    
    # Pack each float into the byte array
    for f in floats:
        byte_array.extend(struct.pack('<f', f))  # '<f' for little-endian float

    # Calculate the CRC-8 checksum
    checksum = crc8(byte_array)

    # Append the checksum to the byte array
    byte_array.append(checksum)

    return byte_array

def bytes_to_hex_string(byte_array):
    return ' '.join(f'{byte:02X}' for byte in byte_array)


def pack_floats_data_for_serial(message_id, float_data):
    # Create a byte array with the headers and message ID
    byte_array = bytearray([M_HEADER_0, M_HEADER_1, message_id])
    
    # Pack the float data into the byte array
    for f in float_data:
        byte_array.extend(struct.pack('<f', f))  # '<f' for little-endian float

    # Calculate the CRC-8 checksum
    checksum = crc8(byte_array)

    # Append the checksum to the byte array
    byte_array.append(checksum)

    return byte_array


def pack_armed_data_for_serial(message_id, armed):
    # Create a byte array with the headers and message ID
    byte_array = bytearray([M_HEADER_0, M_HEADER_1, message_id])
    
    # If the message is M_ID_ARMED, append the armed status
    byte_array.append(armed)

    # Calculate the CRC-8 checksum
    checksum = crc8(byte_array)

    # Append the checksum to the byte array
    byte_array.append(checksum)

    return byte_array


def make_dict_error(message_lable, error_code):
    if error_code == 0x00:
        return {"type": "SETTINGS_LOG", "data": message_lable + " OK"}
    else:
        return {"type": "SETTINGS_ERROR", "data": message_lable + " ERROR"}


def crc_index_by_id(id):
    bit4ids = [
        M_ID_SETTINGS_CHEKSUM,
        M_ID_SETTINGS_PID,
        M_ID_SETTINGS_TRIM,
    ]

    is_present = id in bit4ids
    if is_present:
        return 4

    if id is M_ID_CONTROLS_STATE:
        return 8

    return 0


def parse_serial_log_data(data):
    if data[0] != M_HEADER_0 or data[1] != M_HEADER_1:
        return {"type": "HEADER_ERROR", "data": "Invalid header"}

    message_id = data[2]

    checksum_index = crc_index_by_id(message_id)

    if len(data) < (checksum_index + 1):  # Minimum length with no payload
        return {"type": "HEADER_ERROR", "data": "Invalid length"}

    checksum = data[checksum_index]

    calculated_checksum = crc8(data[:checksum_index])
    # print("checksum_index", checksum_index)
    # print("data", data)
    # print("calculated_checksum 0x{0:02x} sss ".format(calculated_checksum), calculated_checksum, checksum)
    if checksum != calculated_checksum:
        return {"type": "ERROR", "data": "PC income checksum"}


    error_code = data[3]
    # payload = data[3:-2]
    if message_id == M_ID_SETTINGS_CHEKSUM:
        # floats = struct.unpack('<' + 'f' * (len(payload) // 4), payload)
        return {"type": "ERROR", "data": "Invalid checksum on device"}

    # elif message_id == 201:
    #     message_lable = "POS_VEL"
    #     make_dict_error(message_lable, error_code)
    
    # elif message_id == 202:
    #     message_lable = "ARMED"
    #     make_dict_error(message_lable, error_code)

    # elif message_id == 203:
    #     message_lable = "SETPOINT"
    #     make_dict_error(message_lable, error_code)

    elif message_id == M_ID_SETTINGS_PID:
        message_lable = "PID"
        return make_dict_error(message_lable, error_code)
        

    elif message_id == M_ID_SETTINGS_TRIM:
        message_lable = "TRIM"
        return make_dict_error(message_lable, error_code)
        
    elif message_id == M_ID_CONTROLS_STATE:
        # message_lable = "TRIM"
        
        # Slice the array to start from the fourth byte
        relevant_data = data[3:crc_index_by_id(message_id)]
        # print(relevant_data)
        # Unpack the data
        switch_arm_state, button_start_cnt, button_stop_cnt, encoder_trim_value, button_alarm_state = struct.unpack('BBBBB', relevant_data)
        # print("switch_arm_state, button_start_cnt, button_stop_cnt, encoder_trim_value, button_alarm_state")
        # print(switch_arm_state, button_start_cnt, button_stop_cnt, encoder_trim_value, button_alarm_state)
        
        return {"type": "CONTROLS", "data": [switch_arm_state, button_start_cnt, button_stop_cnt, encoder_trim_value, button_alarm_state] }
        

    # Add more message ID handling as needed

    return {"type": "UNKNOWN", "data": "Unknown message ID"}


if __name__ == '__main__':
    # Example usage
    floats = [1.0, 2.0, 3.0, 4.0]  # Example float data
    message_id = 0x01  # Example message ID
    packed_data = pack_floats_with_crc8(floats, message_id)
    print(f"Packed Data: {packed_data}")

    hex_string = bytes_to_hex_string(packed_data)
    print(f"Packed Data: {hex_string}")


    # Example data for different message IDs
    pos_vel_data = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]
    armed_status = False
    setpoint_data = [8.0, 9.0, 10.0]
    pid_data = [11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0]
    trim_data = [28.0, 29.0, 30.0, 31.0]

    # Packing different types of messages
    packed_pos_vel = pack_floats_data_for_serial(M_ID_POS_VEL, pos_vel_data)
    packed_armed = pack_armed_data_for_serial(M_ID_ARMED, armed_status)
    packed_setpoint = pack_floats_data_for_serial(M_ID_SETPOINT, setpoint_data)
    packed_pid = pack_floats_data_for_serial(M_ID_PID, pid_data)
    packed_trim = pack_floats_data_for_serial(M_ID_TRIM, trim_data)

    # Print packed data
    print("Packed POS_VEL:", packed_pos_vel)
    print("Packed ARMED:", packed_armed)
    print("Packed SETPOINT:", packed_setpoint)
    print("Packed PID:", packed_pid)
    print("Packed TRIM:", packed_trim)