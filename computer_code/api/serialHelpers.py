import struct


M_HEADER_0 = 0xBA
M_HEADER_1 = 0xCC

# Example usage
M_ID_POS_VEL = 0x01
M_ID_ARMED = 0x02
M_ID_SETPOINT = 0x03
M_ID_PID = 0x04
M_ID_TRIM = 0x05


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



# import struct

# M_HEADER_0 = 0xBA
# M_HEADER_1 = 0xCC

# def crc8(data):
#     POLYNOMIAL = 0x07
#     crc = 0x00

#     for byte in data:
#         crc ^= byte

#         for _ in range(8):
#             if crc & 0x80:
#                 crc = (crc << 1) ^ POLYNOMIAL
#             else:
#                 crc <<= 1

#             crc &= 0xFF  # Ensure CRC remains an 8-bit value

#     return crc


# def pack_floats():

# def pack_data_for_serial(message_id, float_data, armed=None):
#     # Create a byte array with the headers and message ID
#     byte_array = bytearray([M_HEADER_0, M_HEADER_1, message_id])
    
#     # Pack the float data into the byte array
#     for f in float_data:
#         byte_array.extend(struct.pack('<f', f))  # '<f' for little-endian float

#     # If the message is M_ID_ARMED, append the armed status
#     if message_id == M_ID_ARMED and armed is not None:
#         byte_array.append(armed)

#     # Calculate the CRC-8 checksum
#     checksum = crc8(byte_array)

#     # Append the checksum to the byte array
#     byte_array.append(checksum)

#     return byte_array


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
