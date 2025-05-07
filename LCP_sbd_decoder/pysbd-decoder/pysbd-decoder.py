import os
import struct

# Firmware constants (mirroring data.h)
IRID_HEADER_LENGTH = 29

def unpack_header(buf):
    """
    Unpacks the 29-byte header from the SBD message buffer.
    Returns a dictionary containing header fields or None if buffer is too short.
    """
    if len(buf) < IRID_HEADER_LENGTH:
        print("LCP :: ERROR, buffer size is smaller than expected header size for unpacking.")
        return None

    header_data = {}
    header_data['system_id'] = buf[0]
    header_data['firmware_version_raw'] = (buf[1] << 8) | buf[2]
    # Interpreting firmware version (example format, adjust if needed)
    header_data['firmware_major'] = (header_data['firmware_version_raw'] >> 12) & 0xF
    header_data['firmware_minor'] = (header_data['firmware_version_raw'] >> 8) & 0xF
    header_data['firmware_patch'] = header_data['firmware_version_raw'] & 0xFF

    header_data['firmware_build_date_raw'] = (buf[3] << 8) | buf[4]
    # Interpreting build date (example format)
    header_data['build_year'] = (header_data['firmware_build_date_raw'] >> 9) & 0x7F # 7 bits for year
    header_data['build_month'] = (header_data['firmware_build_date_raw'] >> 5) & 0xF   # 4 bits for month
    header_data['build_day'] = header_data['firmware_build_date_raw'] & 0x1F     # 5 bits for day

    # Latitude and Longitude are signed 32-bit integers, scaled by 1,000,000
    header_data['latitude'] = struct.unpack('>i', buf[5:9])[0] / 1000000.0
    header_data['longitude'] = struct.unpack('>i', buf[9:13])[0] / 1000000.0

    header_data['lcp_variant'] = buf[13]
    # Serial number is a 24-bit (3-byte) number
    header_data['serial_number'] = (buf[14] << 16) | (buf[15] << 8) | buf[16]
    header_data['profile_number'] = buf[17]
    # mlength is the number of measurements IN THIS SPECIFIC PAGE/PACKET
    header_data['measurements_in_page'] = (buf[18] << 8) | buf[19]

    header_data['mode'] = (buf[20] >> 4) & 0x0F
    header_data['page_number'] = buf[20] & 0x0F

    # Start and Stop times are unsigned 32-bit integers (epoch time)
    header_data['start_time_epoch'] = struct.unpack('>I', buf[21:25])[0]
    header_data['stop_time_epoch'] = struct.unpack('>I', buf[25:29])[0]
    
    return header_data

def unpack_measurements(buf, mlength):
    t = []
    p = []
    bitpos = 0

    TEMPERATURE_BITS_FIRMWARE = 16
    PRESSURE_BITS_FIRMWARE = 12

    for _ in range(mlength):
        # --- Unpack Temperature (16 bits) ---
        if (bitpos + TEMPERATURE_BITS_FIRMWARE -1) // 8 >= len(buf):
            print(f"LCP :: WARNING, not enough buffer for temperature at measurement index {_}")
            break
        
        T_raw = 0
        for j in range(TEMPERATURE_BITS_FIRMWARE):
            byte_index = bitpos // 8
            bit_in_byte = bitpos % 8
            if (buf[byte_index] & (1 << (7 - bit_in_byte))):
                T_raw |= (1 << ((TEMPERATURE_BITS_FIRMWARE - 1) - j))
            bitpos += 1
        
        temperature_val = (T_raw - 500.0) / 100.0
        t.append(temperature_val)

        # --- Unpack Pressure (12 bits) ---
        if (bitpos + PRESSURE_BITS_FIRMWARE - 1) // 8 >= len(buf):
            print(f"LCP :: WARNING, not enough buffer for pressure at measurement index {_}")
            break

        P_raw = 0
        for j in range(PRESSURE_BITS_FIRMWARE):
            byte_index = bitpos // 8
            bit_in_byte = bitpos % 8
            if (buf[byte_index] & (1 << (7 - bit_in_byte))):
                P_raw |= (1 << ((PRESSURE_BITS_FIRMWARE - 1) - j))
            bitpos += 1
        
        pressure_val = P_raw / 100.0
        p.append(pressure_val)

    return t, p

def decode_sbd_message(sbdfile, txtfile):
    with open(sbdfile, 'rb') as sourcefile:
        buf = sourcefile.read()

    if not buf:
        print(f"LCP :: ERROR, Either {sbdfile} file is missing or not able to open")
        return

    # Decode the header
    header = unpack_header(buf)
    if header is None:
        return # Error message already printed by unpack_header

    # Debugging information
    print(f"SBD File: {sbdfile}")
    print(f"Buffer (hex, first 60 bytes): {buf[:60].hex()}") # Print only first 60 for brevity
    print(f"Decoded Header: {header}")

    profile_dir = "profile"
    park_dir = "park"
    
    # Determine destination directory based on mode
    if header['mode'] == 0x00: # LCP_PARK_MODE
        dest_dir = os.path.join(os.path.dirname(sbdfile), park_dir)
        mode_str = f"{header['mode']}, Park Mode"
    elif header['mode'] == 0x01: # LCP_PROFILE_MODE
        dest_dir = os.path.join(os.path.dirname(sbdfile), profile_dir)
        mode_str = f"{header['mode']}, Profile Mode"
    else:
        print(f"LCP :: WARNING, unrecognized mode: {header['mode']}")
        dest_dir = os.path.join(os.path.dirname(sbdfile), "unknown_mode")
        mode_str = f"{header['mode']}, Unknown Mode"

    os.makedirs(dest_dir, exist_ok=True)
    path = os.path.join(dest_dir, os.path.basename(txtfile))

    with open(path, 'w') as destfile:
        destfile.write("\n\tLCP Profiler Information\n")
        destfile.write("=======================================\n")
        destfile.write(f"System ID         :   {header['system_id']}\n")
        destfile.write(f"Firmware Version  :   {header['firmware_major']}.{header['firmware_minor']}.{header['firmware_patch']}-dev\n")
        destfile.write(f"Firmware BuildDate:   {header['build_month']}.{header['build_day']}.{header['build_year'] + 2000}\n") # Assuming year is offset from 2000
        destfile.write(f"Latitude          :   {header['latitude']:.7f}\n")
        destfile.write(f"Longitude         :   {header['longitude']:.7f}\n")
        destfile.write(f"LCP Variant       :   {header['lcp_variant']}\n")
        # Serial number formatting based on C code: L<byte1><byte2>
        serial_byte1 = (header['serial_number'] >> 8) & 0xFF
        serial_byte2 = header['serial_number'] & 0xFF
        serial_char = chr((header['serial_number'] >> 16) & 0xFF)
        destfile.write(f"LCP Serial        :   {serial_char}{serial_byte1:02X}{serial_byte2:02X}\n") # Example: L4C00
        destfile.write(f"Profile Number    :   {header['profile_number']}\n")
        destfile.write(f"Measurements in page: {header['measurements_in_page']}\n") # This is 'mlength'
        destfile.write(f"Mode              :   {mode_str}\n")
        destfile.write(f"Page Number       :   {header['page_number']}\n")
        destfile.write(f"Start Time (Epoch):   {header['start_time_epoch']}\n")
        destfile.write(f"Stop Time (Epoch) :   {header['stop_time_epoch']}\n")
        destfile.write("\n")

        destfile.write("Sr.No.\tPressure(bar)\tTemperature(°C)\n")
        destfile.write("=======================================\n")

        # Measurements start after the header
        measurements_buf = buf[IRID_HEADER_LENGTH:]
        
        # Unpack measurements using the corrected function
        t_values, p_values = unpack_measurements(measurements_buf, header['measurements_in_page'])

        for j in range(len(t_values)): # Iterate up to the number of actually unpacked measurements
            destfile.write(f"{j+1}\t{p_values[j]:.3f}\t\t{t_values[j]:.3f}\n")
        
        destfile.write(f"\nTotal measurements decoded = {len(t_values)}\n")
        if len(t_values) != header['measurements_in_page']:
             destfile.write(f"WARNING: Expected {header['measurements_in_page']} measurements, decoded {len(t_values)}\n")
        destfile.write("\n")

def open_directory(dirpath):
    for filename in os.listdir(dirpath):
        if filename.endswith(".sbd"):
            parse_sbd_to_txt(os.path.join(dirpath, filename), ".sbd")

def parse_sbd_to_txt(sbdfile, extension):
    if sbdfile.endswith(extension):
        txtfile = sbdfile.replace(extension, ".txt")
        decode_sbd_message(sbdfile, txtfile)

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print(f"LCP :: Usage, {sys.argv[0]} <directory>")
        sys.exit(1)

    open_directory(sys.argv[1])