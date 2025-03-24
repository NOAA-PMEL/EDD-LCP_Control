import os
import struct

def open_directory(dirpath):
    for filename in os.listdir(dirpath):
        if filename.endswith(".sbd"):
            parse_sbd_to_txt(os.path.join(dirpath, filename), ".sbd")

def parse_sbd_to_txt(sbdfile, extension):
    if sbdfile.endswith(extension):
        txtfile = sbdfile.replace(extension, ".txt")
        decode_sbd_message(sbdfile, txtfile)

def decode_sbd_message(sbdfile, txtfile):
    with open(sbdfile, 'rb') as sourcefile:
        buf = sourcefile.read()

    if not buf:
        print(f"LCP :: ERROR, Either {sbdfile} file is missing or not able to open")
        return

    if len(buf) < 29:
        print("LCP :: ERROR, buffer size is smaller than expected header size")
        return

    id = buf[0]
    fw = (buf[1] << 8) | (buf[2] & 0xFF)
    fw_date = (buf[3] << 8) | (buf[4] & 0xFF)
    latitude = struct.unpack('>i', buf[5:9])[0] / 1000000.0
    longitude = struct.unpack('>i', buf[9:13])[0] / 1000000.0
    lcp_var = buf[13]
    ser = (buf[14] << 16) | (buf[15] << 8) | buf[16]
    profNr = buf[17]
    mlength = (buf[18]<<8) | (buf[19] & 0xFF)
    mode = (buf[20] >> 4) & 0x0F  # Correct mask to 0x0F
    pageNr = buf[20] & 0x0F
    start = (buf[21] << 24) | (buf[22] << 16) | (buf[23] << 8) | buf[24]
    stop = (buf[25] << 24) | (buf[26] << 16) | (buf[27] << 8) | buf[28]

    # Debugging information
    print(f"Buffer (hex): {buf.hex()}")
    print(f"ID: {id}, FW: {fw}, FW Date: {fw_date}, Latitude: {latitude}, Longitude: {longitude}")
    print(f"LCP Variant: {lcp_var}, Serial: {ser}, Profile Nr: {profNr}, Measurements: {mlength}")
    print(f"Mode: {mode}, Page Nr: {pageNr}, Start: {start}, Stop: {stop}")

    profile_dir = "profile"
    park_dir = "park"

    if mode == 0x00:
        dest_dir = os.path.join(os.path.dirname(sbdfile), park_dir)
    elif mode == 0x01:
        dest_dir = os.path.join(os.path.dirname(sbdfile), profile_dir)
    else:
        print(f"LCP :: WARNING, unrecognized mode: {mode}, continuing with the process")
        dest_dir = os.path.join(os.path.dirname(sbdfile), "unknown_mode")

    os.makedirs(dest_dir, exist_ok=True)
    path = os.path.join(dest_dir, os.path.basename(txtfile))

    with open(path, 'w') as destfile:
        destfile.write("\n\tLCP Profiler Information\n")
        destfile.write("=======================================\n")
        destfile.write(f"ID              :   {id}\n")
        destfile.write(f"Firmware        :   {fw>>12&0xF}.{fw>>8&0xF}.{fw&0xFF}-dev\n")
        destfile.write(f"Firmware Date   :   {fw_date>>5&0xF}.{fw_date&0x1F}.{fw_date>>9&0x7F}\n")
        destfile.write(f"Latitude        :   {latitude:.7f}\n")
        destfile.write(f"Longitude       :   {longitude:.7f}\n")
        destfile.write(f"LCP Variant     :   {lcp_var}\n")
        destfile.write(f"LCP Serial      :   {ser>>16&0xFF}{ser>>8&0xFF}{ser&0xFF}\n")
        destfile.write(f"Profile Nr      :   {profNr}\n")
        destfile.write(f"Measurements    :   {mlength}\n")

        if mode == 0x00:
            destfile.write(f"Mode            :   {mode}, Park Mode\n")
        elif mode == 0x01:
            destfile.write(f"Mode            :   {mode}, Profile Mode\n")
        else:
            destfile.write(f"Mode            :   {mode}, Unknown Mode\n")

        destfile.write(f"Page Nr         :   {pageNr}\n")
        destfile.write(f"Start Time      :   {start}\n")
        destfile.write(f"Stop Time       :   {stop}\n")
        destfile.write("\n")

        destfile.write("Sr.No.\tPressure(bar)\tTemperature(°C)\n")
        destfile.write("=======================================\n")

        lbuf = buf[29:]
        t, p = unpack_measurements(lbuf, mlength)

        for j in range(len(t)):
            destfile.write(f"{j+1}\t{p[j]:.3f}\t\t{t[j]:.3f}\n")
        destfile.write(f"\ntotal measurements={len(t)}\n\n")

def unpack_measurements(buf, mlength):
    t = []
    p = []

    bitpos = 0
    for _ in range(mlength):
        if bitpos // 8 >= len(buf):
            break

        T = 0
        for j in range(12):
            if bitpos // 8 >= len(buf):
                break
            if buf[bitpos // 8] & (1 << (7 - (bitpos % 8))):
                T |= 1 << (11 - j)
            bitpos += 1

        t.append(unpack_data(T) / 100.0)

        P = 0
        for j in range(8):
            if bitpos // 8 >= len(buf):
                break
            if buf[bitpos // 8] & (1 << (7 - (bitpos % 8))):
                P |= 1 << (7 - j)
            bitpos += 1

        p.append(P / 10.0)

    return t, p

def unpack_data(temp):
    return temp - 500

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print(f"LCP :: Usage, {sys.argv[0]} <directory>")
        sys.exit(1)

    open_directory(sys.argv[1])
