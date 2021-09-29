from ctypes import create_unicode_buffer
from struct import pack
from typing import final
import i2cdriver
from pprint import pprint
import time

UBX_HEADER_1 = 0xB5
UBX_HEADER_2 = 0x62

UBX_CFG_PRT_CLASS = 0x06
UBX_CFG_PRT_ID = 0x00
UBX_CFG_MSG = 0x01
UBX_CFG_NAV5 = 0x24
UBX_CFG_NAVX5 = 0x23
UBX_CFG_RATE = 0x08


UBX_CFG_PRT_PAYLOAD = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x42,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x01,0x00,0x00,0x00,0x00]

UBX_NAV_CLASS = 0x01
UBX_NAV_STATUS = 0x03
UBX_NAV_PVT  = 0x07
UBX_NAV_SAT = 0x35

UBX_HNR_CLASS = 0x28
UBX_HNR_PVT = 0x00

I2C_ADDR = 0x42


def chksum(vals):
    CK_A = 0
    CK_B = 0
    for v in vals:
        CK_A += v
        CK_A = CK_A & 0x00FF
        CK_B += CK_A
        CK_B = CK_B & 0x00FF

    # print(CK_A, CK_B)
    return (CK_A, CK_B)

def create_ubx_packet( ubx_class, ubx_id, payload):
    msg1 = [UBX_HEADER_1, UBX_HEADER_2]
    msg2 =[ubx_class]
    msg2 += [ubx_id]

    plen = int.to_bytes(len(payload), 2, 'little')
    msg2 += [plen[0]]
    msg2 += [plen[1]]
    msg2 += payload
    (ckA, ckB) = chksum(msg2)
    msg2 += [ckA]
    msg2 += [ckB]

    msg = msg1 + msg2
    return msg

def parse_ubx_packet(msg):
    if (msg[0] != UBX_HEADER_1) or (msg[1] != UBX_HEADER_2):
        return False
    
    length =  (msg[5]<<8) + msg[4]
    ubx = {
        'class': msg[2],
        'id': msg[3],
        'len': length,
        'payload': msg[6: 6 + length],
        'chA': msg[6+length],
        'chB': msg[7+length]
    }
    return ubx

def parse_ubx_nav_pvt_packet(packet):
    if(packet):
        payload = packet['payload']
        cfg = {
            'iTOW': int.from_bytes(payload[0:4], 'little'),
            'year': int.from_bytes(payload[4:6], 'little'),
            'month': int.from_bytes(payload[6:7], 'little'),
            'day': int.from_bytes(payload[7:8], 'little'),
            'hour': int.from_bytes(payload[8:9], 'little'),
            'min': int.from_bytes(payload[9:10], 'little'),
            'sec': int.from_bytes(payload[10:11], 'little'),
            'valid': int.from_bytes(payload[11:12], 'little'),
            'tAcc': int.from_bytes(payload[12:16], 'little'),
            'nano': int.from_bytes(payload[16:20], 'little'),
            'fixType': int.from_bytes(payload[20:21], 'little'),
            'flags': int.from_bytes(payload[21:22], 'little'),
            'flags2': int.from_bytes(payload[22:23], 'little'),
            'numSV': int.from_bytes(payload[23:24], 'little'),
            'lon': int.from_bytes(payload[24:28], 'little')*1e-7,
            'lat': int.from_bytes(payload[28:32], 'little')*1e-7,
            'height': int.from_bytes(payload[32:36], 'little'),
            'hMSL': int.from_bytes(payload[36:40], 'little'),
            'hAcc': int.from_bytes(payload[40:44], 'little'),
            'vAcc': int.from_bytes(payload[44:48], 'little'),
            'velN': int.from_bytes(payload[48:52], 'little'),
            'velE': int.from_bytes(payload[52:56], 'little'),
            'velD': int.from_bytes(payload[56:60], 'little'),
            'gSpeed': int.from_bytes(payload[60:64], 'little'),
            'headMot': int.from_bytes(payload[64:68], 'little')*1e-5,
            'sAcc': int.from_bytes(payload[68:72], 'little'),
            'headAcc': int.from_bytes(payload[72:76], 'little')*1e-5,
            'pDop': int.from_bytes(payload[76:78], 'little')*0.01,
            'flags3': int.from_bytes(payload[78:80], 'little'),
            'reserved1': int.from_bytes(payload[80:84], 'little'),
            'headVeh': int.from_bytes(payload[84:88], 'little')*1e-5,
            'magDec': int.from_bytes(payload[88:90], 'little')*1e-2,
            'magAcc': int.from_bytes(payload[90:92], 'little')*1e-2,
        }
    else:
        cfg = {}

    return cfg
def parse_ubx_cfg_prt_packet(packet):
    if(packet):
        cfg = {
            'portID':packet['payload'][0],
            'reserved1': packet['payload'][1],
            'txReady': int.from_bytes(packet['payload'][2:4], 'little'),
            'mode': int.from_bytes(packet['payload'][4:8], 'little'),
            'reserved2':int.from_bytes(packet['payload'][8:12], 'little'),
            'inProtoMask':int.from_bytes(packet['payload'][12:14], 'little'),
            'outProtoMask':int.from_bytes(packet['payload'][14:16], 'little'),
            'flags':int.from_bytes(packet['payload'][16:18], 'little'),
            'reserved3':int.from_bytes(packet['payload'][18:], 'little'),
        }
    else:
        cfg = {}
    return cfg


def parse_ubx_cfg_nav5_packet(packet):
    if(packet):
        payload = packet['payload']
        cfg = {
            'mask':int.from_bytes(payload[0:2], 'little'),
            'dynModel':int.from_bytes(payload[2:3], 'little'),
            'fixMode':int.from_bytes(payload[3:4], 'little'),
            'fixedAlt':int.from_bytes(payload[4:8], 'little'),
            'fixedAltVar':int.from_bytes(payload[8:12], 'little'),
            'minElev':int.from_bytes(payload[12:13], 'little'),
            'drLimit':int.from_bytes(payload[13:14], 'little'),
            'pDop':int.from_bytes(payload[14:16], 'little'),
            'tDop':int.from_bytes(payload[16:18], 'little'),
            'pAcc':int.from_bytes(payload[18:20], 'little'),
            'tAcc':int.from_bytes(payload[20:22], 'little'),
            'staticHoldThresh':int.from_bytes(payload[22:23], 'little'),
            'dgnssTimeout':int.from_bytes(payload[23:24], 'little'),
            'cnoThreshNumSVs':int.from_bytes(payload[24:25], 'little'),
            'cnoThresh':int.from_bytes(payload[25:26], 'little'),
            'reserved1':int.from_bytes(payload[26:28], 'little'),
            'staticHoldMaxDist':int.from_bytes(payload[28:30], 'little'),
            'utcStandard':int.from_bytes(payload[30:31], 'little'),
            'reserved2':int.from_bytes(payload[31:36], 'little')

        }
        return cfg
    else:
        return {}

def parse_ubx_cfg_rate_packet(packet):
    if(packet):
        payload = packet['payload']
        cfg = {
            'measRate':int.from_bytes(payload[0:2], 'little'),
            'navRate':int.from_bytes(payload[2:4], 'little'),
            'timeRef':int.from_bytes(payload[4:6], 'little'),
        }
        return cfg
    else:
        return {}

def parse_ack_or_nak_msg(packet, expectedCls, expectedId):
    retVal = -2

    if packet:
        if packet['class'] == 0x05:
            if packet['payload'][0] == expectedCls and packet['payload'][1] == expectedId:
                if packet['id'] == 0x01:
                    retVal = 0
                    print("ACK-ACK")
                elif packet['id'] == 0x00:
                    print("ACK-NAK")
                    retVal = -1
        
    return retVal

class iDriver:
    def __init__(self, i2c):
        self._i2c = i2c
        self._i2c = self._i2c.setspeed(100)
        
    
    def config_port(self, i2c, in_config, out_config):
        payload= [0]*20
        payload[4] = I2C_ADDR << 1
        payload[12] = in_config
        payload[14] = out_config
        packet = create_ubx_packet(UBX_CFG_PRT_CLASS, UBX_CFG_PRT_ID, payload)

        self.send_packet(i2c, packet)

        read_packet = []
        cnt = 0
        while (len(read_packet) == 0) and cnt < 10:
            read_packet = self.read_buffer(i2c)
            cnt += 1

        if read_packet:    
            read_packet = read_packet[read_packet.index(0xB5):]

            parsed = parse_ubx_packet(read_packet)
            if( parse_ack_or_nak_msg(parsed, UBX_CFG_PRT_CLASS, UBX_CFG_PRT_ID) == 0):
            
                print("Config Port Successfully Set")
            else:
                print("Config Port Not Successfully Set - NAK")
        else:
            print("No results")

    def config_port_msg(self, i2c, ubx_class, ubx_id, rate):
        print(f"Setting -CFG-MSG: Class={ubx_class}, ID={ubx_id}, Rate={rate}")


        payload = [ubx_class, ubx_id, rate]
        packet = create_ubx_packet(UBX_CFG_PRT_CLASS, UBX_CFG_MSG, payload)
        self.send_packet(i2c, packet)

        read_packet = []
        cnt = 0
        while (len(read_packet) == 0) and cnt < 10:
            read_packet = self.read_buffer(i2c)
            cnt += 1

        if read_packet:    
            read_packet = read_packet[read_packet.index(0xB5):]

            parse_ubx_packet(read_packet)
        else:
            print("No results")


    def read_ubx(self, i2c, ubx_class, ubx_id, payload=[], delay=0):
        payload = payload
        packet = create_ubx_packet(ubx_class, ubx_id, payload)
        self.send_packet(i2c, packet)

        if(delay > 0):
            time.sleep(delay)

        flag = False
        buf_packet = []
        cnt = 0
        while not flag:
            if cnt > 10:
                return buf_packet
            cnt += 1
            buf = self.read_buffer(i2c)
            if(buf):
                flag = True
        
        if len(buf) > 0:
            buf = buf[buf.index(0xB5):]
            buf_packet = parse_ubx_packet(buf)
        return buf_packet

    def read_version(self, i2c):
        print("\n\n***Reading Version ***\n")
        ver_packet = self.read_ubx(i2c, 0x0A, 0x04);
        print("-MON-VER Packet Contents")
        pprint(ver_packet)
        return ver_packet
 
    def read_config(self, i2c):
        print("\n\n***Reading Config Port***\n")
        cfg_packet = self.read_ubx(i2c, UBX_CFG_PRT_CLASS, UBX_CFG_PRT_ID);
        cfg_parsed = parse_ubx_cfg_prt_packet(cfg_packet)
        print("-CFG-PRT Packet Contents")
        pprint(cfg_packet)
        print("\n-CFG-PRT Packet Parsed")
        pprint(cfg_parsed)
        return cfg_packet

    def read_cfg_nav5(self, i2c):
        print("\n\n***Reading Config NAV5***\n")
        packet = self.read_ubx(i2c, UBX_CFG_PRT_CLASS, UBX_CFG_NAV5);
        print("-CFG-NAV5 Packet Contents")
        pprint(packet)
        parsed = parse_ubx_cfg_nav5_packet(packet)
        print("\n-CFG-NAV5 Packet Parsed")
        pprint(parsed)

        return packet

    def read_cfg_msg(self, i2c, ubx_class=[], ubx_id=[]):
        

        if ubx_class and ubx_id:
            print(f"\n\n***Reading Config MSG: Class = {ubx_class}, ID={ubx_id}***\n")
            payload = [ubx_class, ubx_id]
        else:
            print("\n\n***Reading Config MSG***\n")
            payload = []
        packet = self.read_ubx(i2c, UBX_CFG_PRT_CLASS, UBX_CFG_MSG, payload);
        
        print("-CFG-MSG Packet Contents")
        pprint(packet)

        return packet

    def read_cfg_rate(self, i2c):
        print("\n\n***Reading Config RATE***\n")
        packet = self.read_ubx(i2c, UBX_CFG_PRT_CLASS, UBX_CFG_RATE);
        
        print("-CFG-RATE Packet Contents")
        pprint(packet)

        print("\n-CFG-RATE Packet Parsed")
        parsed = parse_ubx_cfg_rate_packet(packet)
        pprint(parsed)

        return packet

    def read_nav(self, i2c):
        print("\n\n***Reading NAV-PVT***\n")
        packet = self.read_ubx(i2c, UBX_NAV_CLASS, UBX_NAV_PVT, delay=0.5);
        print("-NAV-PVT Packet Contents")
        pprint(packet)
        print("\n-CFG-PRT Packet Parsed")
        parsed = parse_ubx_nav_pvt_packet(packet)
        pprint(parsed)
        return packet

    def read_nav_status(self, i2c):
        print("\n\n***Reading NAV-STATUS***\n")
        nav_packet = self.read_ubx(i2c, UBX_NAV_CLASS, UBX_NAV_STATUS);
        print("-NAV-PVT Packet Contents")
        pprint(nav_packet)
        return nav_packet

    def read_nav_sat(self, i2c):
        print("\n\n***Reading NAV-SAT***\n")
        nav_packet = self.read_ubx(i2c, UBX_NAV_CLASS, UBX_NAV_SAT);
        print("-NAV-SAT Packet Contents")
        pprint(nav_packet)
        return nav_packet

    def send_packet(self, i2c, packet):
        self._i2c = i2c
        self._i2c.start(I2C_ADDR, 0)
        self._i2c.write(packet)

    def read_buffer(self, i2c):
        # valid_flag = False
        cnt = 0
        # while not valid_flag:
            # if cnt > 1:
            #     self._i2c.stop()
            #     return []
            # cnt += 1

        self._i2c = i2c
        self._i2c.start(I2C_ADDR, 0)
        self._i2c.write([0xFD])


        self._i2c.start(0x42, 1)
        num_bytes = i2c.read(2)
        num_bytes = int.from_bytes(num_bytes, byteorder='big')
        self._i2c.stop()

        if num_bytes == 0xFFFF:
            print("Error num_bytes = 0xFFFF")
            return []

        readval = []
        if num_bytes > 0:
            self._i2c.start(0x42, 0)
            self._i2c.write([0xFF])

            self._i2c.start(0x42, 1)
            readval = self._i2c.read(num_bytes)
            self._i2c.stop()
        return readval

    



    

if __name__ == "__main__":

    i2c_drive = i2cdriver.I2CDriver(port='COM23', reset=False)
    i2c_drive.reboot()
    try:
        i2c = iDriver(i2c_drive)

        i2c.read_version(i2c_drive)

        i2c.read_config(i2c_drive)

        i2c.config_port(i2c_drive, 1, 1)

        i2c.read_config(i2c_drive)

        i2c.read_cfg_nav5(i2c_drive)

        i2c.read_cfg_rate(i2c_drive)

        i2c.read_cfg_msg(i2c_drive)

        i2c.read_cfg_msg(i2c_drive, UBX_NAV_CLASS, UBX_NAV_PVT)

        i2c.config_port_msg(i2c_drive, UBX_NAV_CLASS, UBX_NAV_PVT, 250)

        i2c.read_cfg_msg(i2c_drive, UBX_NAV_CLASS, UBX_NAV_PVT)

        for i in range(25):
            i2c.read_nav(i2c_drive)

    finally:
        i2c_drive.stop()
        i2c_drive.reboot()

        del(i2c)
        del(i2c_drive)
