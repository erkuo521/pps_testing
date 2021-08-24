import os
import time
import sys
import math
import serial
import serial.tools.list_ports
import struct

preamble = bytearray.fromhex('5555')
packet_def = {'s1': [43, bytearray.fromhex('7331')],\
              's2': [43, bytearray.fromhex('7332')],\
              'iN': [45, bytearray.fromhex('694E')],\
              'd1': [37, bytearray.fromhex('6431')],\
              'd2': [31, bytearray.fromhex('6432')],\
              'gN': [53, bytearray.fromhex('674E')],\
              'sT': [38, bytearray.fromhex('7354')]}

class rtk330l:
    def __init__(self, port, baud=115200, packet_type='gN', pipe=None):
        self.port = port
        self.baud = baud
        self.physical_port = True
        self.file_size = 0
        if baud > 0:
            self.ser = serial.Serial(self.port, self.baud)
            self.open = self.ser.isOpen()
        else:
            self.ser = open(port, 'rb')
            self.open = True
            self.physical_port = False
            self.file_size = os.path.getsize(port)
        self.latest = []
        self.ready = False
        self.pipe = pipe
        self.size = 0
        self.header = None
        self.parser = None
        if packet_type in packet_def.keys():
            self.size = packet_def[packet_type][0]
            self.header = packet_def[packet_type][1]
            self.parser = eval('self.parse_' + packet_type)
        else:
            self.open = False
            print('Unsupported packet type: %s'% packet_type)
        self.bf = bytearray(self.size*2)
        self.nbf = 0

    def start(self, reset=False, reset_cmd='5555725300FC88'):
        if self.open:
            # send optional reset command if port is a pysical serial port
            if self.physical_port:
                if reset is True:
                    self.ser.write(bytearray.fromhex(reset_cmd))
                self.ser.reset_input_buffer()
            while True:
                if self.physical_port:
                    read_size = self.ser.in_waiting
                else:
                    read_size = self.file_size
                data = self.ser.read(read_size)
                if not data:
                    # end processing if reaching the end of the data file
                    if not self.physical_port:
                        break
                else:
                    # parse new coming data
                    self.parse_new_data(data)
            #close port or file
            self.ser.close()
            print('End of processing.')
            self.pipe.send('exit')
        
    def parse_new_data(self, data):
        '''
        add new data in the buffer
        '''
        n = len(data)
        for i in range(n):
            self.bf[self.nbf] = data[i]
            self.nbf += 1
            while self.nbf >= self.size:
                if self.bf[0] == preamble[0] and self.bf[1] == preamble[1] and\
                    self.bf[2] == self.header[0] and self.bf[3] == self.header[1]:
                    # crc
                    packet_crc = 256 * self.bf[self.size-2] + self.bf[self.size-1]
                    calculated_crc = self.calc_crc(self.bf[2:self.bf[4]+5])
                    # decode
                    if packet_crc == calculated_crc:
                        self.latest = self.parse_packet(self.bf[2:self.bf[4]+5])
                        if self.latest[0]%5 == 0:
                            print(self.latest) 
                        if self.pipe is not None:
                            self.pipe.send(self.latest)
                        self.nbf -= self.size
                        for i in range(self.nbf):
                            self.bf[i] = self.bf[i+self.size]
                    else:
                        print('crc fail: %s %s %s %s'% (self.size, self.nbf, packet_crc, calculated_crc))
                        print(" ".join("{:02X}".format(self.bf[i]) for i in range(0, self.nbf)))
                        # remove the first byte from the buffer
                        self.nbf -= 1
                        for i in range(self.nbf):
                            self.bf[i] = self.bf[i+1]
                        self.nbf = self.sync_packet(self.bf, self.nbf, preamble)
                else:
                    self.nbf = self.sync_packet(self.bf, self.nbf, preamble)

    def get_latest(self):
        a = self.latest
        return a

    def parse_packet(self, payload):
        data = self.parser(payload[3::])
        return data

    def parse_s1(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3])

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11]) * 1000

        accels = [0 for x in range(3)]
        for i in range(3):
            accels[i] = (math.pow(4, 12) * payload[4*i+12] + math.pow(4, 8) * payload[4*i+13] + \
            math.pow(4, 4) * payload[4*i+14] + payload[4*i+15])
        
        angles = [0 for x in range(3)]
        for i in range(3):
            angles[i] = (math.pow(4, 12) * payload[4*i+16] + math.pow(4, 8) * payload[4*i+17] + \
            math.pow(4, 4) * payload[4*i+18] + payload[4*i+19])

        return gps_week, time_of_week, accels, angles
       
    def parse_s2(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3])

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11]) * 1000

        accels = [0 for x in range(3)]
        for i in range(3):
            accels[i] = (math.pow(4, 12) * payload[4*i+12] + math.pow(4, 8) * payload[4*i+13] + \
            math.pow(4, 4) * payload[4*i+14] + payload[4*i+15])
        
        angles = [0 for x in range(3)]
        for i in range(3):
            angles[i] = (math.pow(4, 12) * payload[4*i+16] + math.pow(4, 8) * payload[4*i+17] + \
            math.pow(4, 4) * payload[4*i+18] + payload[4*i+19])

        return gps_week, time_of_week, accels, angles

    def parse_iN(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3])

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11]) 
        
        ins_status = payload[12]
        ins_pos_status = payload[13]

        latitude = (math.pow(4, 12) * payload[14] + math.pow(4, 8) * payload[15] + \
            math.pow(4, 4) * payload[16] + payload[17]) * (180 / (math.pow(2,31)))

        longitude = (math.pow(4, 12) * payload[18] + math.pow(4, 8) * payload[19] + \
            math.pow(4, 4) * payload[20] + payload[21]) * (180 / (math.pow(2,31)))
        
        hight = (math.pow(4, 12) * payload[22] + math.pow(4, 8) * payload[23] + \
            math.pow(4, 4) * payload[24] + payload[25])

        velocity_north = (math.pow(4, 4) * payload[26] + payload[27])
        velocity_east = (math.pow(4, 4) * payload[28] + payload[29])
        velocity_up = (math.pow(4, 4) * payload[30] + payload[31])

        roll = (math.pow(4, 4) * payload[32] + payload[33])    
        pitch = (math.pow(4, 4) * payload[34] + payload[35]) 
        head = (math.pow(4,4) * payload[36] + payload[37]) 

        return gps_week, time_of_week, ins_status, ins_pos_status, latitude, longitude, hight, velocity_north, \
            velocity_east, velocity_up, roll, pitch, head
    
    def parse_d1(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3]) 

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11])
        
        latitude = (math.pow(4, 4) * payload[12] + payload[13]) / 100
        longitude = (math.pow(4, 4) * payload[14] + payload[15]) / 100
        hight = (math.pow(4, 4) * payload[16] + payload[17]) / 100 

        velocity_north = (math.pow(4, 4) * payload[18] + payload[19]) / 100 
        velocity_east = (math.pow(4, 4) * payload[20] + payload[21]) / 100
        velocity_up = (math.pow(4, 4) * payload[22] + payload[23]) / 100

        roll = (math.pow(4, 4) * payload[24] + payload[25]) / 100 
        pitch = (math.pow(4, 4) * payload[26] + payload[27]) / 100
        head = (math.pow(4, 4) * payload[28] + payload[29]) / 100

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up, roll, pitch, head

    def parse_d2(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3]) 

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11])

        latitude = (math.pow(4, 4) * payload[12] + payload[13]) / 100 
        longitude = (math.pow(4, 4) * payload[14] + payload[15]) / 100 
        hight = (math.pow(4, 4) * payload[16] + payload[17]) / 100

        velocity_north = (math.pow(4, 4) * payload[18] + payload[19]) / 100
        velocity_east = (math.pow(4, 4) * payload[20] + payload[21]) / 100 
        velocity_up = (math.pow(4, 4) * payload[22] + payload[23]) /100

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up

    def parse_gN(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3]) 

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11])

        pos_mode = payload[12]
        
        latitude = (math.pow(4, 12) * payload[13] + math.pow(4, 8) * payload[14] + \
            math.pow(4, 4) * payload[15] + payload[16]) * (8 / math.pow(2, 31))

        longitude = (math.pow(4, 12) * payload[17] + math.pow(4, 8) * payload[18] + \
            math.pow(4, 4) * payload[19] + payload[20]) * (8 / math.pow(2, 31))

        hight = (math.pow(4, 12) * payload[21] + math.pow(4, 8) * payload[22] + \
            math.pow(4, 4) * payload[23] + payload[24]) * (8 / math.pow(2, 31)) 
        
        num_of_SVs = payload[25]

        hdop = (math.pow(4, 12) * payload[26] + math.pow(4, 8) * payload[27] + \
            math.pow(4, 4) * payload[28] + payload[29]) * (8 / math.pow(2, 31)) 
        
        vdop = (math.pow(4, 12) * payload[30] + math.pow(4, 8) * payload[31] + \
            math.pow(4, 4) * payload[32] + payload[33]) * (8 / math.pow(2, 31)) 

        tdop = (math.pow(4, 12) * payload[34] + math.pow(4, 8) * payload[35] + \
            math.pow(4, 4) * payload[36] + payload[37]) * (8 / math.pow(2, 31)) 

        diffage = (math.pow(4,4) * payload[38] + payload[39])

        velocity_north = (math.pow(4,4) * payload[40] + payload[41])
        velocity_east = (math.pow(4,4) * payload[42] + payload[43])
        velocity_up = (math.pow(4,4) * payload[44] + payload[45])

        return gps_week, time_of_week, pos_mode, latitude, longitude, hight, num_of_SVs, hdop, vdop, tdop, \
            diffage, velocity_north, velocity_east, velocity_up

    def parse_sT(self, payload):
        gps_week = (math.pow(4, 12) * payload[0] + math.pow(4, 8) * payload[1] + \
            math.pow(4, 4) * payload[2] + payload[3]) 

        time_of_week = (math.pow(4, 28) * payload[4] + math.pow(4, 24) * payload[5] + \
            math.pow(4, 20) * payload[6] + math.pow(4,16) * payload[7] + math.pow(4, 12) * \
            payload[8] + math.pow(4, 8) * payload[9] + math.pow(4, 4) * payload[10] + \
            payload[11])
        
        year = (math.pow(4, 4) * payload[12] + payload[13])
        month = payload[14]
        day = payload[15]
        hour = payload[16]
        minute = payload[17]
        sec = payload[18]

        imu_status = int(math.pow(4,12) * payload[19] + math.pow(4, 8) * payload[20] + \
            math.pow(4, 4) * payload[21] + payload[22])

        imu_temp = (math.pow(4, 12) * payload[23] + math.pow(4, 8) * payload[24] + \
            math.pow(4, 4) * payload[25] + payload[26])
        
        mcu_temp = (math.pow(4, 12) * payload[27] + math.pow(4, 8) * payload[28] + \
            math.pow(4, 4) * payload[29] + payload[30])

        return  gps_week, time_of_week, year, month, day, hour, minute, sec, imu_status, imu_temp, mcu_temp
        
    def sync_packet(self, bf, bf_len, preamble):
        idx = -1
        while 1:
            idx = bf.find(preamble[0], idx+1, bf_len)
            # first byte of the header not found
            if idx < 0:
                bf_len = 0
                break
            # first byte of the header is found and there is enough bytes in buffer
            #   to match the header and packet type
            elif idx <= (bf_len-4):
                if bf[idx+1] == preamble[1] and\
                    bf[idx+2] == self.header[0] and bf[idx+3] == self.header[1]:
                    bf_len = bf_len - idx
                    for i in range(bf_len):
                        bf[i] = bf[i+idx]
                    break
                else:
                    continue
            # first byte of the header is found, but there is not enough bytes in buffer
            #   to match the header and packet type
            else:
                bf_len = bf_len - idx
                for i in range(bf_len):
                    bf[i] = bf[i+idx]
                break
        return bf_len

    def calc_crc(self, payload):
        crc = 0x1D0F
        for bytedata in payload:
            crc = crc^(bytedata << 8) 
            for i in range(0,8):
                if crc & 0x8000:
                    crc = (crc << 1)^0x1021
                else:
                    crc = crc << 1

        crc = crc & 0xffff
        return crc

if __name__ == "__main__":
    port = 'COM3'
    baud = 115200
    packet_type = 'gN'

    num_of_args = len(sys.argv)
    if num_of_args > 1:
        port = sys.argv[1]
        if num_of_args > 2:
            baud = int(sys.argv[2])
            if num_of_args > 3:
                packet_type = sys.argv[3]
    # run
    unit = rtk330l(port, baud, packet_type, pipe=None)
    unit.start()
