import os
import time
import sys
import math
import serial
import serial.tools.list_ports
import struct
import datetime

preamble = bytearray.fromhex('5555')
packet_def = {'s1': [43, bytearray.fromhex('7331')],\
              's2': [43, bytearray.fromhex('7332')],\
              'iN': [45, bytearray.fromhex('694E')],\
              'd1': [37, bytearray.fromhex('6431')],\
              'd2': [31, bytearray.fromhex('6432')],\
              'gN': [53, bytearray.fromhex('674E')],\
              'sT': [38, bytearray.fromhex('7354')]}

class rtk330l:
    def __init__(self, port, baud=115200, packet_type='sT', pipe=None):
        self.port = port
        self.baud = baud
        self.physical_port = True
        self.file_size = 0
        if baud > 0:
            self.ser = serial.Serial(self.port, self.baud)
            self.open = self.ser.isOpen()
        else:
            self.ser = open('user_2021_09_01_13_59_08.bin', 'rb')
            self.open = True
            self.physical_port = False
            self.file_size = os.path.getsize('user_2021_09_01_13_59_08.bin')
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

        self.f = open("data/rtk330la_data_" + time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime()) + '.txt', "w")

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
                # print(data)
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
                        # if self.latest[0]%5 == 0:
                        #print(self.bf[:self.size-1].hex())
                        cur_tm2 = time.time()
                        print(cur_tm2, self.latest)
                        timestrt_hms = datetime.datetime.now().strftime('%H:%M:%S.%f')
                        timestrt_ymd = time.strftime("%Y-%m-%d", time.localtime())                        
                        self.f.writelines(str(cur_tm2) + str(timestrt_hms) + "=" + str(self.latest) + '\n')
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
        fmt = '=Idffffff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        accels = data[2:4]
        angles = data[5:7]

        return gps_week, time_of_week, accels, angles

    def parse_s2(self, payload):
        fmt = '=Idffffff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        accels = data[2:4]
        angles = data[5:7]

        return gps_week, time_of_week, accels, angles

    def parse_iN(self, payload):
        fmt = 'IdBBIIfhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        ins_status = data[2]
        ins_pos_status = data[3]
        latitude = data[4]
        longitude = data[5]
        hight = data[6]
        velocity_north = data[7]
        velocity_east = data[8]
        velocity_up = data[9]
        roll = data[10]
        pitch = data[11]
        head = data[12]

        return gps_week, time_of_week, ins_status, ins_pos_status, latitude, longitude, hight, velocity_north, \
            velocity_east, velocity_up, roll, pitch, head

    def parse_d1(self, payload):
        fmt = '=Idhhhhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        latitude = data[2]
        longitude = data[3]
        hight = data[4]
        velocity_north = data[5]
        velocity_east = data[6]
        velocity_up = data[7]
        roll = data[8]
        pitch = data[9]
        head = data[10]

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up, \
            roll, pitch, head

    def parse_d2(self, payload):
        fmt = 'Idhhhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        latitude = data[2]
        longitude = data[3]
        hight = data[4]
        velocity_north = data[5]
        velocity_east = data[6]
        velocity_up = data[7]

        return gps_week, time_of_week, latitude, longitude, hight, velocity_north, velocity_east, velocity_up

    def parse_gN(self, payload):
        fmt = '<IdBIIfBfffhhhh'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        pos_mode = data[2]
        latitude = data[3]
        longitude = data[4]
        hight = data[5]
        num_of_SVs = data[6]
        hdop = data[7]
        vdop = data[8]
        tdop = data[9]
        diffage = data[10]
        velocity_north = data[11]
        velocity_east = data[12]
        velocity_up = data[13]

        return gps_week, time_of_week, pos_mode, latitude, longitude, hight, num_of_SVs, hdop, vdop, tdop, \
            diffage, velocity_north, velocity_east, velocity_up

    def parse_sT(self, payload):
        fmt = '<IdhBBBBBIff'
        data = struct.unpack(fmt, payload)
        gps_week = data[0]
        time_of_week = data[1]
        year = data[2]
        month = data[3]
        day = data[4]
        hour = data[5]
        minute = data[6]
        sec = data[7]
        imu_status = data[8]
        imu_temp = data[9]
        mcu_temp = data[10]

        T = time_of_week
        num_week = int((T-18) / 86400)
        H = (int((T-18) / 3600) % 24) + 8
        M = int((T-18) / 60) % 60
        S = (T-18) - (int((T-18) / 60) * 60)
        time_stamp = (str(num_week) + "-" + str(H) + "-" + str(M) + "-" + str(S))

        return gps_week, time_of_week, year, month, day, hour, minute, sec, imu_status, \
             imu_temp, mcu_temp, time_stamp
        

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
    port = '/dev/ttyUSB0'
    baud = 230400
    packet_type = 'sT'

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


