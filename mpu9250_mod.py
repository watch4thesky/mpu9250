# Based on code from Kevin J. Walchko and the Adafruit library
# https://github.com/MomsFriendlyRobotCompany/mpu9250.git
# https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library

# Register map for the mpu9250
# https://www.invensense.com/wp-content/uploads/2015/02/RM-MPU-9250A-00-v1.6.pdf

from __future__ import print_function
from __future__ import division

try:
    import smbus2 as smbus
except ImportError:
    print('WARNING: Using fake hardware')
    from .fakeHW import smbus
    # from fake_rpi import smbus

from time import sleep
import struct
import ctypes # for signed int

# Martin imports:
import numpy as np
import operator
import time


# Todo:
# - replace all read* with the block read?

################################
# MPU9250
################################
MPU9250_ADDRESS = 0x68
AK8963_ADDRESS  = 0x0C
DEVICE_ID       = 0x71
WHO_AM_I        = 0x75
PWR_MGMT_1      = 0x6B
PWR_MGMT_2      = 0x6C
INT_PIN_CFG     = 0x37
INT_ENABLE      = 0x38
# --- Accel ------------------
ACCEL_DATA    = 0x3B
ACCEL_CONFIG  = 0x1C
ACCEL_CONFIG2 = 0x1D
ACCEL_2G      = 0x00
ACCEL_4G      = (0x01 << 3)
ACCEL_8G      = (0x02 << 3)
ACCEL_16G     = (0x03 << 3)
# --- Temp --------------------
TEMP_DATA = 0x41
# --- Gyro --------------------
GYRO_DATA    = 0x43
GYRO_CONFIG  = 0x1B
GYRO_250DPS  = 0x00
GYRO_500DPS  = (0x01 << 3)
GYRO_1000DPS = (0x02 << 3)
GYRO_2000DPS = (0x03 << 3)

# --- AK8963 ------------------
MAGNET_DATA  = 0x03
AK_DEVICE_ID = 0x48
AK_WHO_AM_I  = 0x00
AK8963_8HZ   = 0x02
AK8963_100HZ = 0x06
AK8963_14BIT = 0x00
AK8963_16BIT = (0x01 << 4)
AK8963_CNTL1 = 0x0A
AK8963_CNTL2 = 0x0B
AK8963_ASAX  = 0x10
AK8963_ST1   = 0x02
AK8963_ST2   = 0x09
AK8963_ASTC  = 0x0C
ASTC_SELF    = 0x01<<6

# --- ADDED BY M ----------
XG_OFFSET_H     =   0x13  # User-defined trim values for gyroscope
XG_OFFSET_L     =   0x14
YG_OFFSET_H     =   0x15
YG_OFFSET_L     =   0x16
ZG_OFFSET_H     =   0x17
ZG_OFFSET_L     =   0x18

FIFO_COUNTH     =   0x72
FIFO_COUNTL     =   0x73
FIFO_R_W        =   0x74

FIFO_EN         =   0x23
I2C_MST_CTRL    =   0x24
USER_CTRL       =   0x6A  # Bit 7 enable DMP, bit 3 reset DMP

CONFIG          =   0x1A
SMPLRT_DIV      =   0x19

class mpu9250(object):
    def __init__(self, mpu9250_address, bus=1):

        self.MPU9250_ADDRESS = mpu9250_address
        self.AK8963_ADDRESS  = 0x0C

        self.DEVICE_ID       = 0x71
        self.WHO_AM_I        = 0x75
        self.PWR_MGMT_1      = 0x6B
        self.PWR_MGMT_2      = 0x6C
        self.INT_PIN_CFG     = 0x37
        self.INT_ENABLE      = 0x38
        # --- Accel ------------------
        self.ACCEL_DATA    = 0x3B
        self.ACCEL_CONFIG  = 0x1C
        self.ACCEL_CONFIG2 = 0x1D
        self.ACCEL_2G      = 0x00
        self.ACCEL_4G      = (0x01 << 3)
        self.ACCEL_8G      = (0x02 << 3)
        self.ACCEL_16G     = (0x03 << 3)
        # --- Temp --------------------
        self.TEMP_DATA = 0x41
        # --- Gyro --------------------
        self.GYRO_DATA    = 0x43
        self.GYRO_CONFIG  = 0x1B
        self.GYRO_250DPS  = 0x00
        self.GYRO_500DPS  = (0x01 << 3)
        self.GYRO_1000DPS = (0x02 << 3)
        self.GYRO_2000DPS = (0x03 << 3)

        # --- AK8963 ------------------
        self.MAGNET_DATA  = 0x03
        self.AK_DEVICE_ID = 0x48
        self.AK_WHO_AM_I  = 0x00
        self.AK8963_8HZ   = 0x02
        self.AK8963_100HZ = 0x06
        self.AK8963_14BIT = 0x00
        self.AK8963_16BIT = (0x01 << 4)
        self.AK8963_CNTL1 = 0x0A
        self.AK8963_CNTL2 = 0x0B
        self.AK8963_ASAX  = 0x10
        self.AK8963_ST1   = 0x02
        self.AK8963_ST2   = 0x09
        self.AK8963_ASTC  = 0x0C
        self.ASTC_SELF    = 0x01<<6
        """
        Setup the IMU

        reg 0x25: SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
        reg 0x29: [2:0] A_DLPFCFG Accelerometer low pass filter setting
            ACCEL_FCHOICE 1
            A_DLPF_CFG 4
            gives BW of 20 Hz
        reg 0x35: FIFO disabled default - not sure i want this ... just give me current reading

        might include an interface where you can change these with a dictionary:
            setup = {
                ACCEL_CONFIG: ACCEL_4G,
                GYRO_CONFIG: AK8963_14BIT | AK8963_100HZ
            }
        """
        self.bus = smbus.SMBus(bus)

        # let's double check we have the correct device address
        ret = self.read8(self.MPU9250_ADDRESS, WHO_AM_I)
        if ret is not DEVICE_ID:
            raise Exception('MPU9250: init failed to find device')
        
        """
        sleep(0.2)
        ret = self.read8(AK8963_ADDRESS, AK_WHO_AM_I)
        if ret is not AK_DEVICE_ID:
            raise Exception('AK8963: init failed to find device')
        """

    def initMPU9250(self):
        
        print("Init MPU9250 - CHECK THIS WITH KRIS WARNER CODE")

        # Let's double check we have the correct device address
        ret = self.read8(self.MPU9250_ADDRESS, WHO_AM_I)
        if ret is not DEVICE_ID:
            raise Exception('MPU9250: init failed to find device')

        self.write(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x00)  # turn sleep mode off
        sleep(0.1)
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x01)  # auto select clock source
        sleep(0.2)
        self.write(self.MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_2G) # Set accelerometer sensistivity
        self.write(self.MPU9250_ADDRESS, GYRO_CONFIG, GYRO_250DPS) # Set gyroscope sensistivity

        # THIS SETUP IS FOR US READING DATA AT 250 Hz (Avoid aliasing)
        self.bus.write_byte_data(self.MPU9250_ADDRESS, CONFIG, 0x02)      # Set low-pass filter to 92 Hz  (0x01 = 184 Hz, 0x03 = 42 Hz)
        self.bus.write_byte_data(self.MPU9250_ADDRESS, SMPLRT_DIV, 0x03) # Set data output-rate to 250 Hz

        # You have to enable the other chips to join the I2C network
        # then you should see 0x68 and 0x0c from:
        # sudo i2cdetect -y 1
        self.write(self.MPU9250_ADDRESS, INT_PIN_CFG, 0x22)
        self.write(self.MPU9250_ADDRESS, INT_ENABLE, 0x01)
        sleep(0.1)

        ret = self.read8(AK8963_ADDRESS, AK_WHO_AM_I)
        if ret is not AK_DEVICE_ID:
            raise Exception('AK8963: init failed to find device')
        self.write(AK8963_ADDRESS, AK8963_CNTL1, (AK8963_16BIT | AK8963_100HZ)) # cont mode 1
        self.write(AK8963_ADDRESS, AK8963_ASTC, 0)
        
        #normalization coefficients 
        self.alsb = 2.0 / 32760 # ACCEL_2G # 32767???
        self.alsb_ms2 = 2.0*9.82 / 32760 # in m/s^2
        self.glsb = 250.0 / 32760 # GYRO_250DPS
        self.glsb_rads = 250*3.141592 / (180*32760) # GYRO_250DPS in rad/s
        self.mlsb = 4800.0 / 32760 # MAGNET range +-4800

        # i think i can do this???
        # self.convv = struct.Struct('<hhh')

    def __del__(self):
        self.bus.close()

    def write(self, address, register, value):
        self.bus.write_byte_data(address, register, value)

    def read8(self, address, register):
        data = self.bus.read_byte_data(address, register)
        return data

    def read16(self, address, register):
        data = self.bus.read_i2c_block_data(address, register, 2)
        return self.conv(data[0], data[1])

    def read_xyz(self, address, register, lsb):
        """
        Reads x, y, and z axes at once and turns them into a tuple.
        """
        # data is MSB, LSB, MSB, LSB ...
        data = self.bus.read_i2c_block_data(address, register, 6)

        # data = []
        # for i in range(6):
        # 	data.append(self.read8(address, register + i))

        x = self.conv(data[0], data[1]) * lsb
        y = self.conv(data[2], data[3]) * lsb
        z = self.conv(data[4], data[5]) * lsb

        #print('>> data', data)
        # ans = self.convv.unpack(*data)
        # ans = struct.unpack('<hhh', data)[0]
        # print('func', x, y, z)
        # print('struct', ans)

        return (x, y, z)

    def conv(self, msb, lsb):
        value = lsb | (msb << 8)
        
        return ctypes.c_short(value).value

    @property
    def accel(self):
        return self.read_xyz(self.MPU9250_ADDRESS, ACCEL_DATA, self.alsb)

    @property
    def gyro(self):
        return self.read_xyz(self.MPU9250_ADDRESS, GYRO_DATA, self.glsb)

    @property
    def temp(self):
        """
        Returns chip temperature in C

        pg 33 reg datasheet:
        pg 12 mpu datasheet:
        Temp_room 21
        Temp_Sensitivity 333.87
        Temp_degC = ((Temp_out - Temp_room)/Temp_Sensitivity) + 21 degC
        """
        temp_out = self.read16(self.MPU9250_ADDRESS, TEMP_DATA)
        temp = ((temp_out -21.0)/ 333.87)+21.0 # these are from the datasheets
        
        return temp

    @property
    def mag(self):
        data=self.read_xyz(AK8963_ADDRESS, MAGNET_DATA, self.mlsb)
        self.read8(AK8963_ADDRESS,AK8963_ST2) # needed step for reading magnetic data
        return data

    # Redefined read functions
    def read_accel(self):
        data = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, ACCEL_DATA, 6)
        x = self.conv(data[0], data[1])# * self.alsb
        y = self.conv(data[2], data[3])# * self.alsb
        z = self.conv(data[4], data[5])# * self.alsb
        return (x, y, z)

    def read_gyro(self):
        data = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, GYRO_DATA, 6)
        x = self.conv(data[0], data[1])# * self.glsb
        y = self.conv(data[2], data[3])# * self.glsb
        z = self.conv(data[4], data[5])# * self.glsb
        return (x, y, z)

    def read_mag(self):
        data = self.bus.read_i2c_block_data(AK8963_ADDRESS, MAGNET_DATA, 6)
        # Data stored as little Endian
        x = self.conv(data[1], data[0])# * self.mlsb
        y = self.conv(data[3], data[2])# * self.mlsb
        z = self.conv(data[5], data[4])# * self.mlsb
        self.read8(AK8963_ADDRESS,AK8963_ST2)
        return (x, y, z)

    def read_temp(self):
        temp_out = self.read16(self.MPU9250_ADDRESS, TEMP_DATA)
        temp = ((temp_out -21.0)/ 333.87)+21.0 # these are from the datasheets
        return temp

    def read_gyro_accel(self):
        t1 = time.time()
        data_accel = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, ACCEL_DATA, 6)
        data_gyro = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, GYRO_DATA, 6)
        t2 = time.time()
        tstamp = int((t1+t2)*500000000) # get the average in ns

        x_accel = self.conv(data_accel[0], data_accel[1]) * self.alsb_ms2
        y_accel = self.conv(data_accel[2], data_accel[3]) * self.alsb_ms2
        z_accel = self.conv(data_accel[4], data_accel[5]) * self.alsb_ms2

        x_gyro = self.conv(data_gyro[0], data_gyro[1]) * self.glsb_rads
        y_gyro = self.conv(data_gyro[2], data_gyro[3]) * self.glsb_rads
        z_gyro = self.conv(data_gyro[4], data_gyro[5]) * self.glsb_rads

        return (tstamp, x_gyro, y_gyro, z_gyro, x_accel, y_accel, z_accel)



    # IT SEEMS LIKE THIS IS NOT WORKING!
    def calibrate_gyro(self):
        print("Make sure device is not moving")

        gyro_bias  = [0, 0, 0]
        accel_bias = [0, 0, 0]

        # reset device
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x80) # Write a one to bit 7 reset bit toggle reset device
        sleep(0.1)

        # get stable time source Auto select clock source to be PLL gyroscope reference if ready 
        # else use the internal oscillator, bits 2:0 = 001
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x01)
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_2, 0x00)
        sleep(0.2)

        # Configure device for bias calculation
        self.bus.write_byte_data(self.MPU9250_ADDRESS, INT_ENABLE, 0x00)   # Disable all interrupts
        self.bus.write_byte_data(self.MPU9250_ADDRESS, FIFO_EN, 0x00)      # Disable FIFO
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x00)   # Turn on internal clock source
        self.bus.write_byte_data(self.MPU9250_ADDRESS, I2C_MST_CTRL, 0x00) # Disable I2C master
        self.bus.write_byte_data(self.MPU9250_ADDRESS, USER_CTRL, 0x00)    # Disable FIFO and I2C master modes
        self.bus.write_byte_data(self.MPU9250_ADDRESS, USER_CTRL, 0x0C)    # Reset FIFO and DMP
        sleep(0.015)

        # Configure MPU6050 gyro and accelerometer for bias calculation
        self.bus.write_byte_data(self.MPU9250_ADDRESS, CONFIG, 0x01)      # Set low-pass filter to 188 Hz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, SMPLRT_DIV, 0x00)  # Set sample rate to 1 kHz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, GYRO_CONFIG, 0x00)  # Set gyro full-scale to 250 degrees per second, maximum sensitivity
        self.bus.write_byte_data(self.MPU9250_ADDRESS, ACCEL_CONFIG, 0x00) # Set accelerometer full-scale to 2 g, maximum sensitivity

        gyrosensitivity  = 131   # = 131 LSB/degrees/sec
        accelsensitivity = 16384  # = 16384 LSB/g

        # Configure FIFO to capture accelerometer and gyro data for bias calculation
        self.bus.write_byte_data(self.MPU9250_ADDRESS, USER_CTRL, 0x40)   # Enable FIFO  
        self.bus.write_byte_data(self.MPU9250_ADDRESS, FIFO_EN, 0x78)     # Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        sleep(0.150) # accumulate 40 samples in 40 milliseconds = 480 bytes

        # At end of sample accumulation, turn off FIFO sensor read
        self.bus.write_byte_data(self.MPU9250_ADDRESS, FIFO_EN, 0x00)        # Disable gyro and accelerometer sensors for FIFO
        #readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]) # read FIFO sample count
        data = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, FIFO_COUNTH, 2)
        #fifo_count = ((uint16_t)data[0] << 8) | data[1]
        fifo_count = self.conv(data[0], data[1])
        packet_count = fifo_count//12 # How many sets of full gyro and accelerometer data for averaging

        """
        uint8_t rawData[6];  // x/y/z gyro register data stored here
        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
        """
        print(fifo_count)
        print("Packet count: ", packet_count)


        for i in range(packet_count): # (ii = 0 ii < packet_count ii++) {
            #int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0}
            print(i)
            accel_temp = [0,0,0]
            gyro_temp = [0,0,0]
            #readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]) # read data for averaging
            data = self.bus.read_i2c_block_data(self.MPU9250_ADDRESS, FIFO_R_W, 12)

            """
            accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  )   # Form signed 16-bit integer for each sample in FIFO
            accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) 
            accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  )     
            gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) 
            gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) 
            gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) 
            """
            
            accel_temp[0] = self.conv(data[0], data[1])
            accel_temp[1] = self.conv(data[2], data[3])
            accel_temp[2] = self.conv(data[4], data[5])
            gyro_temp[0]  = self.conv(data[6], data[7])
            gyro_temp[1]  = self.conv(data[8], data[9])
            gyro_temp[2]  = self.conv(data[10], data[11])
            #print(data)
            print(gyro_temp)
            
            accel_bias[0] += accel_temp[0] # Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
            accel_bias[1] += accel_temp[1]
            accel_bias[2] += accel_temp[2]
            gyro_bias[0]  += gyro_temp[0]
            gyro_bias[1]  += gyro_temp[1]
            gyro_bias[2]  += gyro_temp[2]
            print(gyro_bias)
                    
        accel_bias[0] //= packet_count # Normalize sums to get average count biases
        accel_bias[1] //= packet_count
        accel_bias[2] //= packet_count
        gyro_bias[0]  //= packet_count
        gyro_bias[1]  //= packet_count
        gyro_bias[2]  //= packet_count

        print("avg gyro bias: ", gyro_bias)
        
        if(accel_bias[2] > 0): # accel_bias[2] > 0L
            accel_bias[2] -= accelsensitivity  # Remove gravity from the z-axis accelerometer bias calculation
        else:
            accel_bias[2] += accelsensitivity

        # Push gyro biases to hardware registers
        """
        print(ctypes.c_short(-gyro_bias[0]))
        print("hEEEEEYY", "{0:b}".format(-200))
        bin(-gyro_bias[0] & 0b1111111111111111)
        """

        # M: what is the shift and & 0xFF actually doing?
        # Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
        data = [None]*6
        """
        data[0] = (-gyro_bias[0]//4  >> 8) & 0xFF # Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
        data[1] = (-gyro_bias[0]//4)       & 0xFF # Biases are additive, so change sign on calculated average gyro biases
        data[2] = (-gyro_bias[1]//4  >> 8) & 0xFF
        data[3] = (-gyro_bias[1]//4)       & 0xFF
        data[4] = (-gyro_bias[2]//4  >> 8) & 0xFF
        data[5] = (-gyro_bias[2]//4)       & 0xFF
        """
        data[0]= ((-gyro_bias[2]//4) & 0b1111111111111111) >> 8
        data[1]= ((-gyro_bias[2]//4) & 0b1111111111111111) & 0xFF
        data[2]= ((-gyro_bias[1]//4) & 0b1111111111111111) >> 8
        data[3]= ((-gyro_bias[1]//4) & 0b1111111111111111) & 0xFF
        data[4]= ((-gyro_bias[0]//4) & 0b1111111111111111) >> 8
        data[5]= ((-gyro_bias[0]//4) & 0b1111111111111111) & 0xFF
        #reg_h = data >> 8
        #reg_l = data & 0xFF
        
        print("data: ", data)
        
        # Push gyro biases to hardware registers
        """
        self.bus.write_byte_data(self.MPU9250_ADDRESS, XG_OFFSET_H, data[0])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, XG_OFFSET_L, data[1])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, YG_OFFSET_H, data[2])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, YG_OFFSET_L, data[3])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, ZG_OFFSET_H, data[4])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, ZG_OFFSET_L, data[5])
        """
        
        
        # Output scaled gyro biases for display in the main program
        dest1 = [None]*3
        dest1[0] = float(gyro_bias[0])/float(gyrosensitivity)  
        dest1[1] = float(gyro_bias[1])/float(gyrosensitivity)
        dest1[2] = float(gyro_bias[2])/float(gyrosensitivity)

        return dest1

    # Another clibration function. Something seems to be wrong with the FIFO stuff in the other implementation...
    def calibrate_gyro2(self):
         # reset device
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x80) # Write a one to bit 7 reset bit toggle reset device
        sleep(0.1)

        # get stable time source Auto select clock source to be PLL gyroscope reference if ready 
        # else use the internal oscillator, bits 2:0 = 001
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_1, 0x01)
        self.bus.write_byte_data(self.MPU9250_ADDRESS, PWR_MGMT_2, 0x00)
        sleep(0.2)

        # Configure MPU6050 gyro and accelerometer for bias calculation
        self.bus.write_byte_data(self.MPU9250_ADDRESS, CONFIG, 0x01)      # Set low-pass filter to 188 Hz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, SMPLRT_DIV, 0x00)  # Set sample rate to 1 kHz
        self.bus.write_byte_data(self.MPU9250_ADDRESS, GYRO_CONFIG, 0x00)  # Set gyro full-scale to 250 degrees per second, maximum sensitivity

        gyrosensitivity  = 131   # = 131 LSB/degrees/sec    # 131*250 = 32750 (32767 is the max of an int16)
        gyro_bias = (0,0,0)

        samples = 500
        # do some settings: set to 1000 Hz?
        for i in range(samples):
            a = self.read_gyro()
            gyro_bias = tuple(map(operator.add, gyro_bias, a))
            sleep(0.001)
        print("Gyro bias: ", gyro_bias)    
        gyro_bias_avg = [int(x/samples) for x in gyro_bias] 
        data_join = '\t'.join(map(str, gyro_bias_avg))

        data = [None]*6

        # Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
        # Also do some bitmasking to get this to work in python.
        data[0]= ((-gyro_bias_avg[0]//4) & 0b1111111111111111) >> 8
        data[1]= ((-gyro_bias_avg[0]//4) & 0b1111111111111111) & 0xFF
        data[2]= ((-gyro_bias_avg[1]//4) & 0b1111111111111111) >> 8
        data[3]= ((-gyro_bias_avg[1]//4) & 0b1111111111111111) & 0xFF
        data[4]= ((-gyro_bias_avg[2]//4) & 0b1111111111111111) >> 8
        data[5]= ((-gyro_bias_avg[2]//4) & 0b1111111111111111) & 0xFF

        # Write to the registers.
        self.bus.write_byte_data(self.MPU9250_ADDRESS, XG_OFFSET_H, data[0])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, XG_OFFSET_L, data[1])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, YG_OFFSET_H, data[2])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, YG_OFFSET_L, data[3])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, ZG_OFFSET_H, data[4])
        self.bus.write_byte_data(self.MPU9250_ADDRESS, ZG_OFFSET_L, data[5])

        

            
    
